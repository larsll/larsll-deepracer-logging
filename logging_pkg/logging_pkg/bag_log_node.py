#!/usr/bin/env python3

import logging
import importlib
import sys
import time
from enum import IntEnum
from threading import Event
from typing import List, Tuple
import traceback

import rclpy
from rclpy.node import Node, Subscription, TopicEndpointInfo, QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import rosbag2_py
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from logging_pkg.constants import (RecordingState)


class NodeState(IntEnum):
    """ Status of node
    Extends:
        Enum
    """
    Starting = 0
    Scanning = 1
    Running = 2
    Error = 3


class BagLogNode(Node):
    """ This node is used to log a topic to a rosbag2.
    """
    _shutdown = Event()
    _target_edit_state = RecordingState.Stopped
    _state = NodeState.Starting

    _subscriptions: List[Subscription] = []
    _topics_type_info: List[Tuple[str, TopicEndpointInfo]] = []
    _topics_to_scan: List[str] = []

    def __init__(self):
        super().__init__('bag_log_node')

        self.declare_parameter(
            'output_path', 'deepracer-bag-{}',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._output_path = self.get_parameter('output_path').value

        self.declare_parameter(
            'monitor_topic', '/inference_pkg/rl_results',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._monitor_topic = self.get_parameter('monitor_topic').value

        self.declare_parameter(
            'monitor_topic_timeout', 1,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self._monitor_topic_timeout = self.get_parameter('monitor_topic_timeout').value
        self._monitor_timeout_duration = Duration(seconds=self._monitor_topic_timeout)
        self._monitor_last_received = self.get_clock().now()

        self.declare_parameter(
            'log_topics', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self._log_topics = self.get_parameter('log_topics').value

        self._topics_to_scan += self._log_topics
        if self._topics_to_scan.count(self._monitor_topic) > 0:
            self._topics_to_scan.remove(self._monitor_topic)

    def __enter__(self):

        # Subscription to monitor topic.
        self._main_cbg = ReentrantCallbackGroup()

        # Start the scan
        self._state = NodeState.Scanning
        self._scan_timer = self.create_timer(1.0, callback=self._scan_for_topics_cb,
                                             callback_group=self._main_cbg)

        # Change monitor
        self._change_gc = self.create_guard_condition(callback=self._change_cb,
                                                      callback_group=self._main_cbg)

        self.get_logger().info('Node started. Monitor \'{}\'. Additionally logging {}.'
                               .format(self._monitor_topic, str(self._topics_to_scan)))

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'.format(ExcType.__name__))

        try:
            if self._target_edit_state == RecordingState.Running:
                self._stop_bag()

            self._shutdown.set()
            self._change_gc.destroy()

            if hasattr(self, '_timeout_check_timer'):
                self._timeout_check_timer.destroy()

            for sub in self._subscriptions:
                sub.destroy()

        except:  # noqa E722
            self.get_logger().error(traceback.format_stack())
        finally:
            self.get_logger().info('Node cleanup done. Exiting.')

    def _scan_for_topics_cb(self):
        """Method that is called by self._scan_timer to check if the monitor topic
        is available. If yes it will trigger the startup procedure.
        """

        try:
            if self._state == NodeState.Scanning:
                topic_endpoints: List[TopicEndpointInfo] = self.get_publishers_info_by_topic(self._monitor_topic)

                for topic_endpoint in topic_endpoints[:1]:
                    self._create_subscription(self._monitor_topic, topic_endpoint)

                    # Check if timeout receiver thread
                    self._timeout_check_timer = self.create_timer(
                        self._monitor_topic_timeout/10, callback=self._timeout_check_timer_cb,
                        callback_group=self._main_cbg)

                    self.get_logger().info('Monitoring {} of type {} with timeout {} seconds'.format(
                                           self._monitor_topic, topic_endpoint.topic_type, self._monitor_topic_timeout))

                    self._state = NodeState.Running

            for scan_topic in self._topics_to_scan[:]:
                topic_endpoints: List[TopicEndpointInfo] = self.get_publishers_info_by_topic(scan_topic)

                for topic_endpoint in topic_endpoints[:1]:
                    self._create_subscription(scan_topic, topic_endpoint)

                    self._topics_to_scan.remove(scan_topic)
                    self.get_logger().info('Logging {} of type {}.'.format(scan_topic,
                                           topic_endpoint.topic_type))

            if self._state == NodeState.Running and len(self._topics_to_scan) == 0:
                self.get_logger().info('All topics found. {} subscriptions active.'
                                       .format(len(self._subscriptions)))
                self._scan_timer.destroy()

        except:  # noqa E722
            self.get_logger().error(traceback.format_stack())

    def _create_subscription(self, topic_name, topic_endpoint: TopicEndpointInfo):
        module_name, class_name = topic_endpoint.topic_type.replace('/', '.').rsplit(".", 1)
        type_module = importlib.import_module(module_name)
        topic_class = getattr(type_module, class_name)

        topic_sub_qos = QoSProfile(depth=10)
        topic_sub_qos.reliability = topic_endpoint.qos_profile.reliability
        topic_sub_qos.history = HistoryPolicy.KEEP_ALL

        self._subscriptions.append(self.create_subscription(
            topic_class, topic_name,
            lambda msg: self._receive_monitor_callback(msg, topic=topic_name),
            qos_profile=topic_sub_qos,
            callback_group=self._main_cbg, raw=True))
        self._topics_type_info.append((topic_name, topic_endpoint))

    def _receive_monitor_callback(self, msg, topic):
        """ All messages are received in this single callback.
            As subscriptions are raw we cannot review the messages, they
            are directly written to the bag!
        """
        try:
            self.get_logger().debug("Got message on {}.". format(topic))
            time_recv = self.get_clock().now()

            if topic == self._monitor_topic:
                self._monitor_last_received = time_recv
                if self._target_edit_state == RecordingState.Stopped:
                    self._target_edit_state = RecordingState.Running
                    self._change_gc.trigger()
                    self.get_logger().info("Got callback from {}. Triggering start.". format(self._monitor_topic))

            # Check that we are running and that bag is open.
            if self._target_edit_state == RecordingState.Running and hasattr(self, '_bag_writer'):
                self._bag_writer.write(topic, msg, time_recv.nanoseconds)

        except Exception as e:  # noqa E722
            self.get_logger().error(traceback.format_stack())
            self.get_logger().error("{} occurred in _receive_monitor_callback.".format(sys.exc_info()[0]))

    def _timeout_check_timer_cb(self):
        try:
            dur_since_last_message = self.get_clock().now() - self._monitor_last_received

            if (dur_since_last_message > self._monitor_timeout_duration) and \
                    self._target_edit_state == RecordingState.Running:
                self._target_edit_state = RecordingState.Stopped
                self._change_gc.trigger()
                self.get_logger().info("Timeout. Triggering stop of recording.". format(self._monitor_topic))

        except:  # noqa E722
            self.get_logger().error("{} occurred in _timeout_check_timer_cb.".format(sys.exc_info()[0]))

    def _change_cb(self):
        """Guard condition trigger callback.
        """
        if not self._shutdown.is_set():
            try:
                self.get_logger().info("Changing state to {}"
                                       .format(self._target_edit_state.name))
                if self._target_edit_state == RecordingState.Running:
                    self._start_bag()
                else:
                    self._stop_bag()
            except:  # noqa E722
                self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))

    def _start_bag(self):

        serialization_format = 'cdr'

        if "{}" in self._output_path:
            bag_path = self._output_path.format(time.strftime("%Y%m%d-%H%M%S"))
        else:
            bag_path = self._output_path

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format)

        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')

        self._bag_writer = rosbag2_py.SequentialWriter()
        self._bag_writer.open(storage_options, converter_options)

        for topic_type_info in self._topics_type_info:
            self.create_topic(self._bag_writer, topic_type_info[0], topic_type_info[1])

    def _stop_bag(self):

        del self._bag_writer

    def create_topic(self, writer, topic_name, topic_type_info: TopicEndpointInfo, serialization_format='cdr'):
        """
        Create a new topic.
        :param writer: writer instance
        :param topic_name:
        :param topic_type:
        :param serialization_format:
        :return:
        """
        topic_name = topic_name
        topic = rosbag2_py.TopicMetadata(name=topic_name, type=topic_type_info.topic_type,
                                         serialization_format=serialization_format)

        writer.create_topic(topic)


def main(args=None):

    try:
        rclpy.init(args=args)
        with BagLogNode() as bag_log_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(bag_log_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        bag_log_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
