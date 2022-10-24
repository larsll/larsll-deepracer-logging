#!/usr/bin/env python3

import logging
import importlib
import sys
import time
from enum import IntEnum
from threading import Event
import traceback

import rclpy
from rclpy.node import Node
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
            'monitor_topic_type', 'NOT_USED',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        self.declare_parameter(
            'monitor_topic_timeout', 1,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self._monitor_topic_timeout = self.get_parameter('monitor_topic_timeout').value
        self._monitor_timeout_duration = Duration(seconds=self._monitor_topic_timeout)
        self._monitor_last_received = self.get_clock().now()

    def __enter__(self):

        # Subscription to monitor topic.
        self._main_cbg = ReentrantCallbackGroup()

        # Start the scan
        self._scan_timer = self.create_timer(1.0, callback=self._scan_for_monitor_cb,
                                             callback_group=self._main_cbg)

        # Start monitoring GC
        self._start_monitor_gc = self.create_guard_condition(callback=self._start_monitoring_cb,
                                                             callback_group=self._main_cbg)

        # Change monitor
        self._change_gc = self.create_guard_condition(callback=self._change_cb,
                                                      callback_group=self._main_cbg)

        self.get_logger().info('Node started. Scanning for {}.'.format(self._monitor_topic))

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
            self._start_monitor_gc.destroy()

            if self._timeout_check_timer is not None:
                self._timeout_check_timer.destroy()

            if self._monitor_node_sub is not None:
                self._monitor_node_sub.destroy()

        except:  # noqa E722
            self.get_logger().error(traceback.format_stack())
        finally:
            self.get_logger().info('Node cleanup done. Exiting.')

    def _scan_for_monitor_cb(self):
        """Method that is called by self._scan_timer to check if the monitor topic
        is available. If yes it will trigger the startup procedure.
        """
        self._state = NodeState.Scanning
        topics = self.get_publishers_info_by_topic(self._monitor_topic)

        for topic in topics:
            self._monitor_topic_type = topic.topic_type
            module_name, class_name = self._monitor_topic_type.replace('/', '.').rsplit(".", 1)
            type_module = importlib.import_module(module_name)
            self._monitor_topic_class = getattr(type_module, class_name)

            self._scan_timer.destroy()
            self._start_monitor_gc.trigger()

    def _start_monitoring_cb(self):
        """Method that is called by self._start_monitor_gc, triggered mainly by self._scan_for_monitor_cb
        to start the subscription, and start the self._timeout_check_timer.
        """

        # Create subscription on monitor
        self._monitor_node_sub = self.create_subscription(
            self._monitor_topic_class, self._monitor_topic, self._receive_monitor_callback, 1,
            callback_group=self._main_cbg, raw=True)

        # Check if timeout receiver thread
        self._timeout_check_timer = self.create_timer(
            self._monitor_topic_timeout/10, callback=self._timeout_check_timer_cb, callback_group=self._main_cbg)

        self.get_logger().info('Monitoring {} of type {} with timeout {} seconds'.format(self._monitor_topic,
                               self._monitor_topic_type, self._monitor_topic_timeout))

        self._state = NodeState.Running

    def _receive_monitor_callback(self, msg):
        """Permanent method that will receive commands via serial
        """
        try:
            self._monitor_last_received = self.get_clock().now()
            if self._target_edit_state == RecordingState.Stopped:
                self._target_edit_state = RecordingState.Running
                self._change_gc.trigger()
                self.get_logger().info("Got callback from {}. Triggering start.". format(self._monitor_topic))

            elif self._target_edit_state == RecordingState.Running:
                self._bag_writer.write(self._monitor_topic, msg, self.get_clock().now().nanoseconds)

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

        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')

        self._bag_writer = rosbag2_py.SequentialWriter()
        self._bag_writer.open(storage_options, converter_options)

        self.create_topic(self._bag_writer, self._monitor_topic, self._monitor_topic_type)

    def _stop_bag(self):

        del self._bag_writer

    def create_topic(self, writer, topic_name, topic_type, serialization_format='cdr'):
        """
        Create a new topic.
        :param writer: writer instance
        :param topic_name:
        :param topic_type:
        :param serialization_format:
        :return:
        """
        topic_name = topic_name
        topic = rosbag2_py.TopicMetadata(name=topic_name, type=topic_type,
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
