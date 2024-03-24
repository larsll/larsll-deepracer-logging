#!/usr/bin/env python3

import logging
import importlib
import sys
import time
import os
from enum import Enum, IntEnum
from threading import Event, Lock
from typing import List, Tuple
import traceback

import rclpy
from rclpy.node import Node, Subscription, TopicEndpointInfo, QoSProfile
from rclpy.qos import HistoryPolicy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import rosbag2_py
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from std_msgs.msg import String

from deepracer_interfaces_pkg.srv import (USBFileSystemSubscribeSrv,
                                          USBMountPointManagerSrv)
from deepracer_interfaces_pkg.msg import USBFileSystemNotificationMsg

import logging_pkg.constants as constants
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


class LoggingMode(Enum):
    """ Status of node
    Extends:
        Enum
    """
    Never = 0
    USBOnly = 1
    Always = 2

    @classmethod
    def _missing_(cls, name_):
        for member in cls:
            if member.name.lower() == name_.lower():
                return member


class BagLogNode(Node):
    """This node is used to log a topic to a rosbag2.

    The `BagLogNode` class is responsible for logging a specified topic to a rosbag2 file.
    It provides functionality to monitor changes in the topic, start and stop the recording,
    and handle USB file system notifications.

    Attributes:
        _shutdown (Event): An event to signal the shutdown of the node.
        _target_edit_state (RecordingState): The target state of the recording (Running or Stopped).
        _state (NodeState): The current state of the node (Starting, Scanning, or Running).
        _topic_subscriptions (List[Subscription]): A list of topic subscriptions.
        _topics_type_info (List[Tuple[str, TopicEndpointInfo]]): A list of topic names and their type information.
        _topics_to_scan (List[str]): A list of topics to scan for.
        _bag_lock (Lock): A lock to ensure thread safety when accessing the rosbag2 file.

    """
    _shutdown = Event()
    _target_edit_state = RecordingState.Stopped
    _state = NodeState.Starting

    _topic_subscriptions: List[Subscription] = []
    _topics_type_info: List[Tuple[str, TopicEndpointInfo]] = []
    _topics_to_scan: List[str] = []
    _bag_lock = Lock()

    _usb_path = False

    def __init__(self):
        super().__init__('bag_log_node')

        self.declare_parameter(
            'output_path', constants.LOGS_DEFAULT_FOLDER,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._output_path = self.get_parameter('output_path').value

        self.declare_parameter(
            'disable_usb_monitor', False,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self._disable_usb_monitor = self.get_parameter('disable_usb_monitor').value

        self.declare_parameter(
            'logging_mode', "Always",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))      
        self._logging_mode = LoggingMode(self.get_parameter('logging_mode').value)

        self.declare_parameter(
            'monitor_topic', '/inference_pkg/rl_results',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._monitor_topic = self.get_parameter('monitor_topic').value

        self.declare_parameter(
            'file_name_topic', '/inference_pkg/model_name',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._file_name_topic = self.get_parameter('file_name_topic').value

        self.declare_parameter(
            'monitor_topic_timeout', 1,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self._monitor_topic_timeout = self.get_parameter('monitor_topic_timeout').value
        self._monitor_timeout_duration = Duration(seconds=self._monitor_topic_timeout)
        self._monitor_last_received = self.get_clock().now()

        self.declare_parameter(
            'log_topics', ['/ctrl_pkg/servo_msg'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self._log_topics = self.get_parameter('log_topics').value

        self._topics_to_scan += self._log_topics
        if self._topics_to_scan.count(self._monitor_topic) > 0:
            self._topics_to_scan.remove(self._monitor_topic)

        self._bag_name = constants.DEFAULT_BAG_NAME

    def __enter__(self):

        # Subscription to monitor topic.
        self._main_cbg = ReentrantCallbackGroup()

        # Start the scan
        if self._logging_mode == LoggingMode.Always:
            self._state = NodeState.Scanning
            self._scan_timer = self.create_timer(1.0, callback=self._scan_for_topics_cb,
                                                callback_group=self._main_cbg)

        # Monitor changes
        self._change_gc = self.create_guard_condition(callback=self._change_cb,
                                                      callback_group=self._main_cbg)

        self.get_logger().info('Node started. Mode {}. Monitor \'{}\'. Additionally logging {}.'
                               .format(self._logging_mode.name, self._monitor_topic, str(self._topics_to_scan)))

        # Create a subscription to the file name topic
        self._file_name_sub = self.create_subscription(String, self._file_name_topic,
                                                       self._file_name_cb, 1)

        if not self._disable_usb_monitor:
            # Client to USB File system subscription service that allows the node to add the "models"
            # folder to the watchlist. The usb_monitor_node will trigger notification if it finds
            # the files/folders from the watchlist in the USB drive.
            self._usb_sub_cb_group = ReentrantCallbackGroup()
            self._usb_file_system_subscribe_client = self.create_client(
                USBFileSystemSubscribeSrv, constants.USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME,
                callback_group=self._usb_sub_cb_group)
            while not self._usb_file_system_subscribe_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("File System Subscribe not available, waiting again...")

            # Client to USB Mount point manager service to indicate that the usb_monitor_node can safely
            # decrement the counter for the mount point once the action function for the file/folder being
            # watched by model_loader_node is succesfully executed.
            self._usb_mpm_cb_group = ReentrantCallbackGroup()
            self._usb_mount_point_manager_client = self.create_client(USBMountPointManagerSrv,
                                                                      constants.USB_MOUNT_POINT_MANAGER_SERVICE_NAME,
                                                                      callback_group=self._usb_mpm_cb_group)
            while not self._usb_mount_point_manager_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("USB mount point manager service not available, waiting again...")

            # Subscriber to USB File system notification publisher to recieve the broadcasted messages
            # with file/folder details, whenever a watched file is identified in the USB connected.
            self._usb_notif_cb_group = ReentrantCallbackGroup()
            self._usb_file_system_notification_sub = self.create_subscription(
                USBFileSystemNotificationMsg, constants.USB_FILE_SYSTEM_NOTIFICATION_TOPIC, self.
                _usb_file_system_notification_cb, 10, callback_group=self._usb_notif_cb_group)

            # Add the "models" folder to the watchlist.
            usb_file_system_subscribe_request = USBFileSystemSubscribeSrv.Request()
            usb_file_system_subscribe_request.file_name = constants.LOGS_SOURCE_LEAF_DIRECTORY
            usb_file_system_subscribe_request.callback_name = constants.LOGS_DIR_CB
            usb_file_system_subscribe_request.verify_name_exists = True
            self._usb_file_system_subscribe_client.call_async(usb_file_system_subscribe_request)

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

            for sub in self._topic_subscriptions:
                sub.destroy()

        except:  # noqa E722
            self.get_logger().error(traceback.format_stack())
        finally:
            self.get_logger().info('Node cleanup done. Exiting.')

    def _file_name_cb(self, filename_msg: String):
        try:
            if self._target_edit_state == RecordingState.Running:
                self._target_edit_state = RecordingState.Stopped
                self._change_gc.trigger()
                self.get_logger().info("Received new file name. Triggering stop of recording.")

            parts = filename_msg.data.split(os.sep)
            self._bag_name = parts[-2]
            self.get_logger().info(f"New filename received: {filename_msg.data} -> {self._bag_name}")

        except:  # noqa E722
            self.get_logger().error(traceback.format_stack())

    def _usb_file_system_notification_cb(self, notification_msg):
        """Callback for messages triggered whenever usb_monitor_node identifies a file/folder
           thats being tracked.

        Args:
            notification_msg (USBFileSystemNotificationMsg): Message containing information about the
                                                             file identified by the usb_monitor_node.
        """
        self.get_logger().info("File system notification:"
                               f" {notification_msg.path}"
                               f" {notification_msg.file_name}"
                               f" {notification_msg.node_name}"
                               f" {notification_msg.callback_name}")
        if notification_msg.file_name == constants.LOGS_SOURCE_LEAF_DIRECTORY and \
           notification_msg.callback_name == constants.LOGS_DIR_CB:
            self._output_path = os.path.join(notification_msg.path, notification_msg.file_name)
            self.get_logger().info(f"New output path: {self._output_path}")

        if self._logging_mode == LoggingMode.USBOnly and self._state == NodeState.Starting:
            self.get_logger().info("USB folder mounted, starting scanning for topics.")
            self._state = NodeState.Scanning
            self._scan_timer = self.create_timer(1.0, callback=self._scan_for_topics_cb,
                                                callback_group=self._main_cbg)
            
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
                                       .format(len(self._topics_type_info)))
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

        self._topic_subscriptions.append(self.create_subscription(
            topic_class, topic_name,
            lambda msg: self._receive_topic_callback(msg, topic=topic_name),
            qos_profile=topic_sub_qos,
            callback_group=self._main_cbg, raw=True))
        self._topics_type_info.append((topic_name, topic_endpoint))

    def _receive_topic_callback(self, msg, topic):
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
                self._bag_lock.acquire()
                self._bag_writer.write(topic, msg, time_recv.nanoseconds)
                self._bag_lock.release()

        except Exception as e:  # noqa E722
            self.get_logger().error(traceback.format_stack())
            self.get_logger().error("{} occurred in _receive_topic_callback.".format(sys.exc_info()[0]))

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

        self._bag_lock.acquire()

        serialization_format = 'cdr'

        bag_path = os.path.join(self._output_path, constants.LOGS_BAG_FOLDER_NAME_PATTERN.format(
            self._bag_name, time.strftime("%Y%m%d-%H%M%S")))
        if not os.path.exists(self._output_path):
            os.makedirs(self._output_path)

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format)

        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')

        self._bag_writer = rosbag2_py.SequentialWriter()
        self._bag_writer.open(storage_options, converter_options)

        for topic_type_info in self._topics_type_info:
            self.create_topic(self._bag_writer, topic_type_info[0], topic_type_info[1])

        self._bag_lock.release()

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
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
