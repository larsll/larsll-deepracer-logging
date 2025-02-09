# DeepRacer in-car Logging and Analysis

## Overview

The LarsLL DeepRacer logging ROS package creates the `bag_log_node`, which is an optional node for DeepRacer. It is used to monitor a specific topic and to write to a bag for later investigation.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

The recommended way to install this node is as part of the DeepRacer Custom Car ([deepracer-custom-car](https://github.com/aws-deepracer-community/deepracer-custom-car)) software stack.

## Launch files

The `logging_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the `bag_log_node`.

## Node details

### Functional Explanation

The `bag_log_node` is a ROS 2 node designed for logging specific topics to a rosbag2 file. This node is particularly useful for the AWS DeepRacer platform, where it can be used to monitor and log data for later analysis. The node provides functionality to start and stop logging based on certain conditions and handles USB file system notifications to manage log storage.

### Key Features

1. **Topic Monitoring and Logging**: The node monitors a specified topic and logs messages to a rosbag2 file. It can also log additional topics as configured.
2. **USB File System Notifications**: The node can handle notifications from the USB file system, allowing it to start logging when a USB drive is inserted and stop logging when it is removed.
3. **Service to Stop Logging**: The node provides a service to stop the logging process, which can be triggered externally.
4. **Configurable Parameters**: The node's behavior can be customized through various parameters, such as logging mode, monitored topics, and output paths.

### Key Considerations When Configuring `bag_log_node`

1. **Logging Mode**: The `logging_mode` parameter controls when logging should occur. It can be set to:
   - `Never`: Disables all logging.
   - `USBOnly`: Logs only when a USB stick is inserted.
   - `Always`: Logs both internally and to USB.

2. **Monitor Topic**: The `monitor_topic` parameter specifies the topic to monitor for inference results. The default is `/inference_pkg/rl_results`.

3. **Monitor Topic Timeout**: The `monitor_topic_timeout` parameter defines the timeout duration (in seconds) before a bag is closed if no message is published. This can be increased to avoid splitting of bags when inference is stopped due to a car crash. The default is 1 second.

4. **Log Topics**: The `log_topics` parameter is an array of additional topics to log. The default is `['/ctrl_pkg/servo_msg']`.

5. **File Name Topic**: The `file_name_topic` parameter specifies the topic where the name of the current model is published on load. This name is embedded in the bag name. The default is `/inference_pkg/model_name`.

6. **Output Path**: The `output_path` parameter defines the folder where the bag files are written. The default is `/opt/aws/deepracer/logs`.

7. **Disable USB Monitor**: The `disable_usb_monitor` parameter is a boolean flag to turn off communication with the `usb_monitor_pkg`. The default is `False`.

8. **Logging Provider**: The `logging_provider` parameter specifies the logging provider to use. The default is `sqlite3`.

### API

#### Parameters

| Parameter name   | Description  |
| ---------------- |  ----------- |
| `logging_mode` | Variable to control behaviour. Values are `Never` which disables all logging, `USBOnly` which only will write logs if a USB stick is inserted, or `Always` if logs will be written both internally and to USB. Default `Always`. | 
| `monitor_topic` | Name of the topic that will be monitored. Default `/inference_pkg/rl_results`|
| `monitor_topic_timeout` | Integer defining the timeout (in seconds) before a bag is closed if no message is published. Can be increased to 10-15 seconds to avoid splitting of bags when inference is stopped due to a car crash. Default is 1.|
| `log_topics` | Array of the topics that will be additionally logged. Default `[ '/ctrl_pkg/servo_msg' ]`. |
| `file_name_topic` | A topic where the name of the current model is published on load. This will be used to embed the model name in the bag name. Default `/inference_pkg/model_name`.|
| `output_path` | Path (folder) to which the bag-files are written. Default `/opt/aws/deepracer/logs`. | 
| `disable_usb_monitor` | Boolean to turn off the communication with the `usb_monitor_pkg`. Default `False`. | 
| `logging_provider` | The logging provider to use. Default is 'sqlite3'. |

#### Published services

| Service name | Service type | Description |
| -------------| -------------| ------------|
| `/bag_log_node/stop_logging` | `std_srvs/srv/Trigger` | Service to stop the logging process. |

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
| Configurable | Any | As part of the launch-file you will define which topic the node will subscribe to.|
| `/usb_monitor_pkg/usb_file_system_notification` | USBFileSystemNotificationMsg | Notifications about new USB folder. | 
| Value of `file_name_topic` | String | Simple string with the name of the Model. | 

#### Subscribed services

| Service name | Service type | Description |
| -------------| -------------| ------------|
| `/usb_monitor_pkg/usb_file_system_subscribe` | USBFileSystemSubscribeSrv | Call to register a subscription for a specific folder. In our case `logs`. |
| `/usb_monitor_pkg/usb_mount_point_manager` | USBMountPointManagerSrv | Defined service, currently unused. |


## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)

