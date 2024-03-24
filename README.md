# LarsLL DeepRacer logging package

## Overview

The LarsLL DeepRacer logging ROS package creates the `bag_log_node`, which is an optional node for DeepRacer. It is used to monitor a specific topic and to write to a bag for later investigation.

It is intended to be built as a custom DeepRacer application stack. For more information about the application and the components, see the  [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

Unless `disable_usb_monitor` is set to `True` the package will also request the `usb_monitor_pkg` of the [aws-deepracer-usb-monitor-pkg](https://github.com/aws-deepracer/aws-deepracer-usb-monitor-pkg) to monitor USB for any device with a `logs` folder in the root. In the case one is found the logs will be written to this device.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

The recommended way to install this node as part of the DeepRacer application is to follow the instructions in [deepracer-scripts](https://github.com/davidfsmith/deepracer-scripts).

## Launch files

The `logging_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the `bag_log_node`.

## Node details

### bag_log_node

#### Parameters

| Parameter name   | Description  |
| ---------------- |  ----------- |
| `logging_mode` | Variable to control behaviour. Values are `Never` which disables all logging, `USBOnly` which only will write logs if a USB stick is inserted, or `Always` if logs will be written both internally and to USB. Default `Always`. | 
| `monitor_topic` | Name of the topic that will be monitoried. Default `/inference_pkg/rl_results`|
| `monitor_topic_timeout` | Integer defining the timeout (in seconds) before a bag is closed if no message is published. Can be increased to 10-15 seconds to avoid splitting of bags when inference is stopped due to a car crash. Default is 1.|
| `log_topics` | Array of the topics that will be additionally logged. Default `[ '/ctrl_pkg/servo_msg' ]`. |
| `file_name_topic` | A topic where the name of the current model is published on load. This will be used to embed the model name in the bag name. Default `/inference_pkg/model_name`.|
| `output_path` | Path (folder) to which the bag-files are written. Default `/opt/aws/deepracer/logs`. | 
| `disable_usb_monitor` | Boolean to turn off the communication with the `usb_monitor_pkg`. Default `False`. | 

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
| Configurable | Any | As part of the launch-file you will define which topic the node will subscribe to.|
| `/usb_monitor_pkg/usb_file_system_notification` | USBFileSystemNotificationMsg | Notifications about new USB folder. | 
| Value of `file_name_topic` | String | Simple string with the name of the Model. | 

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
| `/usb_monitor_pkg/usb_file_system_subscribe` | USBFileSystemSubscribeSrv | Call to register a subscription for a specific folder. In our case `logs`. |
| `/usb_monitor_pkg/usb_mount_point_manager` | USBMountPointManagerSrv | Defined service, currently unused. |


## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)

