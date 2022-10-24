# LarsLL DeepRacer logging package

## Overview

The LarsLL DeepRacer logging ROS package creates the `bag_log_node`, which is an optional node for DeepRacer. It is used to monitor a specific topic and to write to a bag for later investigation.

It is intended to be built as a custom DeepRacer application stack. For more information about the application and the components, see the  [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).


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
| `monitor_topic` | Name of the topic that will be monitoried. Default `/inference_pkg/rl_results`|
| `monitor_topic_timeout` | Integer defining the timeout (in seconds) before a bag is closed. Default is 1.|
| `output_path` | Path to which the bag-files are written. Supports including `{}` to add timestamp to the bag directory. Default `deepracer-bag-{}`.| 

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|Configurable|Any|As part of the launch-file you will define which topic the node will subscribe to.|

#### Services

Not applicable. The node does not provide or call services.

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)

