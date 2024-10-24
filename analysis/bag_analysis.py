import argparse
import datetime
import json
import os
from functools import partial
from multiprocessing import Pool, cpu_count
from typing import List, Dict, Tuple

import cv2

import matplotlib
import numpy as np
import pandas as pd
import rosbag2_py

from cv_bridge import CvBridge
from matplotlib.figure import Figure
from matplotlib import gridspec, pyplot as plt
from rclpy.serialization import deserialize_message
from tqdm import tqdm

from deepracer_viz.model.model import Model
from deepracer_viz.model.metadata import ModelMetadata
from deepracer_viz.gradcam.cam import GradCam
from deepracer_interfaces_pkg.msg import InferResultsArray

matplotlib.use("Agg")

bridge = CvBridge()
WIDTH = 1280
HEIGHT = 720


def get_rosbag_options(path: str, serialization_format: str = 'cdr') -> Tuple[rosbag2_py.StorageOptions, rosbag2_py.ConverterOptions]:
    """
    Get the ROS bag options for a given path and serialization format.

    Args:
        path (str): The path to the ROS bag file.
        serialization_format (str, optional): The serialization format to use. Defaults to 'cdr'.

    Returns:
        Tuple[rosbag2_py.StorageOptions, rosbag2_py.ConverterOptions]: A tuple containing the storage options and converter options.
    """

    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def create_plot(action_names: List[str], height: float, width: float, dpi: int, title: str) -> matplotlib.figure.Figure:
    """
    Create a plot with four subplots using matplotlib.

    Parameters:
    - action_names (list): A list of action names.
    - height (float): The height of the plot in inches.
    - width (float): The width of the plot in inches.
    - dpi (int): The resolution of the plot in dots per inch.

    Returns:
    - fig (Figure): The matplotlib Figure object containing the plot.
    """

    fig = plt.figure(figsize=(width/dpi, height/dpi), dpi=dpi)
    fig.set_facecolor('black')
    fig.suptitle(title, color='white', fontsize=20)
    
    x = list(range(0, len(action_names)))

    spec = gridspec.GridSpec(ncols=4, nrows=2,
                             width_ratios=[1, 1, 1, 1], wspace=0.1,
                             hspace=0.1, height_ratios=[3.5, 1], left=0.05, right=0.95, top=0.95, bottom=0.15)
    ax0 = fig.add_subplot(spec[0, :-2])
    ax0.set_xticks([])
    ax0.set_yticks([])
    for spine in ax0.spines.values():
        spine.set_edgecolor('white')      
        spine.set_linewidth(1.5)
    
    ax1 = fig.add_subplot(spec[0, -2:])
    ax1.set_xticks([])
    ax1.set_yticks([])
    for spine in ax1.spines.values():
        spine.set_edgecolor('white')      
        spine.set_linewidth(1.5)

    ax2 = fig.add_subplot(spec[1, :])
    ax2.set_ylim(0.0, 1.0)
    ax2.set_facecolor('black')
    for spine in ax2.spines.values():
        spine.set_edgecolor('white')      
        spine.set_linewidth(1.5)

    plt.xticks(x, action_names[::-1], rotation='vertical', color='white', fontsize=15)

    fig.canvas.draw()

    return fig

def create_img(
        step: Dict, bag_info: Dict, img: np.ndarray, grad_img: np.ndarray, action_names: List[str],
        height: int, width: int) -> np.ndarray:
    """
    Create an image with multiple plots and return it as a cv2 MatLike object.

    Args:
        step (Dict): A dictionary containing step information.
        bag_info (Dict): A dictionary containing information about the bag file.
        img (np.ndarray): The input image to be displayed in the first plot.
        grad_img (np.ndarray): The gradient image to be displayed in the second plot.
        action_names (List[str]): A list of action names.
        height (int): The height of the resulting image.
        width (int): The width of the resulting image.

    Returns:
        np.ndarray: The resulting image as a cv2 MatLike object.
    """

    timestamp_formatted = "{:02}:{:05.2f}".format(int(step['timestamp'] // 60), step['timestamp'] % 60)
    fig = create_plot(action_names, height, width, 72, title="{} - {} / {}".format(bag_info['name'], timestamp_formatted, step['seq_0']))

    x = list(range(0, len(action_names)))

    car_result = pd.DataFrame(step['car_results'])

    ax = fig.get_axes()

    for a in ax:
        for p in set(a.containers):
            p.remove()
        for i in set(a.images):
            i.remove()

    ax[0].imshow(img)
    ax[1].imshow(grad_img)
    # Highlight the highest bar in a different color
    bar_colors = ['#95d0fc'] * len(car_result['probability'])
    max_index = car_result['probability'].idxmax()
    bar_colors[max_index] = '#1f77b4'  # Darker tone similar to '#95d0fc'

    ax[2].bar(x, car_result['probability'][::-1], color=bar_colors[::-1])

    fig.canvas.draw()

    buf = fig.canvas.buffer_rgba()
    ncols, nrows = fig.canvas.get_width_height()
    plt.close(fig)
    img = cv2.cvtColor(np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 4), cv2.COLOR_RGBA2BGR)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Adjust the quality as needed
    _, encimg = cv2.imencode('.jpg', img, encode_param)
    return encimg


def process_data(data: bytes, start_time: float, seq: int, cam: GradCam) -> Tuple[Dict, np.ndarray, np.ndarray]:
    """
    Process data from a bag file.

    Args:
        data (bytes): The data to process.
        start_time (float): The start time of the data.
        start_seq (int): The number of the first frame of the data.
        cam (GradCam): The GradCam object used for image processing.

    Returns:
        Tuple[Dict, np.ndarray, np.ndarray]: A tuple containing the processed data, the original image,
        and the processed image.

    Raises:
        Exception: If an error occurs during the processing.

    """
    try:
        step = {}

        msg = deserialize_message(data, InferResultsArray)

        # Timestamp
        timestamp: float = (msg.images[0].header.stamp.sec + msg.images[0].header.stamp.nanosec / 1e9)
        timestamp = timestamp - start_time

        step['timestamp'] = timestamp
        step['seq'] = int(msg.images[0].header.frame_id)
        step['seq_0'] = seq

        # Extract original image from first camera
        cv_img = bridge.compressed_imgmsg_to_cv2(msg.images[0], desired_encoding="passthrough")
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2RGB)

        # Find best OpenVINO Result
        step['car_action'] = {'action': -1, 'probability': -1}
        step['car_results'] = []
        for r in msg.results:
            step['car_results'].append({'action': r.class_label, 'probability': r.class_prob})
            if r.class_prob > step['car_action']['probability']:
                step['car_action'] = {'action': r.class_label, 'probability': r.class_prob}

        # Process image with Tensorflow
        tf_result, grad_img = cam.process(cv_img)

        step['tf_action'] = {'action': -1, 'probability': -1}
        step['tf_results'] = []
        for i, r in enumerate(tf_result):
            step['tf_results'].append({'action': i, 'probability': r})
            if r > step['tf_action']['probability']:
                step['tf_action'] = {'action': i, 'probability': r}

        # Results
        step['results'] = []

        return step, cv_img, grad_img

    except Exception as e:
        print(e)


def get_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    """
    Returns a SequentialReader object for reading a ROS bag file.

    Parameters:
    - bag_path (str): The path to the ROS bag file.

    Returns:
    - reader (rosbag2_py.SequentialReader): The SequentialReader object for reading the bag file.
    """
    storage_options, converter_options = get_rosbag_options(bag_path)
    storage_filter = rosbag2_py.StorageFilter(topics=['/inference_pkg/rl_results'])

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(storage_filter)

    return reader


def analyze_bag(bag_path: str) -> Dict:
    """
    Analyzes a bag file and returns information about the bag.

    Args:
        bag_path (str): The path to the bag file.

    Returns:
        dict: A dictionary containing information about the bag file, including start time, FPS, total frames,
              step difference, elapsed time, action space size, and image shape.
    """

    bag_info = {}

    reader = get_reader(bag_path)

    first_stamp: float = -1
    steps_data = {'steps': []}

    s = 0

    while reader.has_next() and s < 60:
        step = {}

        (_, data, _) = reader.read_next()
        msg = deserialize_message(data, InferResultsArray)

        # Timestamp
        timestamp: float = (msg.images[0].header.stamp.sec + msg.images[0].header.stamp.nanosec / 1e9)

        if s == 0:
            first_stamp = timestamp

        step['timestamp'] = timestamp - first_stamp
        step['seq'] = int(msg.images[0].header.frame_id)
        steps_data['steps'].append(step)

        s += 1

    while reader.has_next():
        (_, _, _) = reader.read_next()
        s += 1

    df = pd.json_normalize(steps_data['steps'])
    del steps_data

    step_diff = df['seq'].max() - df['seq'].min()
    tmp_img = bridge.compressed_imgmsg_to_cv2(msg.images[0], desired_encoding="passthrough")

    bag_info['name'] = os.path.basename(bag_path)
    bag_info['start_time'] = first_stamp
    bag_info['fps'] = step_diff / df['timestamp'].max()
    bag_info['total_frames'] = s
    bag_info['step_diff'] = step_diff + 1
    bag_info['elapsed_time'] = df['timestamp'].max()
    bag_info['action_space_size'] = len(msg.results)
    bag_info['image_shape'] = tmp_img.shape

    print("Start time: {}".format(datetime.datetime.fromtimestamp(bag_info['start_time'])))
    print("Loaded {} steps from {}.".format(len(df.index), bag_info['step_diff']))
    print("Elapsed time: {:.2f} seconds".format(bag_info['elapsed_time']))
    print("Average FPS: {:.1f}".format(bag_info['fps']))
    print("Action Space: {} actions".format(bag_info['action_space_size']))
    print("Input image: {}x{}, {} channels.".format(
        bag_info['image_shape'][1],
        bag_info['image_shape'][0],
        bag_info['image_shape'][2]))
    print("Total messages: {}, expected duration: {:.1f}".format(
        bag_info['total_frames'], bag_info['total_frames']/bag_info['fps']))

    return bag_info


def main():
    """
    Analyzes a rosbag file and generates a video with annotated frames.

    This function takes command line arguments for the codec, rosbag file path, and model directory path.
    It performs the following steps:
    1. Checks if the model directory exists.
    2. Checks if the model metadata file and model file exist.
    3. Checks if the rosbag file exists.
    4. Loads the model metadata and extracts action names.
    5. Analyzes the rosbag file to get information about the frames.
    6. Creates a video writer with the specified codec and frame rate.
    7. Processes the data from the rosbag file, applies Grad-CAM, and creates annotated frames.
    8. Writes the annotated frames to the video file.
    9. Releases the video writer.
    10. Performs analysis on the recorded steps and prints the results.

    Args:
        None

    Returns:
        None
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--codec", help="The codec for the video writer", default="avc1")
    parser.add_argument("--bag_path", help="The path to the rosbag file", required=True)
    parser.add_argument("--model_path", help="The path to the model directory", required=True)
    parser.add_argument("--frame_limit", help="Max number of frames to process", default=None)
    parser.add_argument("--describe", help="Describe the actions", default=False)

    args = parser.parse_args()

    model_path = args.model_path
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model directory '{model_path}' does not exist.")

    metadata_json = os.path.join(model_path, 'model_metadata.json')
    model_pb = os.path.join(model_path, 'agent/model.pb')

    if not os.path.exists(metadata_json):
        raise FileNotFoundError(f"Model metadata file '{metadata_json}' does not exist.")

    if not os.path.exists(model_pb):
        raise FileNotFoundError(f"Model file '{model_pb}' does not exist.")

    bag_path = args.bag_path.rstrip('/')
    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Bag directory '{bag_path}' does not exist.")
    else:
        print(f"Processing bag file: {bag_path}")

    metadata = ModelMetadata.from_file(metadata_json)
    model = Model.from_file(model_pb_path=model_pb, metadata=metadata, log_device_placement=False)

    print("Using model: {}".format(model_path))
    print("")

    if args.frame_limit:
        frame_limit = float(args.frame_limit)
    else:
        frame_limit = float("inf")

    action_names = []
    with open(metadata_json, "r") as jsonin:
        model_metadata = json.load(jsonin)
    for action in model_metadata['action_space']:
        action_names.append(str(action['steering_angle']) + u'\N{DEGREE SIGN}' + " "+"%.1f" % action["speed"])
    bag_info = analyze_bag(bag_path)
    print("")
    print("Analysed file. Starting processing...")

    CODEC = args.codec
    output_file = "{}.mp4".format(bag_path)
    writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(
        *CODEC), int(round(bag_info['fps'], 0)), (WIDTH, HEIGHT))

    s = 0
    steps_data = {'steps': []}
    async_results = []

    try:
        with Pool(min(cpu_count()-2, 10)) as pool:
            with model.session as sess:
                cam = GradCam(model, model.get_conv_outputs())
                reader = get_reader(bag_path)

                pbar = tqdm(total=min(bag_info['total_frames'], frame_limit), desc="Loading messages", unit="messages")
                while reader.has_next() and s < frame_limit:
                    try:
                        (_, data, _) = reader.read_next()
                        step, img, grad_img = process_data(data, start_time=bag_info['start_time'], seq=s, cam=cam)
                        async_results.append(pool.apply_async(create_img, (step, bag_info, img, grad_img, action_names, HEIGHT, WIDTH)))
                        steps_data['steps'].append(step)
                        s += 1
                        pbar.update(1)
                    except Exception as e:
                        print(f"Error processing data: {e}")
                        raise

                pbar.close()

            pbar = tqdm(total=min(bag_info['total_frames'], frame_limit), desc="Writing image frames", unit="frames")
            for res in async_results:
                try:
                    encimg = res.get()
                    writer.write(cv2.imdecode(np.frombuffer(encimg, dtype=np.uint8), cv2.IMREAD_COLOR))
                    pbar.update(1)
                except Exception as e:
                    print(f"Error writing frame: {e}")
                    raise

            pbar.close()

        writer.release()
    except Exception as e:
        print(f"Unexpected error: {e}")
        raise

    df = pd.json_normalize(steps_data['steps'])
    del steps_data

    step_diff = df['seq'].max() - df['seq'].min()
    fps = step_diff / df['timestamp'].max()
    print("Start time: {}".format(datetime.datetime.fromtimestamp(bag_info['start_time'])))
    print("Loaded {} steps from {}.".format(len(df.index), step_diff + 1))
    print("Duration: {:.2f} seconds".format(df['timestamp'].max()))
    print("Average FPS: {:.1f}".format(fps))
    print("Action Space: {} actions".format(len(action_names)))
    
    df['action_agree'] = np.where(df['car_action.action'] == df['tf_action.action'], 1, 0)
    print("Car inference vs. Tensorflow: {} of {} in agreement".format(df['action_agree'].sum(), len(df.index)))
    
    if args.describe:
        df['action_diff'] = np.abs(df['car_action.action'] - df['tf_action.action'])
        action_analysis = df[['timestamp', 'seq', 'car_action.action', 'tf_action.action', 
                            'car_action.probability', 'tf_action.probability', 'action_agree', 'action_diff']]
        print(action_analysis.describe())

    print("")
    print("Created video file: {}".format(output_file))

if __name__ == "__main__":
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
    import tensorflow as tf
    tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
    main()
