import argparse
import datetime
import os
from multiprocessing import Pool
from typing import List, Dict, Tuple

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # or any {'0', '1', '2'}

import cv2

import matplotlib
from matplotlib import gridspec, pyplot as plt
from matplotlib import font_manager as fm, rcParams

import numpy as np
import psutil
import pandas as pd

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from tqdm import tqdm

from deepracer_viz.model.model import Model
from deepracer_viz.model.metadata import ModelMetadata
from deepracer_viz.gradcam.cam import GradCam
from deepracer_interfaces_pkg.msg import InferResultsArray

import utils

matplotlib.use("Agg")

bridge = CvBridge()
WIDTH = 1280
HEIGHT = 720

# Define global color constants
COLOR_EDGE = '#a783e1'
COLOR_HIGHLIGHT = '#ff9900'
COLOR_TEXT_PRIMARY = '#a166ff'
COLOR_TEXT_SECONDARY = 'lightgray'
COLOR_BACKGROUND = 'black'


def create_plot(
        action_names: List[str],
        height: float, width: float, dpi: int, title: List[str],
        transparent: bool = False) -> matplotlib.figure.Figure:
    """
    Create a plot with four subplots using matplotlib.

    Parameters:
    - action_names (list): A list of action names.
    - height (float): The height of the plot in inches.
    - width (float): The width of the plot in inches.
    - dpi (int): The resolution of the plot in dots per inch.
    - title List(str): The titles of the plot.
    - transparent (bool): Whether the plot should have a transparent background.

    Returns:
    - fig (Figure): The matplotlib Figure object containing the plot.
    """

    script_dir = os.path.dirname(os.path.abspath(__file__))
    prop_big = fm.FontProperties(fname=os.path.join(script_dir, "resources", "Amazon_Ember_Rg.ttf"), size=20)
    prop_big_bd = fm.FontProperties(fname=os.path.join(script_dir, "resources", "Amazon_Ember_Bd.ttf"), size=25)
    prop_small = fm.FontProperties(fname=os.path.join(script_dir, "resources", "Amazon_Ember_Rg.ttf"), size=15)

    fig = plt.figure(figsize=(width/dpi, height/dpi), dpi=dpi)

    if transparent:
        fig.patch.set_alpha(0.0)
    else:
        fig.set_facecolor(COLOR_BACKGROUND)

    # Add left-aligned suptitle
    fig.text(0.025, 0.95, title[0], color=COLOR_TEXT_PRIMARY, fontproperties=prop_big_bd, ha='left')

    # Add right-aligned suptitle
    fig.text(0.975, 0.95, title[1], color=COLOR_TEXT_SECONDARY, fontproperties=prop_big, ha='right')

    x = list(range(0, len(action_names)))

    spec = gridspec.GridSpec(ncols=4, nrows=2,
                             width_ratios=[1, 1, 1, 1], wspace=0.1,
                             hspace=0.1, height_ratios=[3.5, 1.2], left=0.025, right=0.975, top=0.925, bottom=0.05)
    ax0 = fig.add_subplot(spec[0, :-2])
    ax0.set_xticks([])
    ax0.set_yticks([])
    for spine in ax0.spines.values():
        spine.set_edgecolor(COLOR_EDGE)
        spine.set_linewidth(1)

    ax1 = fig.add_subplot(spec[0, -2:])
    ax1.set_xticks([])
    ax1.set_yticks([])
    for spine in ax1.spines.values():
        spine.set_edgecolor(COLOR_EDGE)
        spine.set_linewidth(1)

    ax2 = fig.add_subplot(spec[1, :])
    ax2.set_ylim(0.0, 1.0)
    ax2.set_facecolor(COLOR_BACKGROUND)
    ax2.set_xticks([])
    ax2.set_yticks([])
    for spine in ax2.spines.values():
        spine.set_edgecolor(COLOR_EDGE)
        spine.set_linewidth(1)

    for i, label in enumerate(action_names[::-1]):
        ax2.text(i, 0.5, label, ha='center', va='center', rotation=90, color=COLOR_TEXT_SECONDARY, fontproperties=prop_small)
    
    return fig


def create_img(
        step: Dict, bag_info: Dict, img: np.ndarray, grad_img: np.ndarray, action_names: List[str],
        height: int, width: int, background: np.ndarray = None) -> np.ndarray:
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
    background (np.ndarray): The background image to use for the plot.

    Returns:
    np.ndarray: The resulting image as a cv2 MatLike object.
    """

    timestamp_formatted = "{:02}:{:05.2f}".format(int(step['timestamp'] // 60), step['timestamp'] % 60)
    transparent = background is not None

    # Split bag_info['name'] into parts
    name_parts = bag_info['name'].split('-')
    # Create one string with the last two parts
    start_time = '-'.join(name_parts[-2:])
    # Create another string with the rest
    model_name = '-'.join(name_parts[:-2])

    fig = create_plot(
        action_names, height, width, 72,
        title=[model_name, "{} {} / {}".format(start_time, timestamp_formatted, step['seq_0'])],
        transparent=transparent)

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
        bar_colors = [COLOR_EDGE] * len(car_result['probability'])
        max_index = car_result['probability'].idxmax()
        bar_colors[max_index] = COLOR_HIGHLIGHT

        ax[2].bar(x, car_result['probability'][::-1], color=bar_colors[::-1])

        fig.canvas.draw()

        buf = fig.canvas.buffer_rgba()

        ncols, nrows = fig.canvas.get_width_height()
        plt.close(fig)
        img = np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 4)

        if background is not None:
            gradient_alpha_rgb_mul, one_minus_gradient_alpha = utils.get_gradient_values(img)
            img = cv2.cvtColor(utils.apply_gradient(background, gradient_alpha_rgb_mul,
                                              one_minus_gradient_alpha), cv2.COLOR_RGBA2BGR)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

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

    reader = utils.get_reader(bag_path, topics=['/inference_pkg/rl_results'])

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
    bag_info['step_actual'] = len(df.index)
    bag_info['elapsed_time'] = df['timestamp'].max()
    bag_info['action_space_size'] = len(msg.results)
    bag_info['image_shape'] = tmp_img.shape
    
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
    parser.add_argument("--relative_labels",
                        help="Make labels relative, not fixed to value in action space", default=False, type=bool)
    parser.add_argument("--background", help="Add a background to the video", default=True, type=bool)

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

    if metadata.action_space_type == 'continuous':
        return NotImplementedError("Continuous action space not supported.")

    model = Model.from_file(model_pb_path=model_pb, metadata=metadata, log_device_placement=False)

    print("Using model: {}".format(model_path))
    print("")

    if args.frame_limit:
        frame_limit = float(args.frame_limit)
    else:
        frame_limit = float("inf")

    action_names = []
    action_space = metadata.action_space.action_space
    max_steering_angle = max(float(action['steering_angle']) for action in action_space)
    max_speed = max(float(action['speed']) for action in action_space)

    for action in action_space:
        if args.relative_labels:
            if float(action['steering_angle']) == 0.0:
                steering_label = "C"
            else:
                steering_label = "L" if float(action['steering_angle']) > 0 else "R"
            steering_value = abs(float(action['steering_angle']) * 100 / max_steering_angle)
            speed_value = float(action["speed"]) * 100 / max_speed
            action_names.append(f"{steering_label}{steering_value:.0f}% x {speed_value:.0f}%")
        else:
            action_names.append(str(action['steering_angle']) + u'\N{DEGREE SIGN}' + " "+"%.1f" % action["speed"])
    bag_info = analyze_bag(bag_path)
    utils.print_baginfo(bag_info)

    print("")
    print("Analysed file. Starting processing...")

    if args.background:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        background_path = os.path.join(
            script_dir, "resources",
            "AWS-Deepracer_Background_Machine-Learning.928f7bc20a014c7c7823e819ce4c2a84af17597c.jpg")
        background = utils.load_background_image(background_path, WIDTH, HEIGHT)
    else:
        background = None

    CODEC = args.codec
    output_file = "{}.mp4".format(bag_path)
    writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(
        *CODEC), int(round(bag_info['fps'], 0)), (WIDTH, HEIGHT))

    s = 0
    steps_data = {'steps': []}
    async_results = []

    try:
        with Pool(psutil.cpu_count(logical=False)-1) as pool:
            with model.session as sess:
                cam = GradCam(model, model.get_conv_outputs())
                reader = utils.get_reader(bag_path, topics=['/inference_pkg/rl_results'])

                pbar = tqdm(total=min(bag_info['total_frames'], frame_limit), desc="Loading messages", unit="messages")
                while reader.has_next() and s < frame_limit:
                    try:
                        (_, data, _) = reader.read_next()
                        step, img, grad_img = process_data(data, start_time=bag_info['start_time'], seq=s, cam=cam)
                        async_results.append(pool.apply_async(create_img, (step, bag_info, img,
                                             grad_img, action_names, HEIGHT, WIDTH, background)))
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