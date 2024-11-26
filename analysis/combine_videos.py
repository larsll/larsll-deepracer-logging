#!/usr/bin/env python3

import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import argparse
import fnmatch

def get_video_files(directory: str, pattern: str) -> dict:
    """
    Get a dictionary of video files grouped by prefix and date, filtered by a pattern.

    Args:
        directory (str): The directory containing the video files.
        pattern (str): The pattern to filter video files.

    Returns:
        dict: A dictionary where keys are tuples of (prefix, date) and values are lists of video file paths.
    """
    video_files = {}
    for file in os.listdir(directory):
        if file.endswith(".mp4") and fnmatch.fnmatch(file, pattern):
            parts = file.split('-')
            prefix = '-'.join(parts[:-2])
            date = parts[-2]
            key = (prefix, date)
            if key not in video_files:
                video_files[key] = []
            video_files[key].append(os.path.join(directory, file))
    
    for key in video_files:
        video_files[key].sort()

    return video_files

def create_divider_frame(width: int, height: int, prefix: str, date_time: str, background_path: str, font_path_bd: str, font_path_rg: str) -> np.ndarray:
    """
    Create a divider frame with the specified text and background.

    Args:
        width (int): The width of the frame.
        height (int): The height of the frame.
        prefix (str): The prefix to display on the first line.
        date_time (str): The date and time to display on the second line.
        background_path (str): The path to the background image.
        font_path_bd (str): The path to the bold font.
        font_path_rg (str): The path to the regular font.

    Returns:
        np.ndarray: The created divider frame.
    """
    background = cv2.imread(background_path)
    background = cv2.resize(background, (width, height))

    # Convert the background to a PIL image
    background_pil = Image.fromarray(cv2.cvtColor(background, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(background_pil)

    # Load custom fonts
    font_bd = ImageFont.truetype(font_path_bd, 60)
    font_rg = ImageFont.truetype(font_path_rg, 45)

    # Calculate text positions using textbbox
    text_bbox_prefix = draw.textbbox((0, 0), prefix, font=font_bd)
    text_x_prefix = (width - (text_bbox_prefix[2] - text_bbox_prefix[0])) // 2
    text_y_prefix = (height // 2) - 60

    # Format the date_time as YYYY-MM-DD HH:MM:ss
    date_time_formatted = f"{date_time[:4]}-{date_time[4:6]}-{date_time[6:8]} {date_time[9:11]}:{date_time[11:13]}:{date_time[13:15]}"

    text_bbox_date_time = draw.textbbox((0, 0), date_time_formatted, font=font_rg)
    text_x_date_time = (width - (text_bbox_date_time[2] - text_bbox_date_time[0])) // 2
    text_y_date_time = (height // 2) + 20

    # Put text on the background
    draw.text((text_x_prefix, text_y_prefix), prefix, font=font_bd, fill=(255, 255, 255))
    draw.text((text_x_date_time, text_y_date_time), date_time_formatted, font=font_rg, fill=(255, 255, 255))

    # Convert the PIL image back to a NumPy array
    background = cv2.cvtColor(np.array(background_pil), cv2.COLOR_RGB2BGR)

    return background

def combine_videos(video_files: list, output_file: str, background_path: str, font_path_bd: str, font_path_rg: str, codec: str = "avc1") -> None:
    """
    Combine multiple video files into a single video file.

    Args:
        video_files (list): A list of video file paths to combine.
        output_file (str): The path to the output video file.
        background_path (str): The path to the background image for dividers.
        font_path_bd (str): The path to the bold font.
        font_path_rg (str): The path to the regular font.
        codec (str): The codec for the video writer.
    """
    if not video_files:
        print("No video files to combine.")
        return

    print(f"Starting new video file: {output_file}")

    # Get the properties of the first video
    cap = cv2.VideoCapture(video_files[0])
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()

    # Create a video writer
    fourcc = cv2.VideoWriter_fourcc(*codec)
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    for video_file in video_files:
        # Extract prefix and date_time from the filename
        parts = os.path.basename(video_file).split('-')
        prefix = '-'.join(parts[:-2])
        date_time = '-'.join(parts[-2:]).replace('.mp4', '')

        cap = cv2.VideoCapture(video_file)
        duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / fps
        if duration < 20:
            print(f"Skipping {video_file} (duration: {duration:.2f} seconds)")
            cap.release()
            continue
        else:
            print(f"Adding video: {video_file}")

        # Create and write divider frame
        divider_frame = create_divider_frame(width, height, prefix, date_time, background_path, font_path_bd, font_path_rg)
        for _ in range(int(fps * 1.5)):  # Display the divider for 1.5 second
            out.write(divider_frame)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            out.write(frame)
        cap.release()

    out.release()
    print(f"Finished video file: {output_file}")

def main():
    parser = argparse.ArgumentParser(description="Combine videos with the same prefix created on the same day.")
    parser.add_argument("--codec", help="The codec for the video writer", default="avc1")
    parser.add_argument("--input_dir", help="The directory containing the video files", required=True)
    parser.add_argument("--output_dir", help="The directory to save the combined videos", required=True)
    parser.add_argument("--background", help="The path to the background image for dividers", default=None)
    parser.add_argument("--pattern", help="Pattern to filter video files", default="*.mp4")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))

    if args.background is None:
        args.background = os.path.join(script_dir, "resources", "AWS-Deepracer_Background_Machine-Learning.928f7bc20a014c7c7823e819ce4c2a84af17597c.jpg")

    font_path_bd = os.path.join(script_dir, "resources", "Amazon_Ember_Bd.ttf")
    font_path_rg = os.path.join(script_dir, "resources", "Amazon_Ember_Rg.ttf")

    video_files_dict = get_video_files(args.input_dir, args.pattern)
    for (prefix, date), video_files in video_files_dict.items():
        output_file = os.path.join(args.output_dir, f"{prefix}-{date}.mp4")
        combine_videos(video_files, output_file, args.background, font_path_bd, font_path_rg, codec=args.codec)

if __name__ == "__main__":
    main()
