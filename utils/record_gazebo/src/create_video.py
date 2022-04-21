import os
import re
import argparse
import moviepy.video.io.ImageSequenceClip as video

from datetime import datetime


DEFAULT_DIR = "/tmp/"
FPS = 10
NOW = datetime.now().strftime("%Y_%m_%d__%H_%M_%S")

def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument("--camera", "-c", default="camera_top")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    matcher = re.compile("\d+(?=(\.png|\.jpg))")

    image_folder = DEFAULT_DIR + args.camera

    image_files = [
        os.path.join(image_folder, image)
        for image in os.listdir(image_folder)
    ]
    image_files.sort(key=lambda i: int(matcher.search(i).group()))

    clip = video.ImageSequenceClip(image_files, fps=FPS)
    clip.write_videofile("../videos/" + args.camera + "-" + NOW + ".mp4")
