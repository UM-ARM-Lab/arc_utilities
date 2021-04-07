#!/usr/bin/env python

import cv2
import numpy as np
from pathlib import Path
import logging


def save_video(image_seq, filepath, fps=15):
    shape = image_seq.shape
    if len(shape) == 4:
        size = shape[2], shape[1]

    out = cv2.VideoWriter(filename=Path(filepath).as_posix(), fourcc=cv2.VideoWriter_fourcc(*'mp4v'), fps=fps,
                          frameSize=size)

    if np.all(image_seq <= 1.0):
        image_seq *= 255
        logging.info('The video has no pixels above 1.0. Assuming the video is not just black, so multiplying by 255')

    for i, im in enumerate(image_seq):
        frame = cv2.cvtColor(np.array(im).astype(np.uint8), cv2.COLOR_BGR2RGB)
        out.write(frame)
