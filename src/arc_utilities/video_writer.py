#!/usr/bin/env python

import cv2
import numpy as np


def save_video(image_seq, filepath, fps=15):
    shape = image_seq.shape
    if len(shape) == 4:
        size = shape[2], shape[1]

    out = cv2.VideoWriter(filepath, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
    for i, im in enumerate(image_seq):
        frame = cv2.cvtColor(np.array(im).astype(np.uint8), cv2.COLOR_BGR2RGB)
        out.write(frame)

