#!/usr/bin/env python3.6

from PIL import Image
from numpy import asarray
import numpy as np
# sample.png is the name of the image
# file and assuming that it is uploaded
# in the current directory or we need
# to give the path
image = Image.open('depth_clamp.png')
  
# summarize some details about the image
print(image.format)
print(image.size)
print(image.mode)

np_image = asarray(image).astype(float)
print(np_image)
np.save("depth_clamp", np_image)