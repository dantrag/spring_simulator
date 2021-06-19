import sys

import numpy as np
from skimage import io
import skimage.data as data
import skimage.segmentation as seg

from itertools import chain

def process_image(filename, n, top, left, bottom, right):
    # NOTE: for the mask, only use pixels with > 128 red component
    def accept_pixel(rgb):
        return rgb[0] >= 128

    # read raw image
    image = io.imread(filename)
    height, width = image.shape[:2]
    mask = np.empty([height, width])
    if not top:
        top = 0
    if not bottom:
        bottom = height - 1
    if not left:
        left = 0
    if not right:
        right = width - 1    
    if top > bottom:
        top, bottom = bottom, top
    if left > right:
        left, right = right, left
    
    # try to increase number of segments until 50 or
    # until our largest segment inside the boundary if fully inside
    while True:
        # SLIC segmentation
        image_slic = seg.slic(image, n_segments = n, enforce_connectivity = True, start_label = 1)

        # count segment sizes within the boundary
        segment_sizes = [0] * n

        for y in range(top, bottom + 1):
            for x in range(left, right + 1):
                if accept_pixel(image[y, x]):
                    segment_sizes[image_slic[y, x]] += 1

        largest_segment = segment_sizes.index(max(segment_sizes))
        out_of_bound_pixels = 0

        for y in chain(range(top), range(bottom + 1, height)):
            for x in chain(range(left), range(right + 1, width)):
                if image_slic[y, x] == largest_segment:
                    out_of_bound_pixels += 1

        # do not accept segmentation that leads to segments extending beyond the boundary
        if out_of_bound_pixels > 0:
            # give up upon reaching 50 segments
            if n > 50:
                return (image, image_slic, mask)
            n += 1
            continue

        mask = np.zeros((height, width, 3), np.uint8)
        for y in range(height):
            for x in range(width):
                if image_slic[y, x] == largest_segment and accept_pixel(image[y, x]) and \
                   top <= y <= bottom and left <= x <= right:
                    mask[y, x] = [255, 255, 255]
                else:
                    mask[y, x] = [0, 0, 0]
        return (image, image_slic, mask)

if len(sys.argv) >= 2:
    input_filename = sys.argv[1]
    output_filename = "mask.png"
    top = None
    left = None
    bottom = None
    right = None
    if len(sys.argv) >= 3:
        output_filename = sys.argv[2]
        if len(sys.argv) >= 7:
            top = int(sys.argv[3])
            left = int(sys.argv[4])
            bottom = int(sys.argv[5])
            right = int(sys.argv[6])
        else:
            print("Warning: region of interest not specified, artefacts might appear")
    else:
        print("Warning: output filename is not specified, writing to " + output_filename)
        print("Warning: region of interest not specified, artefacts might appear")
    image, segmented_image, mask = process_image(input_filename, 10, top, left, bottom, right)
    io.imsave(output_filename, mask)
else:
    print("Error: no input file specified")
