#!/usr/bin/python3

import cv2

def findCentroid(mask_original):
    """
    Create a mask with the largest blob of mask_original and return its centroid coordinates
    :param mask_original: Cv2 image - Uint8
    :return mask: Cv2 image - Bool
    :return centroid: List of 2 values
    """

    # Defining maximum area and mask label
    maxArea = 0
    maxLabel = 0

    # You need to choose 4 or 8 for connectivity type
    connectivity = 4

    # Perform the operation
    output = cv2.connectedComponentsWithStats(mask_original, connectivity, cv2.CV_32S)

    # Get the results
    # The first cell is the number of labels
    num_labels = output[0]

    # The second cell is the label matrix
    labels = output[1]

    # The third cell is the stat matrix
    stats = output[2]

    # The fourth cell is the centroid matrix
    centroids = output[3]

    # For each blob, find their area and compare it to the largest one
    for label in range(1, num_labels):
        # Find area
        area = stats[label, cv2.CC_STAT_AREA]

        # If the area is larger then the max area to date, replace it
        if area > maxArea:
            maxArea = area
            maxLabel = label

    # If there are blobs, the label is different than zero
    if maxLabel != 0:
        # Create a new mask and find its centroid
        mask = labels == maxLabel
        centroid = centroids[maxLabel]
    else:
        # If there are no blobs, the mask stays the same, and there are no centroids
        mask = mask_original
        centroid = None

    return mask, centroid