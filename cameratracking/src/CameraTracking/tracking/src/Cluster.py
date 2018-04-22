from enum import Enum
import numpy as np

class ClusterParams(Enum):

    max_dist = 1
    min_leds = 2
    max_leds = 3
    timeout = 4

def search(idx, keypoints, visited, cluster):
    """
    Given a set of keypoints over the whole image, form clusters of neighbor LEDs.

    :param idx: Index of the LED under examination
    :param keypoints: Array of all the keypoints
    :param cluster: Array of arrays of points in which cluters will be placed
    :return: Nothing
    """
    if visited[idx] == 1:
        return
    point_a = keypoints[idx]
    cluster.append(keypoints[idx])
    for i in range(idx+1, len(keypoints)):
        if visited[i] == 1:
            continue
        point_b = keypoints[i]
        dist = point_b-point_a
        if np.linalg.norm(dist) < 100:
            search(i, keypoints, visited, cluster)
    visited[idx] = 1

def compute_clusters(keypoints, cluster_params):
    """
    Given an array of keypoints, find and return clusters.

    :param keypoints: Array of keypoints from the whole image.
    :return: An array of array of points, with neighboring points in each array.
    """
    visited = np.zeros(len(keypoints))
    clusters = []

    for i in range(0, len(visited)):
        if visited[i] == 0:
            cluster = []
            search(i, keypoints, visited, cluster)
            clusters.append(cluster)
    #an array of arrays of indices
    #the indices are of keypoints
    return clusters
