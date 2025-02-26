import cv2
import numpy as np
from math import atan2


def pca(mask, scale_maj: float = 0.02, scale_min: float = 0.02):

    # h, w = mask.shape[:2]
    data_points = cv2.findNonZero(mask).sum(axis=1).astype(np.float32)

    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_points, mean)
    cntr = (int(mean[0, 0]), int(mean[0, 1]))

    # major
    maj_ax = (
        cntr[0] + scale_maj * eigenvectors[0, 0] * eigenvalues[0, 0],
        cntr[1] + scale_maj * eigenvectors[0, 1] * eigenvalues[0, 0],
    )

    # minor
    min_ax = (
        cntr[0] - scale_min * eigenvectors[1, 0] * eigenvalues[1, 0],
        cntr[1] - scale_min * eigenvectors[1, 1] * eigenvalues[1, 0],
    )

    angle = atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # radians
    # angle is to maj ax, "Up" negative "Down" positive

    return cntr, maj_ax, min_ax, angle


def pca2axescorners(cntr, maj_ax, min_ax, scale: float = 0.5):
    cntr = np.array(cntr)
    maj_ax = np.array(maj_ax)
    min_ax = np.array(min_ax)
    maj_vec = scale * (cntr - maj_ax)
    min_vec = scale * (cntr - min_ax)

    # this creates a rectangle around the axes
    # but i dont think this really works. i think what we want is to only consider one axis length?
    return np.array(
        [
            cntr + maj_vec + min_vec,
            cntr + maj_vec - min_vec,
            cntr - maj_vec + min_vec,
            cntr - maj_vec - min_vec,
        ]
    )


def pca_minorax2corners(cntr, min_ax, scale: float = 0.5):
    """Uses the centroid and minor axis from a PCA on an image blob
    to generate 4 visual servo corners

    Args:
        cntr (tuple): centroid from opencv PCA
        min_ax (tuple): minor axis from opencv PCA
        scale (float, optional): what percent of the minor axis length to extend from the centroid. Defaults to 0.5.

    Returns:
        np.ndarray: 4 corner points
    """
    cntr = np.array(cntr)
    min_ax = np.array(min_ax)
    min_vec = scale * (min_ax - cntr)
    min_orth_vec = np.array([-min_vec[1], min_vec[0]])

    # i think this is what we want: creates a square from the minor axis, oriented such that the major axis
    # points are in the order we want (at least from initial testing)
    return np.array(
        [
            cntr + min_orth_vec + min_vec,
            cntr + min_orth_vec - min_vec,
            cntr - min_orth_vec + min_vec,
            cntr - min_orth_vec - min_vec,
        ]
    )

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
  return result

if __name__ == "__main__":
    img = cv2.imread("oval1.jpg")
    img = cv2.flip(img, 0)
    img = rotate_image(img, 45)
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # cv2.imshow("img", img)
    # cv2.waitKey()
    # mask created with HSV thresholds
    filter_low_mask = (0, 0, 0)
    filter_high_mask = (240, 240, 240)
    mask = cv2.inRange(img, filter_low_mask, filter_high_mask)
    cntr, maj_ax, min_ax, angle = pca(mask)
    # points = pca_minorax2corners(cntr, min_ax)
    # print(angle)

    maskim = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.line(maskim, cntr, (int(maj_ax[0]), int(maj_ax[1])), (255, 0, 0), 4)
    cv2.line(maskim, cntr, (int(min_ax[0]), int(min_ax[1])), (0, 255, 0), 4)
    min_ax = np.array(min_ax)
    maj_ax = np.array(maj_ax)
    min_vec = 0.5 * (min_ax - cntr)
    min_orth_vec = np.array([-min_vec[1], min_vec[0]])
    # maj_vec = 0.5 * (maj_ax - cntr)

    points = np.array(
        [
            cntr + min_vec,
            cntr + min_orth_vec,
            cntr - min_vec,
            cntr - min_orth_vec,
        ]
    )

    for pt in points:
        print(pt[0])
        cv2.circle(maskim, (int(pt[0]), int(pt[1])), 2, (0, 0, 255), -1)
        cv2.imshow("mask", maskim)
        cv2.waitKey()
        cv2.destroyAllWindows()

    # cv2.imshow("mask", maskim)
    # cv2.waitKey()
