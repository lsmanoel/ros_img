import numpy as np


def pretty_depth(depth):
    """Converts depth into a 'nicer' format for display

    This is abstracted to allow for experimentation with normalization

    Args:
        depth: A numpy array with 2 bytes per pixel

    Returns:
        A numpy array that has been processed whos datatype is unspecified
    """
    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)
    return depth


def pretty_depth_cv(depth):
    """Converts depth into a 'nicer' format for display

    This is abstracted to allow for experimentation with normalization

    Args:
        depth: A numpy array with 2 bytes per pixel

    Returns:
        An opencv image who's datatype is unspecified
    """
    import cv2
    depth = pretty_depth(depth)
    image = cv2.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv2.IPL_DEPTH_8U,
                                 1)
    cv2.SetData(image, depth.tostring(),
               depth.dtype.itemsize * depth.shape[1])
    return image


def video_cv(video):
    """Converts video into a BGR format for opencv

    This is abstracted out to allow for experimentation

    Args:
        video: A numpy array with 1 byte per pixel, 3 channels RGB

    Returns:
        An opencv image who's datatype is 1 byte, 3 channel BGR
    """
    import cv2
    video = video[:, :, ::-1]  # RGB -> BGR
    image = cv2.CreateImageHeader((video.shape[1], video.shape[0]),
                                 cv2.IPL_DEPTH_8U,
                                 3)
    cv2.SetData(image, video.tostring(),
               video.dtype.itemsize * 3 * video.shape[1])
    return image
