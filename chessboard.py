import numpy as np
import cv2
import pyrealsense2 as rs
import glob





def get_intrinsic_pipeline():
    # we could get the fixed intrinsic parameters of the realsense Camera from pyrealsense2 package

    config = rs.config()
    # rs.config.enable_device_from_file(config, bag_fname, repeat_playback=False)
    pipeline = rs.pipeline()
    # create a pipeline to easily configure and start the camera
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipeline.start(config)

    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()

    # get intrinsics of the camera before alignment
    depth_profile = depth.get_profile()
    color_profile = color.get_profile()
    cvsprofile = rs.video_stream_profile(color_profile)
    dvsprofile = rs.video_stream_profile(depth_profile)

    color_intrin = cvsprofile.get_intrinsics()

    # build color_intrinsic matrix:
    ppx = color_intrin.ppx
    ppy = color_intrin.ppy
    fx = color_intrin.fx
    fy = color_intrin.fy
    intrin_matrix = np.array([fx, 0, ppx], [0, fy, ppy], [0, 0, 1]).reshape((3, 3))
    disortion = color_intrin.coeffs

    return intrin_matrix, disortion


# set the camera matrix:

# detect the corner of chessboard:
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# the size of the chessboard (we could set it arbitrary):
width = 8
height = 6
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((width * height, 3), np.float32)
objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)  # we delete z-axis here to make it as two-dimensional matrix
# Arrays to store object points and image points from all the images.
images = glob.glob('cali/*.png')



def extrinsic_camera_calibration(fname):

    # in this part, we calculate the intrinsic and extrinsic parameters from cv2.cameraCalibration
    # of each pattern view

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

    # If found, add object points, image points (after refining them)
    if ret:

        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        # cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        # cv2.imshow('img', img)

    else:
        print("the picture from {} has problem".format(fnames))

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("the intrinsic matrix:", mtx)
    print("disortion:", dist)
    rvecs = cv2.Rodrigues(np.array(rvecs))[0]
    tvecs = np.array(tvecs).reshape(3,1)

    return rvecs, tvecs


def get_extrinsic_pnp(fname, mtx, dist):

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
    rotation_matrix = cv2.Rodrigues(np.array(rvecs))[0]

    return rotation_matrix, tvecs


def world_project_camera(objpoint, rotation_matrix, translation_vector):
    # convert the objpoint to Homogeneous matrix (4*1)
    objpoint = np.hstack((objpoint, np.array([1])))

    # combine the rotation_matrix and translation_vector to build the homogeneous extrinsic matrix (4*4)
    rotation_matrix = np.vstack((rotation_matrix, np.zeros((1, 3))))
    translation_matrix = np.vstack((translation_vector, np.array([1])))
    extrin = np.hstack((rotation_matrix, translation_matrix))

    campoint = extrin.dot(objpoint)
    return campoint[:3]


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


