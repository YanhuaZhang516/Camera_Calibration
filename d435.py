
import numpy as np
import cv2
import pyrealsense2 as rs
import glob

config = rs.config()
#rs.config.enable_device_from_file(config, bag_fname, repeat_playback=False)
pipeline = rs.pipeline()
# create a pipeline to easily configure and start the camera
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# start streaming and return the profile
profile = pipeline.start(config)

# try:
#     for i in range(0, 100):
#         frames = pipeline.wait_for_frames()
#         for f in frames:
#             print(f.profile)
#
# finally:
#    pipeline.stop()

frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()

# get intrinsics of the camera before alignment
depth_profile = depth.get_profile()
color_profile = color.get_profile()
cvsprofile = rs.video_stream_profile(color_profile)
dvsprofile = rs.video_stream_profile(depth_profile)

color_intrin = cvsprofile.get_intrinsics()
print(color_intrin)

# the print result: [width,height: 640x480  ppx,ppy: [309.876 238.268]
# fx, fy: [615.141 614.387]  Inverse Brown Conrady [0 0 0 0 0] ]

depth_intrin = dvsprofile.get_intrinsics()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(depth_scale)
#print(depth_intrin)
# the print result: [width, height: 640x480  ppx, ppy:[317.067 242.07]
# fx, fy: [388.373 388.373]  Brown Conrady [0 0 0 0 0] ]

extrin = depth_profile.get_extrinsics_to(color_profile)
print(extrin)
# rotation: [0.999996, 0.000990313, -0.00253084,
#            -0.00100375, 0.999985, -0.00531286,
#             0.00252554, 0.00531538, 0.999983]
# translation: [0.0149237, 0.00036078, 0.000310389]

# get intrinsics of the camera after the alignment

# get the pixel-coordination of the points
def get_pixel():

    # detect the corner of chessboard:
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # the size of the chessboard (we could set it arbitrary):
    width = 9
    height = 6
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((width*height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2) # we delete z-axis here to make it as two-dimensional matrix
    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('calib/*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # find the chess board corners, corner points in pixel space
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # if found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria) # refine the corners
            imgpoints.append(corners)

            # draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
            #cv2.imshow('findCorners', img)

            # find the rotation and translation vectors
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

            # draw the picture with the axis
            axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
            img2 = draw(img, corners=corners2, imgpts=axis)
            cv2.imshow('corners_3d', img2)

            cv2.waitKey(0)

    cv2.destroyAllWindows()

    return imgpoints, corners2

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img




def get_camera_3d(pixel_point, color_intrin, depth_scale):

    point_3d = np.zeros((3,1))
    x = (pixel_point[0] - color_intrin.ppx)/color_intrin.fx
    y = (pixel_point[1] - color_intrin.ppy)/color_intrin.fy

    # coeffs = color_intrin.coeffs
    #
    # r2 = x*x+y*y
    # f = 1 + coeffs[0]*r2 + coeffs[1]* r2*r2 + coeffs[4]*r2*r2*r2
    # ux = x*f + 2*coeffs[2]*x*y + coeffs[3]*(r2 + 2*x*x)
    # uy = y*f + 2*coeffs[3]*x*y + coeffs[2]*(r2 + 2*y*y)
    #
    # x = ux
    # y = uy

    point_3d[0] = depth_scale* x
    point_3d[1] = depth_scale*y
    point_3d[2] = depth_scale


    return point_3d

def get_opt_3d():

    return opt_3d_point

def transformation_cam_to_opt(camera_3d_point, opt_3d_point):

    return transformation_matrix


imagepoints, corners2 = get_pixel()
print(corners2)
pixel_coordi_center= imagepoints[0][25][0]
#pixel_coordi_center= [100,100]

