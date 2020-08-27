# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import glob
from chessboard import *
import sys

def camera_record():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # show images
            cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('realsense', color_image)

            if cv2.waitKey(1000) == 27:
                break

    finally:
        # stop streaming
        pipeline.stop()


if __name__== "__main__":

    #f = open('record.csv', 'w')
    # first_row = ['picture number',
    #              'D435i',
    #              'Opti_Track',
    #              'transformation']
    # with f:
    #     writer = csv.writer(f)
    #     writer.writerow(first_row)


    print("camera recording:\n"
          "press c: record the rgb picture from D345i\n"
          "press o: record the pictures from Opti-track system\n")

    c = sys.stdin.read(1)
    if c == "c":
        # camera_record()
        images = glob.glob('new_cali/*.png')

        if images is not None:
            i = 0
            for image in images:
                rotation_matrix, translation_vector = extrinsic_camera_calibration(image)
                objpoint = objp[10]  # unsure which point
                objposition = world_project_camera(objpoint, rotation_matrix, translation_vector)
                print("object position in camera coordiantion of image {} is:{}".format(i, objposition))

                with open('record.csv', 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([i, objposition])
                i += 1
