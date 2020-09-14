import pyrealsense2 as rs
# import numpy as np
# import cv2
# import glob
from chessboard import *
import sys
from optitrack_to_tf import *
import ast

def camera_record():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

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

            if cv2.waitKey(1) & 0xFF == ord('q'):

                i = raw_input('enter the number of image:')

                cv2.imwrite('new_cali/color_{}.png'.format(int(i)), color_image)

                cv2.destroyWindow('realsense')
                return i

    finally:
        # stop streaming
        pipeline.stop()


if __name__ == "__main__":

    # print("camera recording:\n"
    #       "press c: record the rgb picture from D345i\n"
    #       "press o: record the pictures from Opti-track system\n")
    #
    #
    # c = sys.stdin.read(1)

    while True:
        try:
            c = raw_input("camera recording:\n"
                          "press c: record the rgb picture from D345i\n"
                          "press o: record the pictures from Opti-track system\n"
                          "press q: break the process\n")


            if c == "c":

                i = camera_record()
                print(i)
                images = glob.glob('new_cali/*.png')
                img = 'new_cali/color_{}.png'.format(i)

                # images = glob.glob('cali/*.png')

                t = raw_input("press a: generate the csv file from the all images file\n"
                              "press b: generate the csv file from the new image\n")

                if t == "a":
                    if images is not None:

                        for image in images:
                            rotation_matrix, translation_vector = extrinsic_camera_calibration(image)
                            objpoint = objp[10]  # unsure which point

                            if rotation_matrix is not None and translation_vector is not None:
                                objposition = world_project_camera(objpoint, rotation_matrix, translation_vector)

                                print("object position in camera coordiantion of image {} is:{}".format(
                                    image.split('/')[1],
                                    objposition))

                                with open('record_d435i.csv', 'a') as f:
                                    writer = csv.writer(f)
                                    writer.writerow([image.split('/')[1].split('_')[1].split('.')[0],
                                                     objposition])

                if t == "b":
                    rotation_matrix, translation_vector = extrinsic_camera_calibration(img)
                    objpoint = objp[10]  # unsure which point

                    if rotation_matrix is not None and translation_vector is not None:
                        objposition = world_project_camera(objpoint, rotation_matrix, translation_vector)

                        print("object position in camera coordiantion of image {} is:{}".format(
                            img.split('/')[1],
                            objposition))

                        with open('record_d435i.csv', 'a') as f:
                            writer = csv.writer(f)
                            writer.writerow([i,
                                             objposition])



            if c == "o":


                rospy.init_node('position_optitrack')
                data = rospy.wait_for_message('/tf', TFMessage, timeout=None)
                position = data.transforms[0].transform.translation
                opti_arr = []
                opti_arr.append(position.x)
                opti_arr.append(position.y)
                opti_arr.append(position.z)
                opti_arr = np.array(opti_arr)

                with open('record_optitrack.csv','a') as f:
                    writer = csv.writer(f)
                    writer.writerow([input('the number of picture:'), opti_arr])


            if c == 'q':
                break

        except:
            break











