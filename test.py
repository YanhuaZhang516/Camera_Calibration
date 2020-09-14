import numpy as np
from camera_record import *

# Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()
# #config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
#
# # Align objects
# align_to = rs.stream.color
# align = rs.align(align_to)
#
# # Start streaming
# pipeline.start(config)
# #
# try:
#     while True:
#         # Wait for a coherent pair of frames: depth and color
#         frames = pipeline.wait_for_frames()
#         aligned_frames = align.process(frames)
#         depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()
#
# finally:
#     pipeline.stop()

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
                i = int(raw_input('enter the number of image:'))

                cv2.imwrite('new_cali/color_{}.png'.format(i), color_image)

                cv2.destroyWindow('realsense')

                return i
                #break

    finally:
        #stop streaming
        pipeline.stop()


#camera_record()



#i = camera_record()
img = 'new_cali/color_{}.png'.format(89)
images = glob.glob('new_cali/*.png')

print(img)
print(type(img))
print(images[-1])
print(type(images[-1]))