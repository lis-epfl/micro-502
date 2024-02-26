# Crop image with four points and unwarp to square
# Select points: left-bottom, left-up, right-up, right-bottom
# Press ESC to quit
# Thanks to https://www.youtube.com/watch?v=1wUnPu4OvOA

import cv2
import numpy as np

input_image_name = 'floor_concrete_original.jpg'
output_image_name = 'floor_concrete.jpg'
scale = 1
output_image_w = 1024
output_image_h = 1024

origin_image = cv2.imread(input_image_name, cv2.IMREAD_COLOR)

downsize_w = round(origin_image.shape[0] / scale)
downsize_h = round(origin_image.shape[1] / scale)
downsize_image = cv2.resize(origin_image, (downsize_h, downsize_w))

selected_points = []

def mouse_callback(event, x, y, flags, param):
    global selected_points, downsize_image
    if event == cv2.EVENT_LBUTTONUP:
        selected_points.append([x * scale, y * scale])
        cv2.circle(downsize_image, (x, y), 10, (0, 255, 0), 3)

def select_points(image, points_num):
    global selected_points
    selected_points = []
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)
    while True:
        cv2.imshow('image', image)
        k = cv2.waitKey(1)
        if k == 27 or len(selected_points) == points_num:
            break
    cv2.destroyAllWindows()
    return np.array(selected_points, dtype=np.float32)

selected_points = select_points(downsize_image, 4)
print(selected_points)

output_points = np.array([[0, output_image_h], [0, 0], [output_image_w, 0], [output_image_w, output_image_h]], dtype=np.float32)
print(output_points)

perspective_m = cv2.getPerspectiveTransform(selected_points, output_points)
unwarped_image = cv2.warpPerspective(origin_image, perspective_m, (output_image_w, output_image_h))
cv2.imshow('origin', downsize_image)
cv2.imshow('output', unwarped_image)
cv2.imwrite(output_image_name, unwarped_image)
cv2.waitKey()
cv2.destroyAllWindows