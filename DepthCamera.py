import cv2
import numpy as np
from openni import openni2
import time

# 内参矩阵
k = np.array([[453.69, 0, 324.87],
              [0, 453.69, 239.5],
              [0, 0, 1]])


def pixel_to_camera(pixel_coord, depth, k_para):
    pixel_homogeneous = np.array([pixel_coord[0], pixel_coord[1], 1])
    K_inv = np.linalg.inv(k_para)
    normalized_coord = np.dot(K_inv, pixel_homogeneous)
    camera_coord = normalized_coord * depth
    return camera_coord


def coordinate_to_affine_matrix(coord):
    x1, y1, z1 = coord / 1000

    hand_eye_result = np.array([
        [-0.0155858, 0.9998631, 0.0055432, -0.3438],
        [0.9997793, 0.0155059, 0.0141765, -0.0628],
        [0.0140887, 0.0057629, -0.9998841, 0.9301569],
        [0, 0, 0, 1]
    ])
    affine_matrix = np.array([
        [1, 0, 0, x1],
        [0, 1, 0, y1],
        [0, 0, 1, z1],
        [0, 0, 0, 1]
    ])

    result_matrix = np.dot(hand_eye_result, affine_matrix)

    x_mo = result_matrix[0, 3]
    y_mo = result_matrix[1, 3]
    z_mo = result_matrix[2, 3]

    return x_mo, y_mo, z_mo


def process_mouse_event(x_para, y_para, dpt_para):
    z_para = dpt_para[y_para, 640 - x_para]
    xyz = pixel_to_camera(np.array([x_para, y_para]), z_para, k)
    res = coordinate_to_affine_matrix(xyz)
    print(f"Pixel: {x_para},{y_para} | Camera Coordinates: {xyz} | Result: {res}")


def mouse_call_back(event, x_para, y_para, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        z_para = dpt[y_para, 640 - x_para]
        xyz = pixel_to_camera(np.array([x_para, y_para]), z_para, k)
        res = coordinate_to_affine_matrix(xyz)
        print(f"Pixel: {x_para},{y_para} | Camera Coordinates: {xyz} | Result: {res}")


if __name__ == '__main__':
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    openni2.initialize()
    dev = openni2.Device.open_any()
    depth_stream = dev.create_depth_stream()
    depth_stream.start()

    cap = cv2.VideoCapture(1)
    cv2.namedWindow('depth')
    cv2.namedWindow('color')
    cv2.setMouseCallback('depth', mouse_call_back)
    cv2.setMouseCallback('color', mouse_call_back)
    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            break
        frame_data = np.array(depth_stream.read_frame().get_buffer_as_triplet()).reshape([480, 640, 2])
        dpt1 = np.asarray(frame_data[:, :, 0], dtype='float32')
        dpt2 = np.asarray(frame_data[:, :, 1], dtype='float32')
        dpt2 *= 255
        dpt = dpt1 + dpt2
        dpt = dpt.astype(np.uint16)
        dim_gray = cv2.convertScaleAbs(dpt, alpha=0.425)
        depth_colormap = cv2.applyColorMap(dim_gray, 0)
        depth_flip = cv2.flip(depth_colormap, 1)
        cv2.imshow('depth', depth_flip)
        cv2.imshow('color', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
