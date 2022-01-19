#!/usr/bin/env python

import numpy as np
import math
import sys
import cv2

# copied from common.transformations/camera.py
eon_focal_length = 910.0  # pixels
eon_dcam_focal_length = 860.0  # pixels

# adjust these for your specific camera specs if known
cam_width = 1280
cam_height = 720
cam_fov = 80
cam_focal_length = cam_width/(2*math.tan(cam_fov/2/180*math.pi))

# EON intrinsics
eon_intrinsics = np.array([
  [eon_focal_length,   0.,   1164/2.],
  [  0.,  eon_focal_length,  874/2.],
  [  0.,    0.,     1.]])

# TODO: implement driver cam
eon_dcam_intrinsics = np.array([
  [eon_dcam_focal_length,   0,   1152/2.],
  [  0,  eon_dcam_focal_length,  864/2.],
  [  0,    0,     1]])

final_warp = []

if __name__ == "__main__":
  in_cmd = sys.argv
  cam_number = 0

  if (len(in_cmd) < 3) or ("-h" in in_cmd)or ("--help" in in_cmd):
    print("USAGE: ./warp_vis.py [camera_number] [path_to_png.png]")
    print("-h or --help to display this dialogue")
    exit()
  else:
    print("CONTROLS:\n\
      q - zoom in\n\
      e - zoom out\n\
      w - shift up in\n\
      s - shift down\n\
      a - shift left\n\
      d - shift right\n\
      x - exit and print warp mesh\n\
      ")
    cam_number = in_cmd[1]
    background = cv2.imread(in_cmd[2])   

  cap = cv2.VideoCapture("/dev/video" + cam_number)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_height)

  while (True):

    k = cv2.waitKey(1)

    if (k == ord('x')):
      break
    if (k == ord('q')):
      cam_focal_length += 1
    if (k == ord('e')):
      cam_focal_length -= 1
    if (k == ord('a')):
      cam_width += 1
    if (k == ord('d')):
      cam_width -= 1
    if (k == ord('w')):
      cam_height += 1
    if (k == ord('s')):
      cam_height -= 1

    ret, img = cap.read()

    if ret:
      cam_intrinsics = np.array([
          [cam_focal_length,   0.,   cam_width/2.],
          [  0.,  cam_focal_length,  cam_height/2.],
          [  0.,    0.,   1]]) 
      rear_warp = np.dot(eon_intrinsics, np.linalg.inv(cam_intrinsics))

      cv2.imshow('original',img)
      img = cv2.warpPerspective(img, rear_warp, (1164,874), borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    #   img = cv2.addWeighted(background,0.8,img,1,0)
      cv2.imshow('preview', img)

  print("FINAL WARP: ")
  final_warp = rear_warp.flatten()

  # string format to make copying into camera_webcam.cc easier
  warp_string = "  float ts[9] = {"
  cnt_fmt = 0
  for item in final_warp:
    warp_string += str(item)
    cnt_fmt += 1
    if cnt_fmt != 9:
      if cnt_fmt % 3 == 0:
        warp_string += ",\n                "
      else:
        warp_string += ", "
  warp_string += "};"

  print(warp_string)
