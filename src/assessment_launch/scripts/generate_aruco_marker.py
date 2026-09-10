"""
Generate a printable ArUco marker image, guaranteed to match the
dictionary used by aruco_pose_distance.py.

USAGE
    python generate_aruco_marker.py --id 0 --dict DICT_5X5_50 --size 600
"""

import argparse
import cv2
import numpy as np

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

parser = argparse.ArgumentParser()
parser.add_argument("--id", type=int, default=0, help="Marker ID to generate")
parser.add_argument("--dict", type=str, default="DICT_5X5_50", choices=list(ARUCO_DICTS.keys()))
parser.add_argument("--size", type=int, default=600, help="Output image size in pixels")
parser.add_argument("--out", type=str, default="marker.png")
args = parser.parse_args()

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[args.dict])
marker_img = np.zeros((args.size, args.size), dtype=np.uint8)
cv2.aruco.generateImageMarker(aruco_dict, args.id, args.size, marker_img, 1)

# Add a white quiet-zone border -- markers with no border are much harder
# to detect reliably, this is a common cause of dropouts too.
border = args.size // 5
bordered = cv2.copyMakeBorder(marker_img, border, border, border, border,
                               cv2.BORDER_CONSTANT, value=255)

cv2.imwrite(args.out, bordered)
print(f"[INFO] Saved {args.out} | dict={args.dict} | id={args.id} | size={args.size}px")
print("[INFO] Print this at a known physical size and pass that size (in meters) "
      "to --marker_length in aruco_pose_distance.py")
