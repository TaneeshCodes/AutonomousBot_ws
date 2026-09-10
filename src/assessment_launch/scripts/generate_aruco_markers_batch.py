"""
Batch-generate multiple ArUco marker images, each with a proper white
quiet-zone border (needed for reliable detection).

USAGE
    python generate_aruco_markers_batch.py
    python generate_aruco_markers_batch.py --count 15 --dict DICT_5X5_50
    python generate_aruco_markers_batch.py --start_id 0 --count 10 --size 600 --out_dir markers
"""

import argparse
import os

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


def parse_args():
    parser = argparse.ArgumentParser(description="Batch-generate ArUco markers")
    parser.add_argument("--dict", type=str, default="DICT_5X5_50",
                         choices=list(ARUCO_DICTS.keys()),
                         help="ArUco dictionary to use (default: DICT_5X5_50)")
    parser.add_argument("--start_id", type=int, default=0,
                         help="First marker ID to generate (default: 0)")
    parser.add_argument("--count", type=int, default=10,
                         help="How many markers to generate (default: 10)")
    parser.add_argument("--size", type=int, default=600,
                         help="Marker image size in pixels, before border (default: 600)")
    parser.add_argument("--border_ratio", type=float, default=0.2,
                         help="White border thickness as a fraction of --size (default: 0.2)")
    parser.add_argument("--out_dir", type=str, default="aruco_markers",
                         help="Output directory (default: ./aruco_markers)")
    parser.add_argument("--sheet", action="store_true",
                         help="Also generate a single contact-sheet image with all "
                              "markers laid out in a grid, for one-page printing.")
    return parser.parse_args()


def generate_marker(aruco_dict, marker_id, size, border_ratio):
    marker_img = np.zeros((size, size), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, size, marker_img, 1)

    border = int(size * border_ratio)
    bordered = cv2.copyMakeBorder(
        marker_img, border, border, border, border,
        cv2.BORDER_CONSTANT, value=255
    )

    # Label under the marker so printed sheets are easy to identify by hand
    label_h = int(size * 0.15)
    labeled = cv2.copyMakeBorder(bordered, 0, label_h, 0, 0,
                                  cv2.BORDER_CONSTANT, value=255)
    text = f"ID {marker_id}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = size / 400.0
    thickness = max(1, size // 200)
    (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
    text_x = (labeled.shape[1] - text_w) // 2
    text_y = labeled.shape[0] - (label_h - text_h) // 2
    cv2.putText(labeled, text, (text_x, text_y), font, font_scale, 0, thickness)

    return labeled


def build_contact_sheet(images, cols=5):
    rows = int(np.ceil(len(images) / cols))
    h, w = images[0].shape[:2]
    pad = 20

    sheet_h = rows * h + (rows + 1) * pad
    sheet_w = cols * w + (cols + 1) * pad
    sheet = np.full((sheet_h, sheet_w), 255, dtype=np.uint8)

    for idx, img in enumerate(images):
        r, c = divmod(idx, cols)
        y = pad + r * (h + pad)
        x = pad + c * (w + pad)
        sheet[y:y + h, x:x + w] = img

    return sheet


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[args.dict])

    generated = []
    for marker_id in range(args.start_id, args.start_id + args.count):
        img = generate_marker(aruco_dict, marker_id, args.size, args.border_ratio)
        out_path = os.path.join(args.out_dir, f"marker_{marker_id}.png")
        cv2.imwrite(out_path, img)
        print(f"[INFO] Saved {out_path}")
        generated.append(img)

    if args.sheet:
        sheet = build_contact_sheet(generated)
        sheet_path = os.path.join(args.out_dir, "contact_sheet.png")
        cv2.imwrite(sheet_path, sheet)
        print(f"[INFO] Saved contact sheet: {sheet_path}")

    print(f"\n[DONE] Generated {args.count} markers "
          f"(IDs {args.start_id}-{args.start_id + args.count - 1}) "
          f"using {args.dict} in '{args.out_dir}/'")
    print("[TIP] Pass --dict / --marker_length matching these when running "
          "aruco_pose_distance.py.")


if __name__ == "__main__":
    main()
