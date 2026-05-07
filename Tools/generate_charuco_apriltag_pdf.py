"""
Generate a ChArUco board PDF with AprilTag 36h11 markers for multi-camera
extrinsic calibration (issue #9 / Phase E).

Why AprilTag instead of ArUco: AprilTag 36h11 has stronger error correction and
fewer false positives at oblique angles than DICT_6X6_*. OpenCV's CharucoBoard
accepts any predefined dictionary, so the existing detection pipeline
(`Aruco.interpolateCornersCharuco` + `Aruco.estimatePoseCharucoBoard`) works
unchanged — we just point CharucoBoardSpec.dictionary at DictAprilTag_36h11.

Output: a PDF sized DIN-A3 landscape (420 x 297 mm) with the ChArUco board
centered on the page. Print at 100% scale (no fit-to-page!) so the printed
board matches the metric values used in calibration.

Usage:
    python generate_charuco_apriltag_pdf.py
    python generate_charuco_apriltag_pdf.py --squaresX 7 --squaresY 5 \\
        --squareMm 55 --markerMm 38 --out board.pdf
"""

import argparse
import os
import sys
import tempfile

import cv2
import fitz   # PyMuPDF
import numpy as np


def build_board(squaresX: int, squaresY: int, squareMm: float, markerMm: float,
                dpi: int = 300):
    """Render the board to a numpy uint8 grayscale image at the requested DPI."""
    if markerMm >= squareMm:
        raise ValueError("markerMm must be strictly less than squareMm")

    px_per_mm = dpi / 25.4
    img_w = int(round(squaresX * squareMm * px_per_mm))
    img_h = int(round(squaresY * squareMm * px_per_mm))

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    # cv2 >= 4.7 uses the constructor + generateImage API.
    board = cv2.aruco.CharucoBoard(
        (squaresX, squaresY),
        squareLength=squareMm * 0.001,   # OpenCV stores meters but we only care about ratio for drawing
        markerLength=markerMm * 0.001,
        dictionary=dictionary,
    )
    img = board.generateImage((img_w, img_h), marginSize=0, borderBits=1)
    return img, img_w, img_h


def write_pdf(img: np.ndarray, board_mm: tuple, page_mm: tuple, out_path: str):
    """Place the board image centered on a DIN-A3 landscape PDF page (mm-accurate)."""
    # 1 mm = 72/25.4 pt ≈ 2.83465 pt
    pt_per_mm = 72.0 / 25.4
    page_w_pt = page_mm[0] * pt_per_mm
    page_h_pt = page_mm[1] * pt_per_mm
    board_w_pt = board_mm[0] * pt_per_mm
    board_h_pt = board_mm[1] * pt_per_mm

    if board_w_pt > page_w_pt + 1e-6 or board_h_pt > page_h_pt + 1e-6:
        raise ValueError(
            f"Board ({board_mm[0]}x{board_mm[1]}mm) does not fit on page "
            f"({page_mm[0]}x{page_mm[1]}mm). Reduce squaresX/Y or squareMm.")

    # Save the image to a temp PNG that fitz can ingest as an Image.
    with tempfile.TemporaryDirectory() as td:
        tmp_png = os.path.join(td, "board.png")
        cv2.imwrite(tmp_png, img)

        doc = fitz.open()
        page = doc.new_page(width=page_w_pt, height=page_h_pt)
        x0 = (page_w_pt - board_w_pt) * 0.5
        y0 = (page_h_pt - board_h_pt) * 0.5
        rect = fitz.Rect(x0, y0, x0 + board_w_pt, y0 + board_h_pt)
        page.insert_image(rect, filename=tmp_png)

        # Add a footer with the spec so what's printed matches what you put in
        # CharucoBoardSpec. Use a small invariant font on the bottom margin.
        footer_y = page_h_pt - 18  # 18 pt above bottom edge
        text = (f"ChArUco + AprilTag36h11  |  "
                f"squares={img.shape[1] // (img.shape[0] // (page_mm[1] // 1)) if False else ''}"
                "")  # placeholder removed; build clearer footer below
        doc.save(out_path)
        doc.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--squaresX", type=int, default=7,
                    help="Number of chessboard squares along X (default: 7).")
    ap.add_argument("--squaresY", type=int, default=5,
                    help="Number of chessboard squares along Y (default: 5). "
                         "Use a non-square count to avoid 180-degree pose ambiguity.")
    ap.add_argument("--squareMm", type=float, default=55.0,
                    help="Chessboard square length in millimeters (default: 55).")
    ap.add_argument("--markerMm", type=float, default=38.0,
                    help="AprilTag marker side length in millimeters (default: 38, ~0.69x square).")
    ap.add_argument("--pageMm", default="420x297",
                    help="Page size in mm, format WxH (default: 420x297, DIN-A3 landscape).")
    ap.add_argument("--dpi", type=int, default=300,
                    help="Render DPI for the embedded image (default: 300).")
    ap.add_argument("--out", default="charuco_apriltag36h11_A3.pdf",
                    help="Output PDF path.")
    args = ap.parse_args()

    page_w_mm, page_h_mm = (float(v) for v in args.pageMm.lower().split("x"))
    board_w_mm = args.squaresX * args.squareMm
    board_h_mm = args.squaresY * args.squareMm

    print(f"Page          : {page_w_mm:.0f} x {page_h_mm:.0f} mm")
    print(f"Board         : {board_w_mm:.0f} x {board_h_mm:.0f} mm "
          f"({args.squaresX} x {args.squaresY} squares @ {args.squareMm:.1f} mm)")
    print(f"Marker        : {args.markerMm:.1f} mm "
          f"(ratio {args.markerMm / args.squareMm:.2f})")
    print(f"Dictionary    : DICT_APRILTAG_36h11")
    print(f"Output        : {args.out}")
    print()

    img, w, h = build_board(args.squaresX, args.squaresY, args.squareMm, args.markerMm, dpi=args.dpi)
    print(f"Rendered      : {w} x {h} px @ {args.dpi} DPI")

    write_pdf(img, (board_w_mm, board_h_mm), (page_w_mm, page_h_mm), args.out)
    print(f"Wrote         : {os.path.abspath(args.out)}")
    print()
    print("CharucoBoardSpec inspector values:")
    print(f"  squaresX            = {args.squaresX}")
    print(f"  squaresY            = {args.squaresY}")
    print(f"  squareLengthMeters  = {args.squareMm * 0.001:.4f}")
    print(f"  markerLengthMeters  = {args.markerMm * 0.001:.4f}")
    print(f"  dictionary          = DictAprilTag_36h11")
    print()
    print("Print at 100% scale (no fit-to-page) so the printed dimensions match.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
