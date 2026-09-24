import cv2
import numpy as np
import os
import sys

# HSV Green thresholds from Stage 2
LOWER_GREEN = np.array([25, 40, 30], dtype=np.uint8)
UPPER_GREEN = np.array([85, 255, 255], dtype=np.uint8)
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def get_green_mask(frame):
    """Filters green plants from the frame and cleans noise."""
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, KERNEL)
    return clean

def detect_crop_rows(mask, prev_left, prev_right, n_windows=6, window_width=150):
    """
    Finds the left and right crop rows using bottom-up sliding windows.
    Returns:
      left_fit: [slope, intercept] for x = m*y + c
      right_fit: [slope, intercept] for x = m*y + c
      left_pts: list of (x, y) detected row centers
      right_pts: list of (x, y) detected row centers
      window_boxes: list of bounding boxes [(x1, y1, x2, y2), ...]
    """
    h, w = mask.shape[:2]
    
    # We focus on the ground area in front of the rover (45% to 92% of frame height)
    roi_top = int(h * 0.45)
    roi_bottom = int(h * 0.92)
    win_h = (roi_bottom - roi_top) // n_windows
    mid_x = w // 2

    # Step 1: Find starting column bases from the bottom slice
    bottom_slice = mask[roi_bottom - win_h:roi_bottom, :]
    hist = np.sum(bottom_slice, axis=0)

    # Search for peak in left half and right half
    left_base = np.argmax(hist[:mid_x]) if np.max(hist[:mid_x]) > 0 else mid_x // 2
    right_base = (mid_x + np.argmax(hist[mid_x:])) if np.max(hist[mid_x:]) > 0 else (mid_x + mid_x // 2)

    # If previous frame fits exist, use them to guide the base position for stability
    if prev_left is not None:
        left_curr = int(np.polyval(prev_left, roi_bottom))
    else:
        left_curr = left_base

    if prev_right is not None:
        right_curr = int(np.polyval(prev_right, roi_bottom))
    else:
        right_curr = right_base

    left_pts = []
    right_pts = []
    window_boxes = []

    # Step 2: Slide windows from bottom of the frame upward
    for i in range(n_windows):
        y_low = roi_bottom - (i + 1) * win_h
        y_high = roi_bottom - i * win_h
        y_center = (y_low + y_high) // 2

        # Window boundaries
        xl_1 = max(0, left_curr - window_width // 2)
        xl_2 = min(w, left_curr + window_width // 2)
        xr_1 = max(0, right_curr - window_width // 2)
        xr_2 = min(w, right_curr + window_width // 2)

        window_boxes.append((xl_1, y_low, xl_2, y_high))
        window_boxes.append((xr_1, y_low, xr_2, y_high))

        # Check plant pixels inside left window
        patch_l = mask[y_low:y_high, xl_1:xl_2]
        if np.sum(patch_l > 0) > 30:
            M_l = cv2.moments(patch_l)
            if M_l['m00'] > 0:
                left_curr = xl_1 + int(M_l['m10'] / M_l['m00'])
                left_pts.append((left_curr, y_center))

        # Check plant pixels inside right window
        patch_r = mask[y_low:y_high, xr_1:xr_2]
        if np.sum(patch_r > 0) > 30:
            M_r = cv2.moments(patch_r)
            if M_r['m00'] > 0:
                right_curr = xr_1 + int(M_r['m10'] / M_r['m00'])
                right_pts.append((right_curr, y_center))

    # Step 3: Fit straight lines: x = poly[0]*y + poly[1]
    left_fit = None
    if len(left_pts) >= 2:
        pts = np.array(left_pts)
        fit = np.polyfit(pts[:, 1], pts[:, 0], 1)
        if prev_left is not None:
            fit = 0.7 * fit + 0.3 * prev_left  # Temporal smoothing
        left_fit = fit
    else:
        left_fit = prev_left

    right_fit = None
    if len(right_pts) >= 2:
        pts = np.array(right_pts)
        fit = np.polyfit(pts[:, 1], pts[:, 0], 1)
        if prev_right is not None:
            fit = 0.7 * fit + 0.3 * prev_right  # Temporal smoothing
        right_fit = fit
    else:
        right_fit = prev_right

    return left_fit, right_fit, left_pts, right_pts, window_boxes, (roi_top, roi_bottom)

def main():
    video_path = os.path.join("data", "crop_row_video.mp4")
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found: {video_path}")
        sys.exit(1)

    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    delay = int(1000 / fps) if fps > 0 else 33

    print("========================================")
    print("  Agricultural Rover Simulation - Stage 3")
    print("  Left and Right Crop Row Detection")
    print("========================================")
    print("Controls:")
    print("  [Space] : Pause / Resume")
    print("  [w]     : Toggle Sliding Window Boxes")
    print("  [q]     : Quit")
    print("========================================")

    paused = False
    show_boxes = True
    prev_left = None
    prev_right = None

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            # 1. Detect green plants (Stage 2)
            mask = get_green_mask(frame)

            # 2. Track left and right crop rows (Stage 3)
            left_fit, right_fit, left_pts, right_pts, boxes, (roi_top, roi_bottom) = detect_crop_rows(
                mask, prev_left, prev_right
            )
            prev_left = left_fit
            prev_right = right_fit

            # 3. Create visualization overlay
            vis = frame.copy()

            # Draw ROI boundary lines (cyan)
            w = frame.shape[1]
            cv2.line(vis, (0, roi_top), (w, roi_top), (255, 255, 0), 1)
            cv2.line(vis, (0, roi_bottom), (w, roi_bottom), (255, 255, 0), 1)
            cv2.putText(vis, "Analysis Region (ROI)", (15, roi_top - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Optionally draw sliding search windows (yellow)
            if show_boxes:
                for (x1, y1, x2, y2) in boxes:
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 1)

            # Draw detected row center points
            for pt in left_pts:
                cv2.circle(vis, pt, 5, (0, 0, 255), -1)   # Red dots for left row
            for pt in right_pts:
                cv2.circle(vis, pt, 5, (255, 0, 0), -1)   # Blue dots for right row

            # Draw fitted left crop line (Red)
            y_eval = np.array([roi_bottom, roi_top])
            if left_fit is not None:
                xl = np.polyval(left_fit, y_eval).astype(int)
                cv2.line(vis, (xl[0], y_eval[0]), (xl[1], y_eval[1]), (0, 0, 255), 3)

            # Draw fitted right crop line (Blue)
            if right_fit is not None:
                xr = np.polyval(right_fit, y_eval).astype(int)
                cv2.line(vis, (xr[0], y_eval[0]), (xr[1], y_eval[1]), (255, 0, 0), 3)

            # Draw HUD Telemetry Header
            cv2.rectangle(vis, (0, 0), (w, 40), (20, 20, 20), -1)
            status_text = f"LEFT ROW: {'TRACKED' if left_fit is not None else 'LOST'} | RIGHT ROW: {'TRACKED' if right_fit is not None else 'LOST'} | [W] Boxes: {'ON' if show_boxes else 'OFF'}"
            cv2.putText(vis, status_text, (15, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Rover Vision - Stage 3: Crop Row Detection", vis)

        key = cv2.waitKey(delay if not paused else 50) & 0xFF
        if key == ord('q') or key == 27:
            print("\n[INFO] Exiting Stage 3...")
            break
        elif key == ord(' '):
            paused = not paused
            print("[INFO] Paused" if paused else "[INFO] Resumed")
        elif key == ord('w'):
            show_boxes = not show_boxes
            print(f"[INFO] Sliding window boxes: {'ON' if show_boxes else 'OFF'}")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Cleaned up and closed window.")

if __name__ == "__main__":
    main()
