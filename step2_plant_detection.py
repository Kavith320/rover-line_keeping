import cv2
import numpy as np
import os
import sys

def main():
    # 1. Load the video stream from Stage 1
    video_path = os.path.join("data", "crop_row_video.mp4")
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found at: {video_path}")
        sys.exit(1)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[ERROR] Could not open video file: {video_path}")
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS)
    delay = int(1000 / fps) if fps > 0 else 33

    print("========================================")
    print("  Agricultural Rover Simulation - Stage 2")
    print("  Green Plant Detection (HSV Masking)")
    print("========================================")
    print("Controls:")
    print("  [Space] : Pause / Resume")
    print("  [m]     : Toggle View Mode (Mask vs Isolated Plants)")
    print("  [l]     : Toggle Adaptive Lighting (CLAHE + ExG vs Static HSV)")
    print("  [q]     : Quit")
    print("========================================")

    # 2. Base HSV color boundaries for static mode
    lower_green = np.array([25, 40, 30], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)

    # Morphological kernel (small 5x5 ellipse) to clean up noise specks
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    paused = False
    use_adaptive = True
    view_mode = "side_by_side_mask"  # modes: 'side_by_side_mask' or 'side_by_side_color'

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                # Loop video when reaching the end
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            amb_brightness = float(np.mean(gray))

            if use_adaptive:
                # Step A1: CLAHE on L channel in LAB color space
                lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
                l, a, b = cv2.split(lab)
                lab_clahe = cv2.merge([clahe.apply(l), a, b])
                proc_frame = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)
                blurred = cv2.GaussianBlur(proc_frame, (5, 5), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # Dynamic S_min and V_min
                v_min = int(np.clip(30 - (128 - amb_brightness) * 0.15, 15, 45))
                s_min = int(np.clip(40 + (amb_brightness - 128) * 0.10, 25, 60))
                hsv_mask = cv2.inRange(hsv, np.array([25, s_min, v_min], dtype=np.uint8), upper_green)

                # Excess Green Index (ExG): 2G - R - B
                b_c = blurred[:, :, 0].astype(np.float32)
                g_c = blurred[:, :, 1].astype(np.float32)
                r_c = blurred[:, :, 2].astype(np.float32)
                exg = np.clip(2.0 * g_c - r_c - b_c, 0, 255).astype(np.uint8)
                _, exg_mask = cv2.threshold(exg, 18, 255, cv2.THRESH_BINARY)

                # Fuse with Hue safety guard
                fused = cv2.bitwise_or(hsv_mask, exg_mask)
                valid_hue = (hsv[:, :, 0] >= 22) & (hsv[:, :, 0] <= 90)
                fused[~valid_hue] = 0
            else:
                blurred = cv2.GaussianBlur(frame, (5, 5), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                fused = cv2.inRange(hsv, lower_green, upper_green)

            # Step D: Morphological Filtering
            clean_mask = cv2.morphologyEx(fused, cv2.MORPH_OPEN, kernel)
            clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, kernel)

            # Step E: Extract actual plant colors
            plant_color = cv2.bitwise_and(frame, frame, mask=clean_mask)

            # Step F: Prepare side-by-side visualization
            disp_w, disp_h = 540, 300
            left_view = cv2.resize(frame, (disp_w, disp_h))

            mode_tag = "ADAPTIVE (CLAHE+ExG)" if use_adaptive else "STATIC HSV"
            if view_mode == "side_by_side_mask":
                right_view = cv2.cvtColor(clean_mask, cv2.COLOR_GRAY2BGR)
                right_view = cv2.resize(right_view, (disp_w, disp_h))
                right_label = f"Mask [{mode_tag}]"
            else:
                right_view = cv2.resize(plant_color, (disp_w, disp_h))
                right_label = f"Isolated [{mode_tag}]"

            # Draw explanatory text headers
            cv2.putText(left_view, f"Camera Feed (Amb: {amb_brightness:.0f})", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(right_view, right_label, (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 255, 100) if use_adaptive else (100, 200, 255), 2)
            cv2.putText(right_view, "[L]: Toggle Light Mode | [M]: View Mode", (15, disp_h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

            # Combine Left and Right side by side
            combined_display = np.hstack([left_view, right_view])

        cv2.imshow("Rover Vision - Stage 2: Plant Detection", combined_display)

        key = cv2.waitKey(delay if not paused else 50) & 0xFF

        if key == ord('q') or key == 27:
            print("\n[INFO] Exiting Stage 2...")
            break
        elif key == ord(' '):
            paused = not paused
            print("[INFO] Paused" if paused else "[INFO] Resumed")
        elif key == ord('l') or key == ord('L'):
            use_adaptive = not use_adaptive
            print(f"[INFO] Light Mode: {'ADAPTIVE (CLAHE + ExG + Dynamic HSV)' if use_adaptive else 'STATIC HSV'}")
        elif key == ord('m') or key == ord('M'):
            if view_mode == "side_by_side_mask":
                view_mode = "side_by_side_color"
                print("[INFO] View mode: Isolated Plant Colors")
            else:
                view_mode = "side_by_side_mask"
                print("[INFO] View mode: Binary Plant Mask")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Cleaned up and closed window.")

if __name__ == "__main__":
    main()
