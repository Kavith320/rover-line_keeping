import cv2
import sys
import os

def main():
    # 1. Path to our recorded field video
    video_path = os.path.join("data", "crop_row_video.mp4")

    # Check if the video file actually exists
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found at: {video_path}")
        print("Please ensure the video is located in the 'data' folder.")
        sys.exit(1)

    # 2. Open the video using OpenCV's VideoCapture
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"[ERROR] Could not open video file: {video_path}")
        sys.exit(1)

    # 3. Read video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print("========================================")
    print("  Agricultural Rover Simulation - Stage 1")
    print("========================================")
    print(f"Video Loaded : {video_path}")
    print(f"Resolution   : {width} x {height}")
    print(f"Frame Rate   : {fps:.1f} FPS")
    print(f"Total Frames : {total_frames}")
    print("----------------------------------------")
    print("Controls:")
    print("  [Space] : Pause / Resume")
    print("  [q]     : Quit")
    print("========================================")

    # Delay between frames in milliseconds to maintain natural playback speed
    delay = int(1000 / fps) if fps > 0 else 33

    paused = False

    # 4. Main loop: process and display frame by frame
    while True:
        if not paused:
            # Read the next frame from the video
            ret, frame = cap.read()

            # If ret is False, we reached the end of the video
            if not ret:
                print("\n[INFO] End of video reached. Looping back to the beginning...")
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

        # Display the frame in an OpenCV window
        cv2.imshow("Agricultural Rover - Front Camera Stream", frame)

        # Wait for user key press
        key = cv2.waitKey(delay if not paused else 50) & 0xFF

        # If 'q' is pressed, exit the loop
        if key == ord('q') or key == 27:  # 27 is ESC key
            print("\n[INFO] 'q' pressed. Exiting...")
            break
        # If Spacebar is pressed, toggle pause
        elif key == ord(' '):
            paused = not paused
            print("[INFO] Paused" if paused else "[INFO] Resumed")

    # 5. Clean up resources
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Cleanup complete. Window closed.")

if __name__ == "__main__":
    main()
