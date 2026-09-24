import cv2
import numpy as np
import math
import os

def generate_video(output_path="data/simulated_field.mp4", width=856, height=480, fps=30, duration_sec=24):
    """
    Generates a clean, synthetic 3D agricultural field video for rover simulation.
    Includes:
      - Horizon, sky gradient, and distant treeline
      - Textured agricultural soil ground with furrow tracks
      - Clear left and right crop rows with leafy green plants
      - Controlled rover drift scenarios (Straight -> Drift Right -> Straight -> Drift Left -> Straight)
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    total_frames = fps * duration_sec
    horizon_y = int(height * 0.38)
    base_plant_spacing = 0.30  # 30cm between plants (dense crop row)
    row_dist_half = 0.70       # 1.4m spacing between rows
    cam_height = 0.65          # Camera height 65cm above ground
    focal_len = 420.0          # Perspective focal length

    print(f"[INFO] Generating {duration_sec}s simulation video ({total_frames} frames)...")

    for f in range(total_frames):
        t = f / fps

        # Trajectory schedule:
        # 0 - 4s: Centered (offset = 0m) -> Rover goes STRAIGHT
        # 4 - 8s: Drift Right (+0.28m)   -> Rover must steer LEFT
        # 8 - 12s: Centered (0m)         -> Rover goes STRAIGHT
        # 12 - 16s: Drift Left (-0.28m)  -> Rover must steer RIGHT
        # 16 - 20s: Gentle curve drift   -> Rover steers RIGHT
        # 20 - 24s: Straight finish
        if t < 4.0:
            rover_x = 0.0
            rover_heading = 0.0
        elif t < 8.0:
            phase = (t - 4.0) / 4.0
            rover_x = 0.28 * math.sin(phase * math.pi)
            rover_heading = 0.08 * math.cos(phase * math.pi)
        elif t < 12.0:
            rover_x = 0.0
            rover_heading = 0.0
        elif t < 16.0:
            phase = (t - 12.0) / 4.0
            rover_x = -0.28 * math.sin(phase * math.pi)
            rover_heading = -0.08 * math.cos(phase * math.pi)
        elif t < 20.0:
            phase = (t - 16.0) / 4.0
            rover_x = 0.22 * math.sin(phase * math.pi)
            rover_heading = 0.06 * math.cos(phase * math.pi)
        else:
            rover_x = 0.0
            rover_heading = 0.0

        forward_dist = t * 1.2  # 1.2 m/s rover speed
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # 1. Sky Gradient
        for y in range(horizon_y):
            ratio = y / horizon_y
            frame[y, :] = (int(225 - 35 * ratio), int(205 - 25 * ratio), int(185 - 15 * ratio))

        # Distant treeline at horizon
        for x in range(width):
            tree_h = int(12 + 6 * math.sin(x * 0.03) + 3 * math.cos(x * 0.08))
            cv2.line(frame, (x, horizon_y), (x, horizon_y - tree_h), (50, 95, 60), 1)

        # 2. Rich Soil Ground
        for y in range(horizon_y, height):
            ratio = (y - horizon_y) / (height - horizon_y)
            frame[y, :] = (int(32 + 28 * (ratio ** 0.8)), int(48 + 38 * (ratio ** 0.8)), int(72 + 52 * (ratio ** 0.8)))

        # Soil texture noise
        np.random.seed(f % 100)
        noise = np.random.randint(-8, 8, (height - horizon_y, width, 3), dtype=np.int16)
        frame[horizon_y:, :] = np.clip(frame[horizon_y:, :].astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Furrow wheel tracks down the center corridor
        vanish_x = int(width / 2 - (rover_x / 1.0) * 80 - rover_heading * 200)
        for track_offset in [-0.28, 0.28]:
            bottom_x = int(width / 2 + ((track_offset - rover_x) / 0.5) * 160)
            cv2.line(frame, (vanish_x, horizon_y), (bottom_x, height), (35, 50, 70), 2)

        # 3. Dense leafy green crop plants along Left and Right rows
        start_plant_idx = int(forward_dist / base_plant_spacing)
        max_row_plants = int(18.5 * 1.2 / base_plant_spacing)  # Rows end at ~18.5s (Field Boundary / Headland)
        plants = []

        for p_idx in range(start_plant_idx, start_plant_idx + 45):
            if p_idx >= max_row_plants:
                continue  # Field edge reached: open headland ahead
            w_z = p_idx * base_plant_spacing - forward_dist
            if w_z < 0.6 or w_z > 14.0:
                continue
            np.random.seed(p_idx * 31)
            jitter_l = (np.random.rand() - 0.5) * 0.05
            jitter_r = (np.random.rand() - 0.5) * 0.05
            plants.append((-row_dist_half + jitter_l, w_z, p_idx, 'left'))
            plants.append((row_dist_half + jitter_r, w_z, p_idx, 'right'))

        # Draw far to near (painter's algorithm)
        plants.sort(key=lambda p: p[1], reverse=True)

        for w_x, w_z, p_idx, side in plants:
            cam_x = w_x - rover_x
            cam_y = cam_height

            sx = int(width / 2 + (cam_x / w_z) * focal_len - rover_heading * 150)
            sy = int(horizon_y + (cam_y / w_z) * focal_len)

            p_size = int(70 / w_z)
            if p_size < 2 or sx < -60 or sx > width + 60:
                continue

            # Plant Stalk
            cv2.line(frame, (sx, sy), (sx, sy - int(p_size * 0.8)), (25, 110, 30), max(1, p_size // 7))

            # Multi-leaf cluster
            np.random.seed(p_idx * 13)
            colors = [(35, 165, 45), (28, 145, 38), (45, 185, 55), (20, 130, 30)]
            for ang in [-55, -30, -10, 15, 40, 60]:
                leaf_len = max(3, int(p_size * (0.8 + 0.3 * np.random.rand())))
                leaf_thick = max(1, leaf_len // 3)
                rad = math.radians(ang + np.random.randint(-8, 8))
                lx = sx + int(math.sin(rad) * leaf_len * 0.5)
                ly = sy - int(p_size * 0.5) - int(math.cos(rad) * leaf_len * 0.3)
                col = colors[abs(ang) % len(colors)]
                cv2.ellipse(frame, (lx, ly), (leaf_len, leaf_thick), ang, 0, 360, col, -1)

        out.write(frame)

    out.release()
    print(f"[SUCCESS] Simulation video saved to: {output_path}")

if __name__ == "__main__":
    generate_video()
