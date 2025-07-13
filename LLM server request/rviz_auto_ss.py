import subprocess
import time
import os
from datetime import datetime
from PIL import Image

interval_seconds = 3  # Delay between screenshots
window_name = "move_base.rviz* - RViz"
crop_margin = 80  # Number of pixels to crop from each edge

# Directory to save cropped screenshots
save_dir = "/home/shivayogiakki/Documents/github/go1_sim/LLM server request/screenshots"
os.makedirs(save_dir, exist_ok=True)

print(f"Saving RViz screenshots to '{save_dir}' every {interval_seconds} seconds. Press Ctrl+C to stop.")
time.sleep(2)

try:
    while True:
        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"rviz_screenshot_{timestamp}.png"
        filepath = os.path.join(save_dir, filename)

        # Take the screenshot
        subprocess.run(["import", "-window", window_name, filepath])

        # Crop 'crop_margin' pixels from all sides
        try:
            with Image.open(filepath) as img:
                width, height = img.size
                if width > 2 * crop_margin and height > 2 * crop_margin:
                    cropped = img.crop((crop_margin, crop_margin, width - crop_margin, height - crop_margin))
                    cropped.save(filepath)
                else:
                    print(f"Image too small to crop {crop_margin} pixels from all sides.")
        except Exception as e:
            print(f"Failed to crop image: {e}")

        print(f"Saved and cropped: {filepath}")
        time.sleep(interval_seconds)
except KeyboardInterrupt:
    print("Stopped by user.")
