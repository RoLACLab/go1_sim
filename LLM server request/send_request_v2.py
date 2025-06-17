import requests
import json
import os
import time
import signal
import sys

SERVER_URL = "http://141.219.171.151:5000/get_action"
SCAN_FILE_PATH = os.path.join(os.path.expanduser("~"), "scan_data.json")
OUTPUT_FILE_PATH = os.path.join(os.path.expanduser("~"), "action_plan.json")

# Action history to accumulate responses
action_history = []

running = True

def signal_handler(sig, frame):
    global running
    print("\n[INFO] Caught interrupt signal. Exiting loop.")
    running = False

# Register Ctrl+C signal handler
signal.signal(signal.SIGINT, signal_handler)

def load_latest_scan():
    try:
        with open(SCAN_FILE_PATH, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[ERROR] Loading scan data: {str(e)}")
        return None

def main_loop(poll_interval=2.0):
    print(f"[INFO] Monitoring {SCAN_FILE_PATH} for new scans. Press Ctrl+C to exit.")
    last_mtime = 0

    while running:
        try:
            current_mtime = os.path.getmtime(SCAN_FILE_PATH)
        except FileNotFoundError:
            print("[WARN] File not found. Waiting...")
            time.sleep(poll_interval)
            continue

        if current_mtime != last_mtime:
            lidar_data = load_latest_scan()
            if lidar_data:
                payload = {
                    "lidar_data": lidar_data,
                    "action_history": action_history
                }

                print("\n--- DATA TO BE SENT ---")
                print(json.dumps(payload, indent=2))

                try:
                    print(f"\n[INFO] Sending request to server at {SERVER_URL}...")
                    response = requests.post(SERVER_URL, json=payload, timeout=30)
                    response.raise_for_status()
                    action_plan = response.json()

                    print("\n--- ACTION PLAN RECEIVED ---")
                    print(json.dumps(action_plan, indent=2))

                    with open(OUTPUT_FILE_PATH, 'w') as f:
                        json.dump(action_plan, f, indent=2)

                    print(f"[✓] Action plan saved to: {OUTPUT_FILE_PATH}")
                    action_history.append(action_plan)

                except requests.exceptions.RequestException as e:
                    print(f"[ERROR] Server communication failed:\n{e}")
            else:
                print("[INFO] Invalid or empty scan data.")

            last_mtime = current_mtime
        else:
            print("[INFO] No new scan data.")

        time.sleep(poll_interval)

    print("[INFO] Loop stopped. Program exiting.")

if __name__ == "__main__":
    main_loop(poll_interval=2.0)
