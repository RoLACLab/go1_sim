from openai import OpenAI
import base64
import json
import re
import glob
import os

def encode_image_to_base64(image_path):
    """Encode image file to base64 string."""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

def extract_first_action(response_text):
    """Extract first command and time from OpenAI response."""
    # Clean markdown code blocks
    cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", response_text.strip(), flags=re.MULTILINE)
    
    try:
        # Parse JSON
        data = json.loads(cleaned)
        if "objects" in data and data["objects"]:
            first_action = data["objects"][0]
            return first_action.get("Command", ""), first_action.get("Time", "")
    except json.JSONDecodeError:
        # Fallback: regex extraction
        command_match = re.search(r'"Command":\s*"([^"]*)"', cleaned)
        time_match = re.search(r'"Time":\s*"([^"]*)"', cleaned)
        if command_match and time_match:
            return command_match.group(1), time_match.group(1)
    
    return None, None

def save_action_plan(command, time):
    """Save command and time to action_plan.txt."""
    with open("action_plan.txt", "w") as f:
        f.write(f"{command} {time}")

def main():
    # Configuration
    client = OpenAI(api_key="YOUR_API_KEY_HERE")
    screenshot_dir = "/home/shivayogiakki/Documents/github/go1_sim/LLM server request/screenshots"
    list_of_files = glob.glob(os.path.join(screenshot_dir, "rviz_screenshot_*.png"))
    if not list_of_files:
        print("No screenshot files found.")
        return
    image_path = max(list_of_files, key=os.path.getctime)
    
    system_prompt = (
        "You are a robot navigation assistant that analyzes LiDAR data from RViz images. Your task is to help a robot navigate through rooms and doorways safely. You should analyze the LiDAR scan data to identify obstacles, walls, and openings. Always prioritize safe navigation and avoid collisions. The available control options are front (w), back (s), move left and right with a and d. It can also turn left and right with J and L. Z is to stop. Things to note: 1. there is a small angle theta behind the robot where the lidar is not scanned. Ignore this region (door does not exist here). 2. this robot needs to walk through the door. Procedure: (I) I will provide you an image, (II) You need to provide me 2 instructions, command (what needs to be done), and time (how many seconds you want me to be moving in that direction, max being 10 secs), and (III) then you will receive an updated rviz image again. Return the 1 response as a JSON object with the following format: "
        '{ "objects": [ {"Short Reason": string, "Command": string, "Time": string}] }.'
        "Also, make sure to avoid hitting walls (indicated by green dots making a line). Move back when required. Door definition: Gaps between walls (indicated by green dots) that allow the robot to exit the room."
    )
    
    user_prompt = (
        "Now provided here is the new lidar map. What should the robot do in this new updated scenario? "
    )
    
    # Encode image and get response
    base64_image = encode_image_to_base64(image_path)
    
    response = client.chat.completions.create(
        model="chatgpt-4o-latest",
        messages=[
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{base64_image}"}}
                ]
            }
        ],
        temperature=0.5
    )
    
    # Process response
    output_text = response.choices[0].message.content
    print("Raw output:", output_text)
    
    command, time = extract_first_action(output_text)
    
    if command and time:
        print(f"First action - Command: {command}, Time: {time}")
        save_action_plan(command, time)
        print("Action saved to action_plan.txt")
    else:
        print("Could not extract command and time from response")

if __name__ == "__main__":
    main()