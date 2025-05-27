import torch
import requests
import os

# Load YOLOv5 model
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', source='github')

# Image paths for both lanes
img_paths = {
    "lane1": "testing_images/lane1(4).jpg",  # Update as needed
    "lane2": "testing_images/lane1(3).jpg"   # Update as neededb
}

car_counts = {}

# Process both images
for lane, img_path in img_paths.items():
    if not os.path.exists(img_path):
        raise FileNotFoundError(f"Image not found: {img_path}")
    
    print(f"\nRunning inference on {lane} - {img_path}...")
    results = model(img_path)
    results.show()

    # Extract detected objects
    df = results.pandas().xyxy[0]
    car_count = (df['name'] == 'car').sum()
    car_counts[lane] = car_count
    print(f"Number of cars detected in {lane}: {car_count}")

# Determine which lane has more vehicles
more_dense_lane = max(car_counts, key=car_counts.get)
print(f"\nLane with more vehicles: {more_dense_lane}")

# Decide lane number for ESP32 (1 or 2)
start_lane = "1" if more_dense_lane == "lane1" else "2"

# Extract car counts
lane1_count = car_counts["lane1"]
lane2_count = car_counts["lane2"]

# Send command to ESP32 to start cycle with vehicle counts
esp32_ip = "192.168.15.173"  # Update to your ESP32 IP
esp32_url = (
    f"http://{esp32_ip}/start_cycle?"
    f"lane={start_lane}&lane1_vehicles={lane1_count}&lane2_vehicles={lane2_count}"
)

print(f"\nStarting traffic light cycle...")
print(f"Request URL: {esp32_url}")

try:
    response = requests.get(esp32_url)
    print(f"ESP32 responded with: {response.text}")
except Exception as e:
    print(f"Failed to send command to ESP32: {e}")
