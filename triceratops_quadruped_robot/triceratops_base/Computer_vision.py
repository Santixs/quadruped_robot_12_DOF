import cv2
import numpy as np
import time
import mediapipe as mp
import subprocess
import re
import argparse
import os
import socket
import threading
import requests

# Add Flask for web streaming
from flask import Flask, Response, render_template
import logging

# Add ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

# Set environment variable for headless operation
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

# Parse command line arguments
parser = argparse.ArgumentParser(description='Computer Vision for Triceratops Robot')
parser.add_argument('--no-gui', action='store_true', help='Disable GUI windows (for running on robot without display)')
parser.add_argument('--web-port', type=int, default=8080, help='Port for web server (default: 8080)')
args, unknown = parser.parse_known_args()

# Import Unity ROS2 messages
import sys

sys.path.append(os.path.expanduser("~/UnityRos2_ws/src/unity_robotics_demo_msgs"))

# --- Web Server Configuration ---
app = Flask(__name__)
# Disable Flask's default logging to avoid flooding console
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Global variable for the latest frame
global_frame = None
frame_lock = threading.Lock()

# --- Create templates directory and HTML file ---
os.makedirs('templates', exist_ok=True)
with open('templates/index.html', 'w') as f:
    f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>P.A.N.D.A Vision Console</title>
    <style>
        body {
            font-family: 'Arial', sans-serif;
            background-color: #1a1a1a;
            color: #ffffff;
            margin: 0;
            padding: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
            height: 100vh;
        }
        .header {
            background-color: #333333;
            width: 100%;
            padding: 15px 0;
            text-align: center;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        h1 {
            margin: 0;
            color: #00cc66;
            font-weight: 300;
            letter-spacing: 2px;
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            width: 90%;
            max-width: 1000px;
        }
        .video-container {
            width: 100%;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
        }
        img {
            width: 100%;
            height: auto;
            display: block;
        }
        .status {
            margin-top: 20px;
            background-color: #333333;
            padding: 10px 20px;
            border-radius: 5px;
            font-size: 14px;
        }
        .highlight {
            color: #00cc66;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>P.A.N.D.A Vision Console</h1>
    </div>
    <div class="container">
        <div class="video-container">
            <img src="{{ url_for('video_feed') }}" alt="Video Feed">
        </div>
        <div class="status">
            Status: <span class="highlight">Connected</span>
        </div>
    </div>
</body>
</html>
''')

def get_ip_address():
    """Get the primary IP address of the machine"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

@app.route('/')
def index():
    """Serve the main web page"""
    return render_template('index.html')

def generate_frames():
    """Generate frames for the web video stream"""
    while True:
        with frame_lock:
            if global_frame is not None:
                frame = global_frame.copy()
            else:
                # If no frame is available, create a blank frame with a message
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for camera...", (50, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Encode frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
            
        # Yield the frame in the HTTP response format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        # Small delay to control frame rate
        time.sleep(0.05)  # ~20 fps

@app.route('/video_feed')
def video_feed():
    """Route for the video feed"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def start_web_server(port):
    """Start the Flask web server in a separate thread"""
    ip = get_ip_address()
    print(f"\n╔════════════════════════════════════════════╗")
    print(f"║ Web server started!                        ║")
    print(f"║ Access P.A.N.D.A Vision Console at:        ║")
    print(f"║ http://{ip}:{port}                  ║")
    print(f"╚════════════════════════════════════════════╝\n")
    
    # Start the Flask app
    app.run(host='0.0.0.0', port=port, threaded=True)

# --- WiFi Camera Configuration ---
DEFAULT_STREAM_URL = "http://192.168.41.116:81/stream"

def set_camera_resolution(ip, resolution="vga"):
    """Set the camera resolution using the control endpoint"""
    try:
        # Map resolution names to ESP32-CAM values
        resolution_map = {
            "qvga": 5,  # 320x240
            "vga": 8,   # 640x480
            "svga": 9,  # 800x600
            "xga": 10,  # 1024x768
            "sxga": 11, # 1280x1024
            "uxga": 12  # 1600x1200
        }
        
        if resolution not in resolution_map:
            print(f"Warning: Unknown resolution {resolution}, using VGA")
            resolution = "vga"
            
        # Set the resolution
        url = f"http://{ip}/control?var=framesize&val={resolution_map[resolution]}"
        response = requests.get(url, timeout=2.0)  # Add timeout
        
        if response.status_code == 200:
            print(f"Successfully set camera resolution to {resolution.upper()}")
            # Wait a moment for the camera to adjust
            time.sleep(1)
            return True
        else:
            print(f"Failed to set camera resolution: {response.status_code}")
            return False
            
    except requests.exceptions.RequestException as e:
        print(f"Error setting camera resolution: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error setting camera resolution: {e}")
        return False

def verify_camera_resolution(cap, target_width=640, target_height=480, max_attempts=3):
    """Verify that the camera is using the correct resolution"""
    for attempt in range(max_attempts):
        ret, frame = cap.read()
        if ret:
            height, width = frame.shape[:2]
            if width == target_width and height == target_height:
                print(f"Verified camera resolution: {width}x{height}")
                return True
            else:
                print(f"Camera resolution mismatch: got {width}x{height}, expected {target_width}x{target_height}")
                if attempt < max_attempts - 1:
                    print("Retrying...")
                    time.sleep(1)
        else:
            print("Failed to read frame for resolution verification")
            if attempt < max_attempts - 1:
                print("Retrying...")
                time.sleep(1)
    return False

def get_camera_ip():
    """
    Extract camera IP address from the camera_output.txt file
    Returns the camera stream URL if found, otherwise returns the default URL
    """
    try:
        # Try to read from the camera_output.txt file
        with open('camera_output.txt', 'r') as f:
            content = f.read()
        
        # Search for the IP address in the output
        # Pattern looks for: http://192.168.xx.xx
        match = re.search(r'http://(\d+\.\d+\.\d+\.\d+)', content)
        
        if match:
            ip_address = match.group(1)
            print(f"Camera IP detected: {ip_address}")
            return ip_address
        else:
            print("No camera IP found in output. Using default URL.")
            return DEFAULT_STREAM_URL.split('/')[2].split(':')[0]
            
    except FileNotFoundError:
        print("Camera output file not found. Using default camera URL")
        return DEFAULT_STREAM_URL.split('/')[2].split(':')[0]
    except Exception as e:
        # Only show error for unexpected exceptions
        print(f"Error getting camera IP: {e}")
        return DEFAULT_STREAM_URL.split('/')[2].split(':')[0]

# Get the camera IP and set resolution
CAMERA_IP = get_camera_ip()
set_camera_resolution(CAMERA_IP, "vga")  # Set to VGA resolution
STREAM_URL = f"http://{CAMERA_IP}:81/stream"

# --- Box Detection Configuration ---
# Brown color ranges for different lighting conditions
LOWER_BROWN1 = np.array([0, 20, 40])
UPPER_BROWN1 = np.array([30, 255, 200])
LOWER_BROWN2 = np.array([5, 20, 100])
UPPER_BROWN2 = np.array([25, 180, 255])
# New range for grayish-brown (60,60,56)
LOWER_BROWN3 = np.array([20, 5, 15])  # Lower bound for grayish-brown
UPPER_BROWN3 = np.array([40, 80, 80])  # Upper bound for grayish-brown
# New range for grayish color (67,69,67)
LOWER_BROWN4 = np.array([30, 2, 20])  # Lower bound for grayish
UPPER_BROWN4 = np.array([90, 30, 100])  # Upper bound for grayish

# Black detection parameters
BLACK_VALUE_THRESH = 40
MIN_LOGO_RATIO = 0.02  # Minimum logo area relative to box area

# Box detection state variables
last_detection_time = 0
box_is_present = False  # Flag to track box presence
last_absence_time = 0   # Track when box was last absent
min_time = 5
mouth_open = False      # Flag to track mouth state
mouth_open_time = 0     # Time when mouth was opened

# --- Wave Detection Configuration ---
WAVE_DURATION_THRESHOLD = 1.0  # Seconds required for a wave to be confirmed
MOVEMENT_THRESHOLD = 0.03      # Minimum horizontal movement range (normalized) to be considered waving
OPEN_HAND_THRESHOLD = 0.05     # Max distance between finger tips and MCP joints for open hand (normalized)
HISTORY_LENGTH = 15            # Number of frames to track horizontal movement
COOLDOWN_PERIOD = 3.0          # Seconds to wait after detecting a wave before detecting again

# Mode configuration
PLAY_MODE = False       # Flag to track if robot is in play mode

# Screen coverage threshold (70%)
BROWN_COVERAGE_THRESHOLD = 0.70

# Message display configuration
MESSAGE_TIMEOUT = 3.0  # How long to show the message (seconds)
current_message = None  # Tuple of (message, end_time)

# ROS2 node
ros_node = None
unity_client = None
unity_request = None
mouth_position = 90  # Default mouth position
mouth_increment = 10  # How much to change the mouth position by

# --- ROS2 Functions ---
def init_ros():
    """Initialize ROS2 node and clients"""
    global ros_node, unity_client, unity_request
    
    # Initialize ROS2 if not already initialized
    if not rclpy.ok():
        try:
            rclpy.init()
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            print("Running in standalone mode without ROS2 control")
            return False
        
    # Create node
    try:
        ros_node = Node('computer_vision_controller')
    except Exception as e:
        print(f"Failed to create ROS2 node: {e}")
        print("Running in standalone mode without ROS2 control")
        return False
    
    # Create the UnityAnimateService client
    try:
        unity_client = ros_node.create_client(UnityAnimateService, 'UnityAnimate_srv')
        unity_request = UnityAnimateService.Request()
        
        # Wait for the service to be available
        timeout = 5.0  # seconds
        if not unity_client.wait_for_service(timeout_sec=timeout):
            ros_node.get_logger().error('UnityAnimate_srv service not available')
            print("Error: UnityAnimate_srv service not found.")
            return False
        else:
            ros_node.get_logger().info('Connected to UnityAnimate_srv service')
            print("Successfully connected to robot control service.")
            return True
            
    except Exception as e:
        ros_node.get_logger().error(f'Failed to create ROS2 client: {e}')
        print(f"Error connecting to robot services: {e}")
        return False

def set_mode(mode_name):
    """Set robot mode using ROS2 service - same as Joy_controller.py"""
    global ros_node, unity_client, unity_request, PLAY_MODE
    
    if ros_node is None or unity_client is None or unity_request is None:
        # Silent fail in vision-only mode
        PLAY_MODE = (mode_name == "play")
        print(f"Would set mode to '{mode_name}' (Unity service not available)")
        return False
    
    try:
        # Set the mode in the request - using same approach as Joy_controller.py
        unity_request.mode = mode_name
        
        # Call service asynchronously
        future = unity_client.call_async(unity_request)
        
        # Log the mode change
        ros_node.get_logger().info(f"Setting mode to: {mode_name}")
        
        # Update mode flag
        PLAY_MODE = (mode_name == "play")
        
        return True
    except Exception as e:
        print(f"Failed to set mode: {e}")
        # Still update the internal flag so UI shows correctly
        PLAY_MODE = (mode_name == "play")
        return False

def set_mouth_position(position):
    """Set mouth position - Joy_controller.py just updates a variable"""
    global mouth_position
    
    # Update the mouth position
    mouth_position = position
    
    # Log the position change
    if ros_node is not None:
        ros_node.get_logger().info(f"Setting mouth position to: {position}")
    
    return True

def open_mouth():
    """Open the mouth (set to maximum position)"""
    global mouth_position, mouth_increment
    mouth_position = min(190, mouth_position + mouth_increment)
    return set_mouth_position(mouth_position)

def close_mouth():
    """Close the mouth (set to minimum position)"""
    global mouth_position, mouth_increment
    mouth_position = max(116, mouth_position - mouth_increment)
    return set_mouth_position(mouth_position)

# --- Box Detection Functions ---
def detect_boxes(frame):
    global last_detection_time, box_is_present, last_absence_time, mouth_open, mouth_open_time
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create combined brown mask
    mask1 = cv2.inRange(hsv, LOWER_BROWN1, UPPER_BROWN1)
    mask2 = cv2.inRange(hsv, LOWER_BROWN2, UPPER_BROWN2)
    mask3 = cv2.inRange(hsv, LOWER_BROWN3, UPPER_BROWN3)
    mask4 = cv2.inRange(hsv, LOWER_BROWN4, UPPER_BROWN4)
    brown_mask = cv2.bitwise_or(cv2.bitwise_or(cv2.bitwise_or(mask1, mask2), mask3), mask4)
    
    # Find brown regions
    contours, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:  # Minimum box area
            continue
            
        # Check if contour is roughly rectangular
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) < 4 or len(approx) > 6:
            continue
            
        # Get rotated rectangle
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = box.astype(np.intp)
        
        # Check for black logo inside the box
        if has_black_logo(frame, cnt):
            boxes.append(box)
            break  # Stop after first valid detection
            
    current_time = time.time()
    
    if boxes:
        if not box_is_present and (current_time - last_absence_time) > min_time:
            show_event_message("Box Detected!")
            box_is_present = True
            last_detection_time = current_time
            
            # Open mouth when box is detected (using ROS2)
            open_mouth()
            mouth_open = True
            mouth_open_time = current_time
            print("Opening mouth")
            
            return True, boxes[0], boxes
        elif box_is_present:
            # Box is still there but already detected
            return True, boxes[0], boxes
        else:
            # Box is there but cooling down
            return False, None, boxes
    else:
        if box_is_present:
            # Box just disappeared, update absence time
            last_absence_time = current_time
        box_is_present = False
        return False, None, boxes

def has_black_logo(frame, contour):
    # Create mask for the current contour
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    
    # Get ROI coordinates
    x, y, w, h = cv2.boundingRect(contour)
    roi = frame[y:y+h, x:x+w]
    roi_mask = mask[y:y+h, x:x+w]
    
    if roi.size == 0:
        return False
    
    # Detect black pixels in ROI
    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    black_mask = cv2.inRange(roi_hsv, (0, 0, 0), (180, 255, BLACK_VALUE_THRESH))
    black_mask = cv2.bitwise_and(black_mask, roi_mask)
    
    # Find black contours
    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not black_contours:
        return False
    
    # Check if any black region is sufficiently large
    contour_area = cv2.contourArea(contour)
    for bc in black_contours:
        area = cv2.contourArea(bc)
        if area / contour_area > MIN_LOGO_RATIO:
            return True
    
    return False

# --- Wave Detection Initialization ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Wave detection state variables
wave_start_time = None
last_wave_detected_time = 0
x_history = []
waving_confirmed = False
wave_status_text = ""

def show_event_message(message):
    """Show an event message for 3 seconds"""
    global current_message
    current_time = time.time()
    current_message = (message, current_time + MESSAGE_TIMEOUT)

def draw_message(frame):
    """Draw the current event message if active"""
    global current_message
    if current_message:
        message, end_time = current_message
        current_time = time.time()
        if current_time <= end_time:
            # Draw message in center of screen with larger font
            font_size = 1.5
            thickness = 3
            font = cv2.FONT_HERSHEY_SIMPLEX
            
            # Get text size
            (text_width, text_height), _ = cv2.getTextSize(message, font, font_size, thickness)
            
            # Calculate center position
            height, width = frame.shape[:2]
            x = (width - text_width) // 2
            y = (height + text_height) // 2
            
            # Draw text with black outline for better visibility
            cv2.putText(frame, message, (x, y), font, font_size, (0, 0, 0), thickness + 2)
            cv2.putText(frame, message, (x, y), font, font_size, (0, 255, 0), thickness)
        else:
            current_message = None

def main():
    global wave_start_time, last_wave_detected_time, x_history, waving_confirmed, wave_status_text
    global PLAY_MODE, mouth_open, mouth_open_time, ros_node, global_frame
    
    # Initialize ROS2 (but continue even if it fails)
    ros_initialized = init_ros()
    
    # Start web server in a separate thread
    web_thread = threading.Thread(target=start_web_server, args=(args.web_port,), daemon=True)
    web_thread.start()
    
    # Get the camera IP and set resolution
    CAMERA_IP = get_camera_ip()
    print(f"Setting up camera at IP: {CAMERA_IP}")
    
    # Set camera resolution to VGA
    if not set_camera_resolution(CAMERA_IP, "vga"):
        print("Warning: Could not set camera resolution, continuing with default settings")
    
    # Initialize WiFi camera connection
    STREAM_URL = f"http://{CAMERA_IP}:81/stream"
    print(f"Connecting to stream: {STREAM_URL}")
    
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print(f"Error: Could not connect to the camera stream at {STREAM_URL}")
        return
    
    # Verify the camera resolution
    if not verify_camera_resolution(cap, 640, 480):
        print("Warning: Could not verify VGA resolution, continuing with current settings")
    
    # Get the actual camera resolution
    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Final camera resolution: {original_width}x{original_height}")
    
    # Optimize camera settings for the detected resolution
    cap.set(cv2.CAP_PROP_FPS, 15)  # Reduced FPS for stability
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # Minimize frame buffer for lower latency
    
    print(f"Successfully connected to {STREAM_URL}")
    print("Starting computer vision... press 'q' to quit.")
    
    # Only create window if GUI is enabled
    if not args.no_gui:
        try:
            cv2.namedWindow('Combined Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Combined Detection', original_width, original_height)
        except Exception as e:
            print(f"Warning: Could not create GUI window: {e}")
            print("Continuing in headless mode...")
            args.no_gui = True
    
    # Initialize hand tracking with optimized settings
    hands = mp_hands.Hands(
        model_complexity=1,  # Increased complexity for better detection
        min_detection_confidence=0.7,  # Higher threshold for reliability
        min_tracking_confidence=0.7,
        max_num_hands=1)
    
    # Add variables for status tracking
    last_box_status = False
    last_wave_status = False
    
    while True:
        # Process ROS2 callbacks if ROS is initialized
        if ros_node is not None and ros_initialized:
            try:
                rclpy.spin_once(ros_node, timeout_sec=0.001)
            except Exception:
                # Ignore ROS errors during operation
                pass
        
        # Read frame from WiFi stream
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to receive frame. Reconnecting...")
            cap.release()
            time.sleep(1)
            cap = cv2.VideoCapture(STREAM_URL)
            continue
        
        # Create a copy of the frame for hand detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        display_frame = frame.copy()
        
        # Check if we need to close the mouth (2 seconds after opening)
        current_time = time.time()
        if mouth_open and current_time - mouth_open_time >= 2.0:
            close_mouth()
            mouth_open = False
        
        # --- Box Detection ---
        box_present, box, potential_boxes = detect_boxes(frame)
        
        # Print box status if it changed
        if box_present and not last_box_status:
            print("Box Detected")
            last_box_status = True
        elif not box_present and last_box_status:
            last_box_status = False
        
        # --- Hand Detection ---
        # Skip hand detection if box is present to avoid false detections
        if box_present:
            wave_start_time = None
            x_history.clear()
            waving_confirmed = False
            wave_status_text = ""
            results = None
        else:
            # To improve performance, mark the image as not writeable
            frame_rgb.flags.writeable = False
            results = hands.process(frame_rgb)
            frame_rgb.flags.writeable = True

        current_time = time.time()
        is_waving_currently = False
        is_hand_open = False
        
        if results and results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                mp_drawing.draw_landmarks(
                    display_frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
                
                # --- Hand Openness Check ---
                # Get key landmarks
                wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
                pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
                
                index_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                middle_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
                ring_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP]
                pinky_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
                
                # Calculate distances from tips to MCP joints
                dist_index = np.linalg.norm([index_tip.x - index_mcp.x, index_tip.y - index_mcp.y])
                dist_middle = np.linalg.norm([middle_tip.x - middle_mcp.x, middle_tip.y - middle_mcp.y])
                dist_ring = np.linalg.norm([ring_tip.x - ring_mcp.x, ring_tip.y - ring_mcp.y])
                dist_pinky = np.linalg.norm([pinky_tip.x - pinky_mcp.x, pinky_tip.y - pinky_mcp.y])
                
                # Calculate hand size for normalization
                hand_size = np.linalg.norm([
                    wrist.x - index_mcp.x,
                    wrist.y - index_mcp.y
                ])
                
                # Normalize distances by hand size
                if hand_size > 0:
                    dist_index /= hand_size
                    dist_middle /= hand_size
                    dist_ring /= hand_size
                    dist_pinky /= hand_size
                
                # Check if fingers are extended
                extended_fingers = 0
                if dist_index > OPEN_HAND_THRESHOLD: extended_fingers += 1
                if dist_middle > OPEN_HAND_THRESHOLD: extended_fingers += 1
                if dist_ring > OPEN_HAND_THRESHOLD: extended_fingers += 1
                if dist_pinky > OPEN_HAND_THRESHOLD: extended_fingers += 1
                
                if extended_fingers >= 3:  # Require at least 3 fingers extended
                    is_hand_open = True
                
                # --- Waving Motion Check ---
                # Use wrist landmark x-coordinate for horizontal movement tracking
                wrist_x = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x
                x_history.append(wrist_x)
                
                # Keep history length limited
                if len(x_history) > HISTORY_LENGTH:
                    x_history.pop(0)
                
                # Check for waving if we have enough history
                if len(x_history) == HISTORY_LENGTH:
                    x_range = np.max(x_history) - np.min(x_history)
                    
                    # Check if horizontal movement range exceeds threshold
                    if x_range > MOVEMENT_THRESHOLD:
                        is_waving_currently = True
                
                # Print hand openness status
                if is_hand_open and last_wave_status != "open":
                    print("✋ Hand is open")
                    last_wave_status = "open"
                elif not is_hand_open and last_wave_status != "closed":
                    print("✋ Hand is closed")
                    last_wave_status = "closed"
                
                # Print waving status
                if is_waving_currently and not last_wave_status:
                    print("Waving Detected")
                    last_wave_status = True
                elif not is_waving_currently and last_wave_status:
                    last_wave_status = False
                
                # We only process one hand for simplicity
                break
        
        else:
            if last_wave_status != "none":
                print("No hands detected")
                last_wave_status = "none"
        
        # --- State Logic for Wave Confirmation ---
        if is_hand_open and is_waving_currently:
            if wave_start_time is None:
                if current_time - last_wave_detected_time > COOLDOWN_PERIOD:
                    wave_start_time = current_time
                    waving_confirmed = False
            else:
                if not waving_confirmed and (current_time - wave_start_time >= WAVE_DURATION_THRESHOLD):
                    if current_time - last_wave_detected_time > COOLDOWN_PERIOD:
                        show_event_message("Wave Detected!")
                        waving_confirmed = True
                        last_wave_detected_time = current_time
                        
                        # Change to play mode when wave is detected
                        set_mode("play")
                        print("Wave detected! Changing to PLAY mode")
        
        elif waving_confirmed:
            wave_start_time = None
            if not (is_hand_open and is_waving_currently):
                wave_status_text = ""
                waving_confirmed = False
                print("Waving stopped or hand changed.")
        
        else:  # Hand is closed, not moving enough, or disappeared
            wave_start_time = None
            if not waving_confirmed:
                wave_status_text = ""
        
        # --- Draw Box Detection ---
        if box_present:
            # Draw green box around detected package
            cv2.drawContours(display_frame, [box], 0, (0, 255, 0), 2)
            box_status_text = "Package for CS Lab Detected"
        else:
            box_status_text = "No package detected"
        
        # --- Display Status ---
        cv2.putText(display_frame, box_status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        if wave_status_text:
            cv2.putText(display_frame, wave_status_text, (10, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display mouth state and mode
        mode_text = "PLAY MODE" if PLAY_MODE else "NORMAL MODE"
        cv2.putText(display_frame, mode_text, (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        
        mouth_text = f"Mouth: {mouth_position}"
        cv2.putText(display_frame, mouth_text, (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Draw the event message (if active)
        draw_message(display_frame)
        
        # Update the global frame for the web server
        with frame_lock:
            global_frame = display_frame.copy()
        
        # Only show the window if GUI is enabled
        if not args.no_gui:
            try:
                # --- Show combined detection results ---
                cv2.imshow('Combined Detection', display_frame)
                cv2.resizeWindow('Combined Detection', original_width, original_height)  # Maintain original resolution
                
                # --- Exit Condition ---
                # Slightly increased wait time for higher resolution processing
                if cv2.waitKey(2) & 0xFF == ord('q'):
                    break
            except Exception as e:
                print(f"Warning: GUI operation failed: {e}")
                print("Continuing in headless mode...")
                args.no_gui = True
        else:
            # In no-gui mode, we still need a way to exit
            # Check for keyboard input without waiting
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    # --- Cleanup ---
    hands.close()
    cap.release()
    if not args.no_gui:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    
    # Shutdown ROS2
    if ros_node is not None:
        ros_node.destroy_node()
    rclpy.shutdown()
    
    print("Program terminated.")

if __name__ == "__main__":
    main() 