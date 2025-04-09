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
from datetime import datetime

# Add Flask for web streaming
from flask import Flask, Response, render_template
import logging

# Add ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from unity_robotics_demo_msgs.msg import JointState
from unity_robotics_demo_msgs.srv import UnityAnimateService

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
# Logo detection parameters
BLACK_VALUE_THRESH = 45  
MIN_LOGO_AREA = 400      
MAX_LOGO_AREA = 3000     # Reduced to avoid large background areas
DEBUG_MODE = False        # Set to False to disable debug info

# Pattern detection parameters
MIN_WIDTH_HEIGHT_RATIO = 0.8   # More strict ratio for the logo
MAX_WIDTH_HEIGHT_RATIO = 1.2
MIN_EXTENT = 0.15             # Logo has specific coverage pattern
MAX_EXTENT = 0.4              # Lower max extent to avoid background

# Line pattern parameters
MIN_LINE_LENGTH = 20          # Minimum length of lines to consider
MIN_PARALLEL_LINES = 2        # Minimum number of parallel lines
LINE_ANGLE_TOLERANCE = 5      # Degrees tolerance for parallel lines

# Confidence parameters
MIN_CONFIDENCE = 60           # Minimum confidence threshold
DETECTION_BUFFER_SIZE = 3     # Number of frames to average
detection_buffer = []         # Store recent detection confidences

# Time-based detection parameters
DETECTION_TIME_THRESHOLD = 1  # Time in seconds needed for stable detection
MIN_DETECTION_RATIO = 0.4      # Minimum ratio of frames that must exceed confidence threshold
detection_history = []         # Store detection history with timestamps
detection_start_time = None    # Track when continuous detection started
stable_detection = False       # Flag for stable detection state

# Box detection state variables
last_detection_time = 0
box_is_present = False    
last_absence_time = 0      
min_time = 2              
countdown_start = 0        
program_start_time = 0     
INITIAL_DELAY = 1          
last_print_time = 0       

# Mouth state variables
mouth_open = False      # Flag to track mouth state
mouth_open_time = 0     # Time when mouth was opened

# Display style parameters
BORDER_COLOR = (0, 255, 0)  # Green border
TEXT_COLOR = (0, 255, 0)    # Green text
SCAN_COLOR = (0, 255, 0)    # Green scan line
CONTOUR_COLOR = (0, 0, 255) # Red for detected logo
TRACKING_COLOR = (0, 255, 0) # Green for tracking box
CORNER_LENGTH = 20          # Length of corner lines
BORDER_THICKNESS = 1        # Thickness of border
SCAN_LINE_SPEED = 3         # Speed of scanning line
TEXT_SCALE = 0.5           # Size of text

# Face Detection parameters
FACE_CASCADE = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
FACE_COLOR = (0, 0, 255)    # Red for face detection
FACE_TEXT_COLOR = (0, 255, 0)  # Green for face text
MIN_FACE_SIZE = (30, 30)    # Minimum face size to detect
FACE_SCALE_FACTOR = 1.1     # How much the image size is reduced at each image scale
FACE_MIN_NEIGHBORS = 5      # How many neighbors each candidate rectangle should have

def find_parallel_lines(mask, min_length=20):
    """Find sets of parallel lines in the image"""
    # Use probabilistic Hough transform to detect lines
    lines = cv2.HoughLinesP(mask, 1, np.pi/180, 50, 
                           minLineLength=min_length, maxLineGap=10)
    
    if lines is None:
        return [], []
    
    # Calculate line angles
    angles = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
        angles.append(angle)
    
    # Group lines by angle
    horizontal_lines = []
    vertical_lines = []
    
    for i, angle in enumerate(angles):
        # Check if line is horizontal-ish (0° ± tolerance) or vertical-ish (90° ± tolerance)
        if angle < LINE_ANGLE_TOLERANCE or angle > (180 - LINE_ANGLE_TOLERANCE):
            horizontal_lines.append(lines[i])
        elif abs(angle - 90) < LINE_ANGLE_TOLERANCE:
            vertical_lines.append(lines[i])
    
    return horizontal_lines, vertical_lines

def analyze_logo_pattern(contour, mask):
    """Analyze if the contour matches the specific logo pattern"""
    # Get basic shape metrics
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)
    
    # Skip if area is outside bounds
    if area < MIN_LOGO_AREA or area > MAX_LOGO_AREA:
        return 0, None
    
    aspect_ratio = w / float(h)
    if not (MIN_WIDTH_HEIGHT_RATIO <= aspect_ratio <= MAX_WIDTH_HEIGHT_RATIO):
        return 0, None
    
    # Create mask for the contour region
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = contour - [x, y]
    cv2.drawContours(roi_mask, [shifted_contour], 0, 255, -1)
    
    # Find lines in the contour region
    horizontal_lines, vertical_lines = find_parallel_lines(roi_mask, MIN_LINE_LENGTH)
    
    # Calculate pattern scores
    h_lines_score = min(1.0, len(horizontal_lines) / MIN_PARALLEL_LINES)
    v_lines_score = min(1.0, len(vertical_lines) / MIN_PARALLEL_LINES)
    
    # Calculate extent (area coverage)
    extent = area / (w * h)
    if not (MIN_EXTENT <= extent <= MAX_EXTENT):
        return 0, None
    
    # Get the convex hull for solidity calculation
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 0 else 0
    
    # Calculate overall pattern score
    pattern_score = (h_lines_score + v_lines_score) / 2.0
    
    # Calculate final confidence score with weighted components
    confidence = (
        pattern_score * 0.4 +           # Highest weight for parallel lines pattern
        (1.0 - abs(1.0 - aspect_ratio)) * 0.2 +  # Square-ish shape
        (1.0 - abs(0.3 - extent) / 0.3) * 0.2 +  # Specific coverage
        (1.0 - abs(0.6 - solidity) / 0.6) * 0.2   # Specific solidity
    ) * 100
    
    metrics = {
        'area': area,
        'aspect_ratio': aspect_ratio,
        'extent': extent,
        'solidity': solidity,
        'h_lines': len(horizontal_lines),
        'v_lines': len(vertical_lines),
        'pattern_score': pattern_score
    }
    
    return confidence, metrics

def update_detection_history(confidence, current_time):
    """Update detection history and check if detection criteria are met"""
    global detection_history, detection_start_time, stable_detection, last_print_time
    
    # Add current detection to history with timestamp
    detection_history.append((current_time, confidence >= MIN_CONFIDENCE))
    
    # Remove old detections (older than DETECTION_TIME_THRESHOLD)
    while detection_history and (current_time - detection_history[0][0]) > DETECTION_TIME_THRESHOLD:
        detection_history.pop(0)
    
    # Calculate detection ratio in the time window
    if detection_history:
        positive_detections = sum(1 for _, detected in detection_history if detected)
        detection_ratio = positive_detections / len(detection_history)
        
        # Check if we meet the criteria for stable detection
        if detection_ratio >= MIN_DETECTION_RATIO:
            if detection_start_time is None:
                detection_start_time = current_time
            elif current_time - detection_start_time >= DETECTION_TIME_THRESHOLD:
                if not stable_detection:
                    stable_detection = True
                    if current_time - last_print_time > 1:  # Prevent spam printing
                        print("Logo Detected")
                        last_print_time = current_time
        else:
            detection_start_time = None
            stable_detection = False
    else:
        detection_start_time = None
        stable_detection = False
    
    return stable_detection

def detect_faces(frame):
    """Detect faces in the frame"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = FACE_CASCADE.detectMultiScale(
        gray,
        scaleFactor=FACE_SCALE_FACTOR,
        minNeighbors=FACE_MIN_NEIGHBORS,
        minSize=MIN_FACE_SIZE
    )
    return faces

def draw_hacker_style(frame, contour, confidence, is_detected):
    """Add hacker-style visual elements to the frame"""
    height, width = frame.shape[:2]
    display = frame.copy()
    
    # Detect faces
    faces = detect_faces(frame)
    
    # Draw faces first
    for (x, y, w, h) in faces:
        # Draw face rectangle
        cv2.rectangle(display, (x, y), (x+w, y+h), FACE_COLOR, 2)
        
        # Draw corner markers
        corner_len = 15
        # Top-left
        cv2.line(display, (x, y), (x + corner_len, y), FACE_COLOR, 2)
        cv2.line(display, (x, y), (x, y + corner_len), FACE_COLOR, 2)
        # Top-right
        cv2.line(display, (x + w, y), (x + w - corner_len, y), FACE_COLOR, 2)
        cv2.line(display, (x + w, y), (x + w, y + corner_len), FACE_COLOR, 2)
        # Bottom-left
        cv2.line(display, (x, y + h), (x + corner_len, y + h), FACE_COLOR, 2)
        cv2.line(display, (x, y + h), (x, y + h - corner_len), FACE_COLOR, 2)
        # Bottom-right
        cv2.line(display, (x + w, y + h), (x + w - corner_len, y + h), FACE_COLOR, 2)
        cv2.line(display, (x + w, y + h), (x + w, y + h - corner_len), FACE_COLOR, 2)
        
        # Add "Friendly Human Detected" text with background
        text = "Friendly Human Detected"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Draw text background
        padding = 5
        cv2.rectangle(display, 
                     (x, y - text_height - 2*padding),
                     (x + text_width + 2*padding, y),
                     FACE_COLOR, -1)
        
        # Draw text
        cv2.putText(display, text,
                   (x + padding, y - padding),
                   font, font_scale, FACE_TEXT_COLOR, thickness, cv2.LINE_AA)
        
        # Add tracking data
        tracking_text = f"Track ID: HUMAN_{x}_{y}"
        cv2.putText(display, tracking_text,
                   (x, y + h + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, FACE_TEXT_COLOR, 1, cv2.LINE_AA)
        
        # Add dimensions
        dim_text = f"Size: {w}x{h}"
        cv2.putText(display, dim_text,
                   (x, y + h + 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, FACE_TEXT_COLOR, 1, cv2.LINE_AA)
    
    # Add scanning effect
    scan_line_pos = int(abs(np.sin(time.time() * SCAN_LINE_SPEED)) * height)
    cv2.line(display, (0, scan_line_pos), (width, scan_line_pos), 
             SCAN_COLOR, 1, cv2.LINE_AA)
    
    # Draw border
    cv2.rectangle(display, (0, 0), (width-1, height-1), 
                 BORDER_COLOR, BORDER_THICKNESS)
    
    # Draw corners
    def draw_corner(x, y, dx1, dy1, dx2, dy2):
        cv2.line(display, (x, y), (x + dx1, y), BORDER_COLOR, 2)
        cv2.line(display, (x, y), (x, y + dy2), BORDER_COLOR, 2)
    
    # Draw corners at each corner of the frame
    draw_corner(0, 0, CORNER_LENGTH, 0, 0, CORNER_LENGTH)  # Top-left
    draw_corner(width-1, 0, -CORNER_LENGTH, 0, 0, CORNER_LENGTH)  # Top-right
    draw_corner(0, height-1, CORNER_LENGTH, 0, 0, -CORNER_LENGTH)  # Bottom-left
    draw_corner(width-1, height-1, -CORNER_LENGTH, 0, 0, -CORNER_LENGTH)  # Bottom-right
    
    # Add timestamp and status
    current_time = datetime.now().strftime("%H:%M:%S.%f")[:-4]
    cv2.putText(display, f"TIME: {current_time}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_COLOR, 1, cv2.LINE_AA)
    
    # Status indicators
    status_text = "STATUS: SCANNING..."
    if len(faces) > 0:
        status_text = f"STATUS: {len(faces)} HUMAN(S) DETECTED"
    if is_detected:
        status_text += " | TARGET ACQUIRED"
    cv2.putText(display, status_text, (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_COLOR, 1, cv2.LINE_AA)
    
    # Draw confidence meter if confidence > 0
    if confidence > 0:
        meter_width = 100
        meter_height = 10
        filled_width = int((confidence / 100) * meter_width)
        cv2.rectangle(display, (width - meter_width - 10, 10),
                     (width - 10, 10 + meter_height),
                     BORDER_COLOR, 1)
        cv2.rectangle(display, (width - meter_width - 10, 10),
                     (width - meter_width - 10 + filled_width, 10 + meter_height),
                     BORDER_COLOR, -1)
        cv2.putText(display, f"CONF: {confidence:.1f}%", 
                   (width - meter_width - 10, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_COLOR, 1, cv2.LINE_AA)
    
    # Draw target box and analysis markers if contour is detected
    if contour is not None and confidence > MIN_CONFIDENCE:
        # Draw the main contour
        cv2.drawContours(display, [contour], 0, CONTOUR_COLOR, 2)
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(contour)
        
        # Draw tracking box corners
        def draw_tracking_corner(corner_x, corner_y, dx1, dy1, dx2, dy2):
            cv2.line(display, (corner_x, corner_y), 
                    (corner_x + dx1, corner_y + dy1), 
                    TRACKING_COLOR, 1, cv2.LINE_AA)
            cv2.line(display, (corner_x, corner_y), 
                    (corner_x + dx2, corner_y + dy2), 
                    TRACKING_COLOR, 1, cv2.LINE_AA)
        
        # Draw tracking box at each corner of the bounding rectangle
        corner_len = 10
        draw_tracking_corner(x, y, corner_len, 0, 0, corner_len)  # Top-left
        draw_tracking_corner(x+w, y, -corner_len, 0, 0, corner_len)  # Top-right
        draw_tracking_corner(x, y+h, corner_len, 0, 0, -corner_len)  # Bottom-left
        draw_tracking_corner(x+w, y+h, -corner_len, 0, 0, -corner_len)  # Bottom-right
        
        # Draw target lines
        center_x = x + w//2
        center_y = y + h//2
        line_length = 10
        gap = 5
        
        # Horizontal target lines
        cv2.line(display, (center_x - line_length - gap, center_y),
                (center_x - gap, center_y), TRACKING_COLOR, 1, cv2.LINE_AA)
        cv2.line(display, (center_x + gap, center_y),
                (center_x + line_length + gap, center_y), TRACKING_COLOR, 1, cv2.LINE_AA)
        
        # Vertical target lines
        cv2.line(display, (center_x, center_y - line_length - gap),
                (center_x, center_y - gap), TRACKING_COLOR, 1, cv2.LINE_AA)
        cv2.line(display, (center_x, center_y + gap),
                (center_x, center_y + line_length + gap), TRACKING_COLOR, 1, cv2.LINE_AA)
        
        # Add dimensions
        cv2.putText(display, f"{w}x{h}", (x + w + 5, y + h//2),
                   cv2.FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_COLOR, 1, cv2.LINE_AA)
    
    return display

def detect_boxes(frame):
    global last_detection_time, box_is_present, last_absence_time, program_start_time, detection_buffer, last_print_time
    
    if time.time() - program_start_time < INITIAL_DELAY:
        return False, None, 0, None
    
    # Convert to grayscale and enhance contrast
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)  # Enhance contrast
    
    # Apply adaptive thresholding
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                 cv2.THRESH_BINARY_INV, 11, 2)
    
    # Apply morphological operations
    kernel = np.ones((3,3), np.uint8)
    black_mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
    
    if DEBUG_MODE:
        cv2.imshow('Black Mask', black_mask)
        debug_frame = frame.copy()
    
    # Find contours with hierarchy to identify nested contours
    contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    current_time = time.time()
    max_confidence = 0
    best_metrics = None
    best_contour = None
    
    # Analyze each contour
    for i, cnt in enumerate(contours):
        confidence, metrics = analyze_logo_pattern(cnt, black_mask)
        
        if confidence > max_confidence:
            max_confidence = confidence
            best_metrics = metrics
            best_contour = cnt
    
    if DEBUG_MODE and best_contour is not None and max_confidence > MIN_CONFIDENCE:
        cv2.drawContours(debug_frame, [best_contour], 0, (0, 0, 255), 2)
        if best_metrics:
            y_pos = 60
            for key, value in best_metrics.items():
                text = f"{key}: {value:.2f}"
                cv2.putText(debug_frame, text, (10, y_pos),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y_pos += 20
        cv2.imshow('Debug Detection', debug_frame)
    
    # Update detection buffer
    detection_buffer.append(max_confidence)
    if len(detection_buffer) > DETECTION_BUFFER_SIZE:
        detection_buffer.pop(0)
    
    # Calculate average confidence over buffer
    avg_confidence = sum(detection_buffer) / len(detection_buffer)
    
    # Update time-based detection history
    is_stable = update_detection_history(avg_confidence, current_time)
    
    # Detection logic
    if avg_confidence > MIN_CONFIDENCE:
        if not box_is_present and (current_time - last_absence_time) > min_time:
            box_is_present = True
            last_detection_time = current_time
            countdown_start = current_time
            return True, None, avg_confidence, best_contour
        elif box_is_present:
            return True, None, avg_confidence, best_contour
    
    if box_is_present:
        last_absence_time = current_time
    box_is_present = False
    return False, None, avg_confidence, best_contour

# --- Wave Detection Configuration ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Wave detection parameters
WAVE_DURATION_THRESHOLD = 1.0  # Seconds required for a wave to be confirmed
MOVEMENT_THRESHOLD = 0.03      # Minimum horizontal movement range (normalized) to be considered waving
OPEN_HAND_THRESHOLD = 0.05     # Max distance between finger tips and MCP joints for open hand (normalized)
HISTORY_LENGTH = 15            # Number of frames to track horizontal movement
COOLDOWN_PERIOD = 3.0          # Seconds to wait after detecting a wave before detecting again

# Wave detection state variables
wave_start_time = None
last_wave_detected_time = 0
x_history = []
waving_confirmed = False
wave_status_text = ""

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
    global PLAY_MODE, mouth_open, mouth_open_time, ros_node, global_frame, program_start_time
    
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
    
    # Initialize program start time
    program_start_time = time.time()
    
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
        box_present, box, confidence, best_contour = detect_boxes(frame)
        
        # Print box status if it changed
        if box_present and not last_box_status:
            print("Box Detected")
            last_box_status = True
            # Open mouth when box is detected (using ROS2)
            open_mouth()
            mouth_open = True
            mouth_open_time = current_time
            print("Opening mouth")
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
        
        # Apply hacker-style visualization
        display_frame = draw_hacker_style(display_frame, best_contour, confidence, box_present)
        
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