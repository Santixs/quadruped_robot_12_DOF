#!/usr/bin/env python3
import cv2
import numpy as np
import time
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import subprocess
import re

#ros2 launch triceratops_quadruped_robot launch_vision_integration.py

# --- WiFi Camera Configuration ---
# Default STREAM_URL as fallback
DEFAULT_STREAM_URL = "http://192.168.41.36:81/stream"

def get_camera_ip():
    """
    Extract camera IP address from the output of 'cat /dev/ttyACM0'
    Returns the camera stream URL if found, otherwise returns the default URL
    """
    try:
        # Run the command to get the output from the terminal
        result = subprocess.run(['cat', '/dev/ttyACM0'], 
                               capture_output=True, 
                               text=True, 
                               timeout=2)
        
        # Search for the IP address in the output
        # Pattern looks for: http://192.168.xx.xx
        match = re.search(r'http://(\d+\.\d+\.\d+\.\d+)', result.stdout)
        
        if match:
            ip_address = match.group(1)
            print(f"Camera IP detected: {ip_address}")
            return f"http://{ip_address}:81/stream"
        else:
            print(f"Camera IP not found in output. Using default.")
            return DEFAULT_STREAM_URL
            
    except (subprocess.SubprocessError, subprocess.TimeoutExpired) as e:
        print(f"Error getting camera IP: {e}")
        return DEFAULT_STREAM_URL

# Get the camera stream URL
STREAM_URL = get_camera_ip()

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

# --- Wave Detection Configuration ---
WAVE_DURATION_THRESHOLD = 1.0  # Seconds required for a wave to be confirmed
MOVEMENT_THRESHOLD = 0.03      # Minimum horizontal movement range (normalized) to be considered waving
OPEN_HAND_THRESHOLD = 0.05     # Max distance between finger tips and MCP joints for open hand (normalized)
HISTORY_LENGTH = 15            # Number of frames to track horizontal movement
COOLDOWN_PERIOD = 3.0          # Seconds to wait after detecting a wave before detecting again

# Screen coverage threshold (70%)
BROWN_COVERAGE_THRESHOLD = 0.70

# Message display configuration
MESSAGE_TIMEOUT = 3.0  # How long to show the message (seconds)
current_message = None  # Tuple of (message, end_time)

# --- Box Detection Functions ---
def detect_boxes(frame):
    global last_detection_time, box_is_present, last_absence_time
    
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

class ComputerVisionNode(Node):
    def __init__(self):
        super().__init__('computer_vision_node')
        
        # Publishers for detection events
        self.wave_publisher = self.create_publisher(Bool, 'wave_detected', 10)
        self.box_publisher = self.create_publisher(Bool, 'box_detected', 10)
        self.body_pub = self.create_publisher(String, 'body_pose', 10)
        
        # Get the camera stream URL (try again in case it failed earlier)
        stream_url = STREAM_URL
        
        # Initialize camera
        self.cap = cv2.VideoCapture(stream_url)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not connect to the camera stream at {stream_url}")
            # Try one more time with a fresh IP address
            stream_url = get_camera_ip()
            self.cap = cv2.VideoCapture(stream_url)
            if not self.cap.isOpened():
                self.get_logger().error(f"Second attempt failed. Could not connect to the camera stream at {stream_url}")
                return
        
        # Set SVGA resolution (800x600)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        
        # Optimize camera settings for performance
        self.cap.set(cv2.CAP_PROP_FPS, 20)  # Lower target FPS for higher resolution
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # Minimize frame buffer for lower latency
        
        self.get_logger().info(f"Successfully connected to {stream_url}")
        
        # Create window for display
        cv2.namedWindow('Computer Vision', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Computer Vision', 800, 600)
        
        # Initialize hand tracking
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
            max_num_hands=1)
        
        # State variables (moved from global to instance)
        self.wave_start_time = None
        self.last_wave_detected_time = 0
        self.x_history = []
        self.waving_confirmed = False
        self.wave_status_text = ""
        self.box_is_present = False
        self.last_detection_time = 0
        self.last_absence_time = 0
        
        # Create a timer to process frames
        self.create_timer(0.05, self.process_frame)  # 20 FPS
        
        self.get_logger().info("Computer Vision Node has started")
    
    def process_frame(self):
        global wave_start_time, last_wave_detected_time, x_history, waving_confirmed, wave_status_text
        global box_is_present, last_detection_time, last_absence_time
        
        # Use instance variables
        wave_start_time = self.wave_start_time
        last_wave_detected_time = self.last_wave_detected_time
        x_history = self.x_history
        waving_confirmed = self.waving_confirmed
        wave_status_text = self.wave_status_text
        box_is_present = self.box_is_present
        last_detection_time = self.last_detection_time
        last_absence_time = self.last_absence_time
        
        # Read frame from WiFi stream
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to receive frame. Reconnecting...")
            self.cap.release()
            time.sleep(1)
            self.cap = cv2.VideoCapture(STREAM_URL)
            return
        
        # Create a copy of the frame for hand detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        display_frame = frame.copy()
        
        # --- Box Detection ---
        box_present, box, potential_boxes = detect_boxes(frame)
        
        # Publish box detection event if detected
        if box_present and not self.box_is_present:
            self.get_logger().info("Box detected! Publishing event.")
            msg = Bool()
            msg.data = True
            self.box_publisher.publish(msg)
            
            # Send command to open mouth
            body_msg = String()
            body_msg.data = "puppy_mode:200:190"  # Keep current mode, set mouth to fully open
            self.body_pub.publish(body_msg)
            
            # Schedule a timer to close the mouth after 2 seconds
            timer = threading.Timer(2.0, self.close_mouth)
            timer.daemon = True
            timer.start()
        
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
            results = self.hands.process(frame_rgb)
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
                
                # We only process one hand for simplicity
                break
        
        else:  # No hands detected
            x_history.clear()
            wave_start_time = None
        
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
                        
                        # Publish wave detection event
                        self.get_logger().info("Wave detected! Publishing event.")
                        msg = Bool()
                        msg.data = True
                        self.wave_publisher.publish(msg)
                        
                        # Send command to switch to puppy mode
                        body_msg = String()
                        body_msg.data = "puppy_mode:200:150"  # puppy_mode with default head/mouth
                        self.body_pub.publish(body_msg)
        
        elif waving_confirmed:
            wave_start_time = None
            if not (is_hand_open and is_waving_currently):
                wave_status_text = ""
                waving_confirmed = False
        
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
        
        # Draw the event message (if active)
        draw_message(display_frame)
        
        # --- Show combined detection results ---
        cv2.imshow('Computer Vision', display_frame)
        cv2.resizeWindow('Computer Vision', 800, 600)
        
        # Update instance variables
        self.wave_start_time = wave_start_time
        self.last_wave_detected_time = last_wave_detected_time
        self.x_history = x_history
        self.waving_confirmed = waving_confirmed
        self.wave_status_text = wave_status_text
        self.box_is_present = box_is_present
        self.last_detection_time = last_detection_time
        self.last_absence_time = last_absence_time
        
        # Process keyboard input (check for exit)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.destroy_node()
            rclpy.shutdown()
    
    def close_mouth(self):
        """Close the mouth after a delay"""
        body_msg = String()
        body_msg.data = "puppy_mode:200:116"  # Keep current mode, close mouth
        self.body_pub.publish(body_msg)
        self.get_logger().info("Closing mouth")
    
    def __del__(self):
        """Clean up resources on node destruction"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'hands'):
            self.hands.close()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ComputerVisionNode()
    rclpy.spin(node)
    
    # Clean up
    node.__del__()
    rclpy.shutdown()
    print("Program terminated.")

if __name__ == "__main__":
    main() 