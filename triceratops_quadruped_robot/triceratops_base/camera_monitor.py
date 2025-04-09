#!/usr/bin/env python3
import subprocess
import time
import os
import re

OUTPUT_FILE = "camera_output.txt"

def monitor_camera():
    # Create or clear the output file
    with open(OUTPUT_FILE, 'w') as f:
        f.write("")
    
    print(f"Monitoring /dev/ttyACM0. Output will be saved to {OUTPUT_FILE}")
    print("Waiting for camera connection...")
    
    while True:
        try:
            # Run the command with a timeout of 1 second
            result = subprocess.run(['cat', '/dev/ttyACM1'], 
                                 capture_output=True, 
                                 text=True, 
                                 timeout=1)
            
            # If we got output, write it to the file
            if result.stdout:
                with open(OUTPUT_FILE, 'w') as f:
                    f.write(result.stdout)
                
                # Check if we found the URL
                match = re.search(r'http://(\d+\.\d+\.\d+\.\d+)', result.stdout)
                if match:
                    print(f"Camera URL found: {match.group(0)}")
                    # Keep monitoring but don't exit - the camera might disconnect/reconnect
                
        except subprocess.TimeoutExpired:
            # Timeout is normal - just means no new data
            pass
        except Exception as e:
            print(f"Error reading from device: {e}")
        
        # Small delay to prevent CPU hogging
        time.sleep(0.1)

if __name__ == "__main__":
    monitor_camera() 