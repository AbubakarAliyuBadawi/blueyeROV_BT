#!/usr/bin/env python3
import subprocess
import time
import os
from datetime import datetime
import signal
import sys
import psutil

def create_directory_if_not_exists(directory):
    """Create directory if it doesn't exist"""
    if not os.path.exists(directory):
        os.makedirs(directory)
        print(f"Created directory: {directory}")

def cleanup_processes(processes):
    """Clean termination of all processes"""
    print("\nTerminating processes...")
    
    for name, process in processes.items():
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                print(f"{name} process terminated")
            except Exception as e:
                print(f"Error terminating {name}: {e}")
    
    # Additional cleanup to ensure all related ROS processes are terminated
    print("Cleaning up any remaining ROS processes...")
    try:
        cleanup_cmd = "pkill -f 'ros2|mundus_mir|blueye_bt'"
        subprocess.run(cleanup_cmd, shell=True)
    except Exception as e:
        print(f"Cleanup warning: {e}")
    
    print("All processes have been terminated.")

def check_simulation_ready():
    """Check if key ROS nodes are running to verify simulation is ready"""
    try:
        # Run ros2 node list and check if expected nodes are present
        result = subprocess.run(['ros2', 'node', 'list'], 
                              stdout=subprocess.PIPE, 
                              stderr=subprocess.PIPE,
                              text=True)
        
        nodes = result.stdout.strip().split('\n')
        
        # Check for key simulation nodes (customize these to match your simulation)
        # These are example node names - replace with actual node names from your simulation
        key_node_patterns = ['mundus', 'simulator', 'controller']
        
        for pattern in key_node_patterns:
            if not any(pattern in node for node in nodes):
                return False
                
        return len(nodes) > 5  # Assuming at least 5 nodes when simulation is running properly
    except Exception as e:
        print(f"Error checking ROS nodes: {e}")
        return False

def run_simulation_and_capture(wait_minutes=45, test_mode=False):
    """Run simulation, capture screenshot after specified wait time"""
    # Create screenshots directory
    screenshots_dir = os.path.expanduser("~/thesis_simulation_screenshots")
    create_directory_if_not_exists(screenshots_dir)
    
    # Path to simulation and BT workspaces
    sim_workspace = "/home/badawi/Desktop/mundus_mir_simulator"
    bt_workspace = "/home/badawi/Desktop/blueyeROV_BT"
    
    # Script to source and launch simulation
    sim_script = f"""
    cd {sim_workspace}
    source install/setup.bash
    source set_env
    exec ros2 launch mundus_mir_simulator_launch generated_mundus_mir_pipeline_world.launch.py
    """
    
    # Script to source and launch behavior tree
    bt_script = f"""
    cd {bt_workspace}
    source install/setup.bash
    exec ros2 launch blueye_bt blueye_bt.launch.py
    """
    
    active_processes = {}
    
    try:
        # Start simulation process
        print("Starting simulation...")
        sim_process = subprocess.Popen(['bash', '-c', sim_script], 
                                      stdout=subprocess.PIPE, 
                                      stderr=subprocess.PIPE,
                                      preexec_fn=os.setsid)
        active_processes["Simulation"] = sim_process
        
        # Give the simulation time to initialize (minimum 15 seconds)
        max_init_wait = 60  # Maximum seconds to wait for initialization
        print(f"Waiting for simulation to fully initialize (up to {max_init_wait} seconds)...")
        
        # Count down while checking if simulation is ready
        ready = False
        for i in range(max_init_wait):
            remaining = max_init_wait - i
            if i >= 15:  # Only start checking after 15 seconds minimum
                if check_simulation_ready():
                    ready = True
                    print(f"\rSimulation is fully initialized after {i} seconds!        ")
                    break
            
            sys.stdout.write(f"\rWaiting for simulation to initialize... {remaining} seconds remaining ")
            sys.stdout.flush()
            time.sleep(1)
            
        if not ready:
            print("\rSimulation initialization time expired. Continuing anyway...      ")
        
        print("Starting behavior tree...")
        
        # Start behavior tree process
        print("Starting behavior tree...")
        bt_process = subprocess.Popen(['bash', '-c', bt_script],
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE,
                                     preexec_fn=os.setsid)
        active_processes["Behavior Tree"] = bt_process
        
        # Calculate wait time (minutes to seconds)
        wait_time = wait_minutes * 60
        
        print(f"Processes started successfully. Waiting for {wait_minutes} minutes...")
        if test_mode:
            print("[TEST MODE ENABLED - Using very short wait time]")
        
        end_time = datetime.now() + datetime.timedelta(seconds=wait_time)
        print(f"Simulation will complete at approximately: {end_time.strftime('%H:%M:%S')}")
        
        # Wait for specified time, but check process status regularly
        # This allows for better handling of early termination
        elapsed = 0
        check_interval = 5  # Check every 5 seconds
        
        while elapsed < wait_time:
            try:
                time.sleep(min(check_interval, wait_time - elapsed))
                elapsed += check_interval
                
                # Check if processes are still running
                if (sim_process.poll() is not None or bt_process.poll() is not None):
                    print("One of the processes has terminated unexpectedly.")
                    # Take screenshot anyway before exiting
                    break
                
                # Show progress every minute
                if elapsed % 60 == 0 and elapsed > 0:
                    mins_left = (wait_time - elapsed) // 60
                    print(f"Still running... {mins_left} minutes remaining")
                    
            except KeyboardInterrupt:
                print("\nProcess interrupted by user (Ctrl+C)")
                user_input = input("Take screenshot before exiting? (y/n): ").lower()
                if user_input == 'y':
                    break
                else:
                    return
        
        # Take screenshot using gnome-screenshot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        screenshot_path = f"{screenshots_dir}/simulation_{timestamp}.png"
        
        print("Taking screenshot...")
        screenshot_cmd = f"gnome-screenshot -f {screenshot_path}"
        subprocess.run(screenshot_cmd, shell=True)
        print(f"Screenshot saved to: {screenshot_path}")
        
        # Allow a moment for screenshot to complete
        time.sleep(2)
        
    except Exception as e:
        print(f"An error occurred: {e}")
        
    finally:
        # Clean termination of all processes
        cleanup_processes(active_processes)

if __name__ == "__main__":
    # Parse command line arguments
    wait_minutes = 45
    test_mode = False
    
    if len(sys.argv) > 1:
        if sys.argv[1].lower() == "test":
            test_mode = True
            wait_minutes = 0.2  # 12 seconds for testing
            print(f"TEST MODE: Using very short wait time ({wait_minutes} minutes)")
        else:
            try:
                wait_minutes = float(sys.argv[1])
                print(f"Using custom wait time: {wait_minutes} minutes")
            except ValueError:
                print(f"Invalid wait time. Using default of {wait_minutes} minutes.")
    
    run_simulation_and_capture(wait_minutes, test_mode)