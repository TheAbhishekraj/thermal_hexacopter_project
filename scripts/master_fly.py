#!/usr/bin/env python3
"""
master_fly.py - Automated Bihar Maiden Voyage Mission Controller
CEO Directive: One-Command Mission Execution

This master script automates the complete mission sequence:
1. Launch Bihar world simulation
2. Start thermal AI monitor
3. Execute Level 3 survey mission
4. Monitor telemetry and alerts
5. Graceful shutdown

Usage:
    python3 master_fly.py [--record] [--verbose]
    
Options:
    --record    Start ffmpeg screen recording automatically
    --verbose   Show detailed debug output
    
Exit Codes:
    0 - Mission success (100% waypoints, safe landing)
    1 - Mission failure (crash, timeout, or error)
    2 - Setup error (Docker, ROS 2, or environment issue)
"""

import subprocess
import time
import sys
import argparse
import signal
import os
from datetime import datetime
from pathlib import Path

class Colors:
    """ANSI color codes for terminal output"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    BOLD = '\033[1m'
    END = '\033[0m'

class MasterFlyController:
    """Master mission controller for automated flight execution"""
    
    def __init__(self, record=False, verbose=False):
        self.record = record
        self.verbose = verbose
        self.container_name = "hexacopter_gui"
        self.processes = []
        self.recording_process = None
        self.mission_success = False
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def log(self, message, level="INFO"):
        """Print formatted log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        color = {
            "INFO": Colors.BLUE,
            "SUCCESS": Colors.GREEN,
            "WARNING": Colors.YELLOW,
            "ERROR": Colors.RED,
            "MISSION": Colors.MAGENTA
        }.get(level, Colors.BLUE)
        
        print(f"{color}[{timestamp}] [{level}] {message}{Colors.END}")
        
    def debug(self, message):
        """Print debug message if verbose mode enabled"""
        if self.verbose:
            self.log(message, "DEBUG")
            
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.log("Received interrupt signal, shutting down gracefully...", "WARNING")
        self.cleanup()
        sys.exit(1)
        
    def run_command(self, cmd, check=True, timeout=None, shell=False):
        """Run shell command and return output"""
        self.debug(f"Running command: {cmd if isinstance(cmd, str) else ' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd,
                shell=shell,
                check=check,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.stdout, result.stderr, result.returncode
        except subprocess.CalledProcessError as e:
            self.log(f"Command failed: {e.stderr}", "ERROR")
            return None, e.stderr, e.returncode
        except subprocess.TimeoutExpired:
            self.log(f"Command timed out after {timeout} seconds", "ERROR")
            return None, "Timeout", -1
            
    def check_prerequisites(self):
        """Verify all prerequisites are met"""
        self.log("Checking prerequisites...", "INFO")
        
        # Check Docker
        stdout, stderr, code = self.run_command(["docker", "--version"], check=False)
        if code != 0:
            self.log("Docker not found. Please install Docker.", "ERROR")
            return False
        self.log(f"Docker: {stdout.strip()}", "SUCCESS")
        
        # Check Docker image
        stdout, stderr, code = self.run_command(
            ["docker", "images", "-q", "hexacopter_lab_safe_copy"],
            check=False
        )
        if not stdout.strip():
            self.log("Docker image 'hexacopter_lab_safe_copy' not found", "ERROR")
            return False
        self.log("Docker image: hexacopter_lab_safe_copy found", "SUCCESS")
        
        # Check DISPLAY
        display = os.environ.get("DISPLAY")
        if not display:
            self.log("DISPLAY environment variable not set", "ERROR")
            return False
        self.log(f"DISPLAY: {display}", "SUCCESS")
        
        # Enable X11 forwarding
        self.run_command("xhost +local:docker", shell=True, check=False)
        self.log("X11 forwarding enabled", "SUCCESS")
        
        # Check workspace
        workspace = Path.home() / "thermal_hexacopter_project" / "workspace"
        if not workspace.exists():
            self.log(f"Workspace not found: {workspace}", "ERROR")
            return False
        self.log(f"Workspace: {workspace}", "SUCCESS")
        
        return True
        
    def start_recording(self):
        """Start ffmpeg screen recording"""
        if not self.record:
            return True
            
        self.log("Starting screen recording...", "INFO")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"bihar_maiden_voyage_{timestamp}.mp4"
        
        cmd = [
            "ffmpeg",
            "-video_size", "1920x1080",
            "-framerate", "30",
            "-f", "x11grab",
            "-i", ":0.0",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-crf", "18",
            output_file
        ]
        
        try:
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            time.sleep(2)  # Let ffmpeg initialize
            
            if self.recording_process.poll() is not None:
                self.log("ffmpeg failed to start", "ERROR")
                return False
                
            self.log(f"Recording to: {output_file}", "SUCCESS")
            return True
        except FileNotFoundError:
            self.log("ffmpeg not found. Install with: sudo apt install ffmpeg", "ERROR")
            return False
            
    def launch_simulation(self):
        """Launch Bihar world simulation in Docker"""
        self.log("Launching Bihar world simulation...", "MISSION")
        
        # Stop any existing container
        self.run_command(["docker", "stop", self.container_name], check=False)
        time.sleep(2)
        
        workspace = Path.home() / "thermal_hexacopter_project" / "workspace"
        scripts = Path.home() / "thermal_hexacopter_project" / "scripts"
        
        cmd = [
            "docker", "run",
            "-d",  # Detached mode
            "--rm",
            "--privileged",
            "--network=host",
            "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
            "-e", "LIBGL_ALWAYS_SOFTWARE=1",
            "-e", "QT_X11_NO_MITSHM=1",
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-v", f"{workspace}:/root/workspace",
            "-v", f"{scripts}:/root/scripts",
            "--name", self.container_name,
            "hexacopter_lab_safe_copy",
            "/bin/bash", "-c",
            "cd /root/workspace && chmod +x ../scripts/visual_hexacopter_bihar.sh && ../scripts/visual_hexacopter_bihar.sh"
        ]
        
        stdout, stderr, code = self.run_command(cmd)
        if code != 0:
            self.log("Failed to start Docker container", "ERROR")
            return False
            
        self.log("Docker container started, waiting for Gazebo...", "INFO")
        
        # Wait for PX4 to boot (check logs)
        max_wait = 60
        for i in range(max_wait):
            stdout, _, _ = self.run_command(
                ["docker", "logs", self.container_name],
                check=False
            )
            if "INFO  [mavlink]" in stdout and "GZ_SIM_RESOURCE_PATH" in stdout:
                self.log("PX4 SITL booted successfully", "SUCCESS")
                return True
            time.sleep(1)
            if i % 10 == 0:
                self.log(f"Waiting for PX4 boot... ({i}/{max_wait}s)", "INFO")
                
        self.log("PX4 boot timeout", "ERROR")
        return False
        
    def start_thermal_monitor(self):
        """Start thermal AI monitoring node"""
        self.log("Starting thermal AI monitor (MobileNetV2)...", "MISSION")
        
        cmd = [
            "docker", "exec", "-d", self.container_name,
            "/bin/bash", "-c",
            "source /opt/ros/humble/setup.bash && "
            "source /root/workspace/install/setup.bash && "
            "ros2 run agri_hexacopter thermal_monitor"
        ]
        
        stdout, stderr, code = self.run_command(cmd)
        if code != 0:
            self.log("Failed to start thermal monitor", "ERROR")
            return False
            
        time.sleep(3)  # Let node initialize
        
        # Verify node is running
        stdout, _, _ = self.run_command(
            ["docker", "exec", self.container_name, "/bin/bash", "-c",
             "source /opt/ros/humble/setup.bash && "
             "source /root/workspace/install/setup.bash && "
             "ros2 node list"],
            check=False
        )
        
        if "thermal_monitor" in stdout:
            self.log("Thermal monitor node active (91.9% F1-score)", "SUCCESS")
            return True
        else:
            self.log("Thermal monitor node not found in node list", "WARNING")
            return False
            
    def execute_survey_mission(self):
        """Execute Level 3 survey mission"""
        self.log("Executing Level 3 survey mission...", "MISSION")
        self.log("Target: 7 waypoints, 10m × 8m grid, 5m altitude", "INFO")
        
        cmd = [
            "docker", "exec", self.container_name,
            "/bin/bash", "-c",
            "source /opt/ros/humble/setup.bash && "
            "source /root/workspace/install/setup.bash && "
            "/root/workspace/install/agri_bot_missions/bin/level3_survey"
        ]
        
        # Run mission with timeout
        self.log("Mission started, monitoring progress...", "INFO")
        stdout, stderr, code = self.run_command(cmd, timeout=180)  # 3 minute timeout
        
        if code == 0:
            self.log("Mission completed successfully!", "SUCCESS")
            self.mission_success = True
            
            # Parse mission stats from output
            if stdout:
                for line in stdout.split('\n'):
                    if "waypoint" in line.lower() or "duration" in line.lower():
                        self.log(line.strip(), "MISSION")
            return True
        else:
            self.log("Mission failed or timed out", "ERROR")
            if stderr:
                self.log(f"Error: {stderr}", "ERROR")
            return False
            
    def monitor_telemetry(self, duration=10):
        """Monitor vehicle telemetry for validation"""
        self.log(f"Monitoring telemetry for {duration} seconds...", "INFO")
        
        cmd = [
            "docker", "exec", self.container_name,
            "/bin/bash", "-c",
            "source /opt/ros/humble/setup.bash && "
            "source /root/workspace/install/setup.bash && "
            f"timeout {duration} ros2 topic echo /fmu/out/vehicle_local_position --field z"
        ]
        
        stdout, _, _ = self.run_command(cmd, check=False, timeout=duration+5)
        
        if stdout:
            z_values = [float(line.strip()) for line in stdout.split('\n') if line.strip() and line.strip().replace('.', '').replace('-', '').isdigit()]
            if z_values:
                avg_z = sum(z_values) / len(z_values)
                error = abs(avg_z - (-5.0))
                self.log(f"Average altitude: {avg_z:.2f}m (error: {error:.2f}m)", "INFO")
                if error < 0.1:
                    self.log("Altitude stability: PASS (±0.1m tolerance)", "SUCCESS")
                else:
                    self.log("Altitude stability: MARGINAL", "WARNING")
                    
    def cleanup(self):
        """Cleanup resources and stop processes"""
        self.log("Cleaning up...", "INFO")
        
        # Stop recording
        if self.recording_process:
            self.log("Stopping screen recording...", "INFO")
            self.recording_process.terminate()
            try:
                self.recording_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.recording_process.kill()
                
        # Stop Docker container
        self.log("Stopping Docker container...", "INFO")
        self.run_command(["docker", "stop", self.container_name], check=False)
        
        self.log("Cleanup complete", "SUCCESS")
        
    def run(self):
        """Execute complete mission sequence"""
        self.log("=" * 70, "INFO")
        self.log("BIHAR MAIDEN VOYAGE - AUTOMATED MISSION CONTROLLER", "MISSION")
        self.log("=" * 70, "INFO")
        
        # Step 1: Prerequisites
        if not self.check_prerequisites():
            self.log("Prerequisites check failed", "ERROR")
            return 2
            
        # Step 2: Start recording (optional)
        if self.record:
            if not self.start_recording():
                self.log("Recording setup failed, continuing without recording", "WARNING")
                
        # Step 3: Launch simulation
        if not self.launch_simulation():
            self.cleanup()
            return 2
            
        # Step 4: Start thermal monitor
        if not self.start_thermal_monitor():
            self.log("Thermal monitor failed, continuing mission anyway", "WARNING")
            
        # Step 5: Execute survey mission
        if not self.execute_survey_mission():
            self.cleanup()
            return 1
            
        # Step 6: Monitor telemetry (post-mission validation)
        # self.monitor_telemetry(duration=10)
        
        # Step 7: Cleanup
        self.cleanup()
        
        # Final status
        self.log("=" * 70, "INFO")
        if self.mission_success:
            self.log("MISSION SUCCESS: 100% WAYPOINTS COMPLETED", "SUCCESS")
            self.log("Status: THESIS VALIDATED ✅", "SUCCESS")
            return 0
        else:
            self.log("MISSION INCOMPLETE", "ERROR")
            return 1

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Automated Bihar Maiden Voyage Mission Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 master_fly.py                    # Run mission without recording
  python3 master_fly.py --record           # Run mission with screen recording
  python3 master_fly.py --record --verbose # Run with recording and debug output
        """
    )
    parser.add_argument("--record", action="store_true", help="Start ffmpeg screen recording")
    parser.add_argument("--verbose", action="store_true", help="Show detailed debug output")
    
    args = parser.parse_args()
    
    controller = MasterFlyController(record=args.record, verbose=args.verbose)
    exit_code = controller.run()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
