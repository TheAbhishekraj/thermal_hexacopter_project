# 🚁 Autonomous Thermal-Imaging Hexacopter for Precision Agriculture
**PhD Research Project | Bihar, India**

![Status](https://img.shields.io/badge/Status-In%20Development-yellow)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![PX4](https://img.shields.io/badge/PX4-v1.14-green)
![Bridge](https://img.shields.io/badge/Bridge-Micro%20XRCE--DDS-orange)

## 📖 Project Overview
[cite_start]This research addresses the critical gap in **low-cost precision agriculture** for smallholder farmers (< 2 hectares) in Bihar [cite: 1-3]. [cite_start]We are developing a custom **Hexacopter** equipped with **Thermal Imaging** and **Edge AI (MobileNetV2)** to detect crop diseases 3-5 days earlier than visual inspection[cite: 88, 97].

**Key Objectives:**
1.  [cite_start]**Cost Reduction:** Replace expensive commercial drones (₹6L+) with a custom build (~₹1.5L)[cite: 4, 87].
2.  [cite_start]**Field Accuracy:** Fine-tune MobileNet on the "FieldBihar" dataset to handle humidity and dust[cite: 10, 29].
3.  [cite_start]**Autonomy:** Validated simulation-to-hardware pipeline using ROS2 and Gazebo[cite: 13, 27].

---

## 📚 Documentation
Detailed guides for installation, operation, and debugging can be found in the `docs/` folder:

* **[📖 User Manual](docs/manual.md):** How to install, build, and run the simulation.
* **[🛣️ Roadmap](docs/roadmap.md):** The 8-Phase implementation plan.
* **[🛠️ Troubleshooting](docs/debugging_checkpoints.md):** Fixes for common Docker and ROS2 errors.
* **[📔 Captain's Log](docs/learning_log.md):** Daily engineering decisions and architecture changes.

---

## 🛠️ System Architecture
* **Host OS:** Ubuntu 22.04 LTS
* **Containerization:** Docker (PX4 v1.14 + ROS2 Humble)
* **Middleware:** Micro XRCE-DDS (Client-Agent Architecture)
* **Hardware Target:**
    * [cite_start]**Frame:** Tarot 650 Hexacopter [cite: 15]
    * [cite_start]**Flight Controller:** Pixhawk 4 Mini / Cube Orange [cite: 15]
    * [cite_start]**Onboard Computer:** Raspberry Pi 4 (8GB) [cite: 15]
    * [cite_start]**Sensors:** Seek Thermal CompactPRO + ELP RGB Camera [cite: 15]

---

## 📂 Project Structure
[cite_start](Aligned with PhD Proposal Appendix [cite: 107-137])

```text
workspace/src/agri_hexacopter/
├── config/             # PID gains & Battery params
├── launch/             # ROS2 Launch files (Startup)
├── models/             # SDF Models (Drone & Camera)
├── worlds/             # Gazebo Maps (Bihar Farm)
├── agri_hexacopter/    # Python Source Code (The Brain)
│   ├── __init__.py
│   └── test_connection.py
└── package.xml         # Dependency Definitions

# Step 1: Go to the documentation folder
cd ~/thermal_hexacopter_project/docs

# Step 2: Create the new file
gedit cheatsheet.md



Terminal,Command,What it does
1. Laptop,qgc,Opens your flight dashboard.
2. Laptop,xhost +local:docker,Unlocks the screen for Gazebo.
3. Docker,cd /root/PX4-Autopilot && make px4_sitl gz_x500,Starts the physics and the drone.
4. Docker,MicroXRCEAgent udp4 -p 8888,Connects ROS 2 to the drone.
5. Docker,ros2 run agri_hexacopter real_takeoff,Your code takes control!


Let’s get it running and connected to your Gazebo simulation.

🚀 Step 1: Launch QGroundControl (On Laptop)
Open a terminal on your laptop (the one where you see (base) abhishek@...) and run:

Bash
chmod +x ~/QGroundControl.AppImage
~/QGroundControl.AppImage
Once the QGC window opens, just leave it sitting there. It will look for the drone automatically.

🚁 Step 2: Launch the Drone (In Docker)
Now, go to your Docker terminal (the one showing root@abhishek...) and start the physics engine:

Bash
cd /root/workspace
./visual_hover.sh
📡 How to confirm they are "Talking"
Look at the QGroundControl window while Gazebo is loading. You will know it worked when:

The red "Disconnected" icon changes to a Drone Icon.

A voice says "Manual Control Lost" (this is normal for simulation).

The map centers on a runway (the Gazebo world location).

🏗️ Step 3: The "Full Flight" sequence
Once QGC is connected, you can run your ROS 2 code in a new Docker terminal (using docker exec):

Terminal 2 (Bridge):

Bash
MicroXRCEAgent udp4 -p 8888
Terminal 3 (Flight):

Bash
source /opt/ros/humble/setup.bash
source /root/px4_msgs_ws/install/setup.bash
cd /root/workspace
source install/setup.bash
ros2 run agri_hexacopter real_takeoff

# Check if the file exists in common locations
ls ~/QGroundControl.AppImage
# OR search for it
find ~ -name "QGroundControl.AppImage"

We will open four separate terminals on your laptop.

🖥️ Terminal 1: The Gatekeeper (Host Laptop)
Open a terminal on your laptop. Do not go into Docker yet.

Bash
xhost +local:docker
Wait for the confirmation message.

🖥️ Terminal 2: The World (Gazebo & PX4)
Open a new terminal tab/window and enter your Docker container. This terminal will handle the 3D physics.

Enter Docker:

Bash
docker run -it --privileged --network=host \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 -v ~/thermal_hexacopter_project/workspace:/root/workspace \
 hexacopter_lab_final
Launch Physics:

Bash
cd /root/workspace
./visual_hover.sh
Wait until the Gazebo window with the drone appears and the terminal says Ready for takeoff!

🖥️ Terminal 3: The Bridge (MicroXRCEAgent)
Open a third terminal tab. Since the container is already running, we "jump" into it.

Jump into the running container:

Bash
docker exec -it $(docker ps -q | head -n 1) bash
Start the Bridge:

Bash
MicroXRCEAgent udp4 -p 8888
You should see session established or topic created messages start scrolling.

🖥️ Terminal 4: The Brain (ROS 2 Code)
Open a fourth terminal tab. This is where your actual PhD code lives.

Jump into the container:

Bash
docker exec -it $(docker ps -q | head -n 1) bash
Source the "Language" (The path we found earlier):

Bash
source /opt/ros/humble/setup.bash
source /root/px4_msgs_ws/install/setup.bash
Run your Takeoff Script:

Bash
cd /root/workspace
source install/setup.bash
ros2 run agri_hexacopter real_takeoff
📋 Why this is better for you right now:

🧹 Step 1: Uninstall/Remove Duplicate QGC Files
We will delete the duplicates and keep the one in your Home directory. Run these commands on your laptop terminal (not in Docker):

Bash
# 1. Remove the version in the Tools folder
rm -rf ~/Tools/QGroundControl.AppImage

# 2. If there are extra copies in Home, we will keep just one
# This command ensures only one remains
rm -f ~/QGroundControl.AppImage* ```

Now, let's put a **fresh, clean copy** in your Home folder to be 100% sure it isn't corrupted:
```bash
# Download a fresh stable version into your home directory
cd ~
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.0/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
🚀 Step 2: The "Quick Run" Command
To make your PhD work faster, you shouldn't have to type the whole path every time. We will create a shortcut (alias).

Run this command once:

Bash
echo "alias qgc='~/QGroundControl.AppImage'" >> ~/.bashrc
source ~/.bashrc
Now, you can open QGC simply by typing:

Bash
qgc
