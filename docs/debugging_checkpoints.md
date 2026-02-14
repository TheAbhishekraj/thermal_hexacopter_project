---

### 3. Debugging Checkpoints (`debugging_checkpoints.md`)
**Command:** `gedit debugging_checkpoints.md`
**Context:** Use this checklist when things crash. It isolates whether the problem is the Code, the Bridge, or the Simulator.

```markdown
# 🛠️ Debugging Checkpoints

## 🛑 Checkpoint 1: The Docker Build
* **Command:** `docker images`
* **Success Criteria:** You see `thermal-hexacopter` with tag `v1.14` listed.
* **Failure Fix:** Check internet connection and re-run `docker build`.

## 🛑 Checkpoint 2: Graphics & Simulator
* **Command:** `launch_simulation`
* **Success Criteria:** A Gazebo window pops up with a drone on grass.
* **Failure Fix:**
    * Error: "No protocol specified"? -> Run `xhost +local:docker` on host.
    * Black screen? -> Check NVIDIA drivers or use software rendering.

## 🛑 Checkpoint 3: The Bridge (Micro XRCE-DDS)
* **Command:** `start_bridge`
* **Success Criteria:**
    * Output says: `[12345] Agent listening on port 8888`
    * When simulation starts, it says: `[12345] New client: 0x01`
* **Failure Fix:** Ensure both terminals are in the same network (Docker `--network=host` handles this).

## 🛑 Checkpoint 4: ROS2 Topics
* **Command:** `ros2 topic list`
* **Success Criteria:** You see topics like `/fmu/out/vehicle_status` or `/fmu/in/trajectory_setpoint`.
* **Failure Fix:** If list is empty, the Bridge (Checkpoint 3) is not running.

## 🛑 Checkpoint 5: Python Script Execution
* **Command:** `ros2 run agri_hexacopter test_connection`
* **Success Criteria:** Script prints "✅ CONNECTION VERIFIED!"
* **Failure Fix:**
    * "Package not found"? -> Run `source install/setup.bash`.
    * "Executable not found"? -> Check `setup.py` entry_points.
