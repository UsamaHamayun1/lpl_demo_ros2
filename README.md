## 🛠️ Prerequisites

Make sure you have the following installed on your system:
* **ubuntu 22 LTS
* **ROS 2** (Humble)
* **Gazebo** (Classic)
* **TurtleBot3 ROS 2 Packages**
* **Python 3 Dependencies**:
  ```bash
  pip install PyQt5 opencv-python numpy
  ```

---

## 🏗️ Build Instructions

1. create a folder and then create a ros ws  (lpl_ws) then clone the this repo into src folder of ws
   ```bash
   cd ~/lpl_ws/src
   ```
2. Build the package using `colcon`:
   ```bash
   colcon build --packages-select lpl_demo
   ```
3. Source your workspace to make the built files accessible:
   ```bash
   source install/setup.bash
   ```
   *(Note: You will need to run `source install/setup.bash` in every new terminal you open, unless you add it to your `~/.bashrc`)*

---

## 🏃‍♂️ How to Run the Demo

To run the full simulation, you will need to open a few separate terminals. **Remember to source your workspace in each one!**

### 1. Launch the Simulation (Gazebo)
Start the Gazebo world with the TurtleBot3 and obstacles.
```bash
ros2 launch lpl_demo gauntlet.launch.py
```
*(You can swap `gauntlet.launch.py` with `actor_puppet.launch.py` or any of the other worlds in the `launch/` folder).*

### 2. Launch the Navigation Stack
Start the ROS 2 Nav2 stack to handle path planning.
```bash
ros2 launch lpl_demo lpl_nav2.launch.py
```

### 3. Run the LPL Guard (The AI Brain)
This node calculates the confidence sliding scale, assesses threats, and controls the velocity commands.
```bash
ros2 run lpl_demo lpl_manager_guard
```

### 4. Open the Mission Control Dashboard
Launch the PyQt5 UI to monitor the robot and provide manual overrides.
```bash
ros2 run lpl_demo lpl_dashboard
```

---
4. **Drive:** Click on the Dashboard window and use the **W, A, S, D** keys to manually drive the robot safely past the obstacle.
5. **Authority Recovery:** Once clear of the obstacle, press **SPACEBAR** to hand control back to the AI.
6. **Confidence Recovery:** The robot will remain physically stopped while the AI "rebuilds" its model. Watch the UI bar fill from 0% back to 100%. Once full, the robot resumes autonomous driving!
