# ROS2 AUTONAV WEBUI

> **A modern, browser-based control center for autonomous robot navigation with ROS2**

Web-based graphical user interface providing intuitive control and real-time visualization of SLAM, localization, navigation, and data management for ROS2-based autonomous robots. Access everything from any device with a web browser.

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-brightgreen.svg)](LICENSE)

---

## 🌟 Key Features

### 🗺️ SLAM & Localization
- **LiDAR SLAM** with FAST-LIO integration
- **Real-time Localization** with live parameter tuning
- **Multi-Session SLAM** for map merging and optimization
  - Async optimization with real-time log streaming
  - Cancel button during running optimization
  - Progress/status indicator with spinner
- Live YAML configuration editing with instant apply
- Real-time terminal output monitoring
- One-click start/stop control
- Async Save Map with cancel support

### 📊 Advanced Data Visualization
- **PlotJuggler-style Real-time Plotting**
  - Interactive time-series data visualization
  - Multi-tab interface for organizing plots
  - Drag-and-drop topic selection from tree view
  - Auto-save/restore plot configurations
  - Zoom, pan, play/pause controls
  - Export plots (PNG)
  - XY Plot support
  - Filter (Low-pass, High-pass, Band-pass)

- **3D Visualization**
  - Real-time PointCloud2 rendering (Ouster, Velodyne, Hesai, Livox)
  - Path and odometry display
  - TF tree visualization with Fixed Frame support
  - Interactive camera controls
  - Python backend binary WebSocket for low-latency updates (port 8081)
  - Point Size / Alpha / Decay Time settings
  - **Image topic streaming** via binary WebSocket Web Worker (`img_stream_worker.js`)
    - GPU-accelerated JPEG decoding via `createImageBitmap()`
    - Subscribe/unsubscribe image topics live
  - Snapshot export

### 💾 Data Management
- **Bag Player**
  - Play ROS2 bag files with topic filtering
  - Timeline control with play/pause/seek
  - Variable playback speed (slider)
  - **Loop mode** for continuous replay
  - Integration with 3D visualization and Plot
  - **ROS1 bag support**: Auto-detect `.bag` files, offline convert to ROS2, or direct real-time playback via `rosbags`
  - Convert ROS2 bag → ROS1 `.bag` format

- **Bag Recorder**
  - Record live ROS2 topics to bag files
  - Selective topic recording
  - Real-time recording status
  - Auto-saved configurations

- **File Player** — Multi-dataset direct playback
  - Supports 4 dataset formats selectable via dropdown:
    - **ConPR**: CSV-based trajectory/LiDAR/camera data
    - **KITTI Raw**: Velodyne HDL-64E, 4 cameras (color/gray), IMU/GPS (OXTS), TF
    - **KAIST Complex Urban**: VLP-16 (left/right), SICK LiDAR (back/mid), stereo camera, IMU, GPS, VRS
    - **MulRan**: Ouster OS1-64 LiDAR, radar polar image, IMU, GPS
  - **Drive/Sequence selection** per dataset (scanned from directory)
  - **Save Bag** in ROS2 or ROS1 format with progress bar
  - Playback controls: Loop / Skip stop section / Auto start
  - Timeline slider for frame-accurate seeking
  - Background worker threads for heavy I/O (non-blocking sensor publish)
  - Low-latency design: 3ms CPU yield per worker frame, batch limit 8 frames, 100 Hz timer

### 🌐 Network Tools
- **Latency Monitor** — real-time HTTP round-trip measurement
  - Sequential 3-ping minimum strategy (avoids self-induced contention)
  - Color-coded indicator: 🟢 Green < 50ms / 🟡 Yellow < 150ms / 🔴 Red ≥ 150ms
  - Measures every 3 seconds

---

## 🚀 Quick Start

### Prerequisites

- **Ubuntu**: 22.04 or later
- **ROS2**: Jazzy or later ([Installation Guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Python**: 3.10+ (included with ROS2)
- **rosbridge_server**: For 3D visualization and plot features

### Installation

1. **Install rosbridge_server**
   ```bash
   sudo apt install ros-jazzy-rosbridge-server
   ```

2. **Clone the repository**
   ```bash
   cd ~/your_workspace/src
   git clone https://github.com/Kimkyuwon/Web-based-GUI-for-ROS2-SLAM-and-File-Player.git ros2_autonav_webui
   ```

3. **Build the package**
   ```bash
   cd ~/your_workspace
   colcon build --packages-select ros2_autonav_webui
   source install/setup.bash
   ```

### Launch

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source ~/your_workspace/install/setup.bash

# Launch the web server
ros2 launch ros2_autonav_webui ros2_autonav_webui.launch.py
```

### Access Web Interface

Once the server starts, you'll see:
```
[INFO] [ros2_autonav_webui_node]: ======================================
[INFO] [ros2_autonav_webui_node]: Web server started on port 8080
[INFO] [ros2_autonav_webui_node]: Local access:   http://localhost:8080
[INFO] [ros2_autonav_webui_node]: Network access: http://YOUR_IP:8080
[INFO] [ros2_autonav_webui_node]: ======================================
```

Open the URL in your web browser.

---

## 📖 Detailed Feature Guide

### 🗺️ LiDAR SLAM

1. **Load Configuration**
   - Click "Config Load" to browse for your SLAM config file
   - Default location: `/path/to/FAST_LIO_ROS2/config/mapping_config.yaml`
   - Edit parameters in real-time using the web interface

2. **Start SLAM**
   - Click "Start SLAM" button
   - Monitor real-time terminal output in the web UI
   - Green status indicator shows running state

3. **Save Map**
   - Click "Save Map" to trigger pose graph optimization
   - Async operation — progress status shown while saving
   - Click "Cancel Save Map" to abort if needed
   - Map is saved to configured output directory

4. **Stop SLAM**
   - Click "Stop SLAM" button
   - Process terminates gracefully (SIGINT → SIGTERM → SIGKILL)

### 🔗 Multi-Session SLAM

1. **Set Inputs**
   - Click "Set Map 1" and "Set Map 2" to select two map directories
   - Click "Set Output" to set output path

2. **Run Optimization**
   - Click "Multi Session Optimization"
   - Real-time log stream appears in the status area with spinner
   - Click "Cancel" to stop at any time

3. **Completion**
   - Status banner turns green on success, red on failure
   - Button label changes to "Exit" after completion — click to reset UI

### 📊 Real-time Plotting

The Plot feature provides PlotJuggler-style visualization directly in your browser:

1. **Navigate to Plot Tab**
   - Click "Visualization" → "Plot" in the main navigation

2. **Browse Topics**
   - Topics are displayed in a tree view (topic → message fields)
   - Search topics by name using the search box
   - Click to expand/collapse message fields

3. **Create Plots**
   - Drag leaf nodes (data fields) from tree to plot area
   - Each drag creates a new trace in the plot
   - Multiple traces can be added to a single plot

4. **Manage Tabs**
   - Click "+" to create new plot tabs
   - Double-click tab title to rename
   - Click "×" to close tabs (minimum 1 tab)
   - Each tab maintains independent plots

5. **Plot Controls**
   - **Play/Pause**: Toggle real-time data updates
   - **t0 Mode**: Show relative time (starts from 0 seconds)
   - **Zoom Out**: Auto-scale to fit all data
   - **Clear Plot**: Reset plot (keeps current subscriptions)
   - **Buffer Time**: Adjust visible time window (1-300 seconds)

6. **Interact with Plots**
   - **Zoom**: Scroll wheel (when paused)
   - **Pan**: Middle mouse button drag (when paused)
   - **Delete Plot**: Right-click on trace or legend → "Delete plot"
   - **Auto-save**: Plot configurations save automatically
   - **Auto-restore**: Plots restore after page refresh

### 🎥 Bag Recorder

1. **Enter Bag Name**
   - Type desired bag name in "Bag Name" field
   - Click "Enter Bag Name" to confirm

2. **Select Topics**
   - Click "Select Topic" button
   - Choose topics to record from the list
   - Click "Confirm"

3. **Record**
   - Click "Record" to start recording
   - Status indicator shows recording state
   - Click "Stop" to finish recording
   - Bag file is saved to `/home/user/dataset/[bag_name]/`

### 🎮 Bag Player

1. **Load Bag**
   - Click "Load Bag File" to browse for bag directory
   - ROS2 bags are stored as directories; ROS1 `.bag` files are also supported

2. **Select Topics**
   - Click "Select Topic" to filter which topics to play
   - All topics are played by default if none selected

3. **Playback**
   - Click "Play" to start playback
   - Use timeline slider for seeking
   - Use the speed slider to adjust playback rate
   - Toggle **Loop** checkbox to replay automatically when finished
   - Click "Stop" to stop playback

4. **ROS1 Bag Support**
   - `.bag` files are auto-detected as ROS1 format ("ROS1 Bag" badge shown)
   - Click "Convert to ROS2" for offline conversion using `rosbags-convert`
   - Click "Play" to stream directly as ROS2 messages without conversion
   - Click "Convert to ROS1" to convert a loaded ROS2 bag back to `.bag` format

### 📂 File Player

The File Player supports direct playback of four dataset formats without conversion.

1. **Select Dataset Format**
   - Use the "Dataset" dropdown to choose: ConPR / KITTI Raw / KAIST Complex Urban / MulRan

2. **Load Directory**
   - Click "Load" and browse to the dataset root directory
   - For KITTI: select the base directory containing drive folders (`2011_09_26_drive_*`)
   - For KAIST: select the directory containing sequence folders (`urban00`, `urban01`, …)
   - For MulRan: select the directory containing sequence folders (`Riverside01`, `KAIST01`, …)

3. **Select Drive/Sequence** *(KITTI / KAIST / MulRan only)*
   - A dropdown is populated with detected drives or sequences
   - Select the desired entry to load it

4. **Play**
   - Click "▶ Play" to start publishing sensor data to ROS2 topics
   - Click "⏸ Pause" to pause; click again to resume
   - Use the **timeline slider** to seek to any position

5. **Playback Options**
   | Option | Description |
   |---|---|
   | **Loop** | Restart from beginning when playback reaches the end |
   | **Skip stop section** | Skip long gaps between sensor events (on by default) |
   | **Auto start** | Begin playback automatically after loading |

6. **Save Bag**
   - Choose output format (ROS2 or ROS1) from the dropdown next to the "Save Bag" button
   - Click "Save Bag" to convert the current dataset to a bag file
   - Progress bar shows conversion progress

#### Published Topics by Dataset

| Dataset | Topics |
|---|---|
| **ConPR** | `/livox/lidar`, `/camera/image_raw`, `/imu/data`, `/gps/fix`, `/pose` |
| **KITTI Raw** | `/kitti/velo/pointcloud`, `/kitti/camera_*/image_raw`, `/kitti/imu`, `/kitti/gps/fix`, `/tf` |
| **KAIST Complex Urban** | `/velodyne_left/points`, `/velodyne_right/points`, `/sick_back/points`, `/sick_mid/points`, `/stereo/left/image_raw`, `/imu/data`, `/gps/fix`, `/vrs_gps/fix`, `/tf` |
| **MulRan** | `/os1_points`, `/radar/polar`, `/imu/data_raw`, `/gps/fix`, `/tf` |

### 🌐 3D Visualization

1. **Select Display Topics**
   - Click "Select PointCloud2 Topics" button
   - Choose PointCloud2, Path, Odometry, or Image topics
   - Click "Confirm" to start visualization

2. **Camera Controls**
   - **Rotate**: Left mouse drag
   - **Zoom**: Mouse scroll wheel
   - **Pan**: Right mouse drag

3. **Fixed Frame**
   - Select the reference coordinate frame from the dropdown
   - TF transformations are applied automatically

4. **Image Viewer**
   - Select an `Image` topic in the topic selection dialog
   - Image frames stream via binary WebSocket (port 8081) with GPU-accelerated JPEG decoding

---

## 🔧 Configuration

### ROS2 Environment Variables

The service automatically sets:
```bash
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
```

For manual ROS2 commands in other terminals, set the same variables:
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
```

### Network Access Configuration

**Ubuntu/Linux:**
```bash
sudo ufw allow 8080/tcp
sudo ufw reload
```

**Windows:**
- Windows Defender Firewall → Advanced Settings → Inbound Rules
- New Rule → Port → TCP 8080 → Allow

### Change Server Port

Edit `ros2_autonav_webui/web_server.py`:
```python
# Change port number (default: 8080)
web_thread = threading.Thread(target=run_web_server, args=(node, 9090), daemon=True)
```

Then rebuild:
```bash
colcon build --packages-select ros2_autonav_webui
```

---

## 📦 Dependencies

### Required

- **ROS2 Jazzy** (Desktop Full)
  - `rclpy` - ROS2 Python library
  - `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_msgs` - Standard message types
  - `rosbag2_py` - Bag file handling

- **rosbridge_server** - WebSocket bridge for browser communication
  ```bash
  sudo apt install ros-jazzy-rosbridge-server
  ```

### Optional

**FAST-LIO** (for SLAM/Localization features):
```bash
cd ~/your_workspace/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ~/your_workspace
colcon build --packages-select fast_lio
```

**livox_ros_driver2** (for Livox LiDAR data):
```bash
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/your_workspace
colcon build --packages-select livox_ros_driver2
```

**rosbags** (for ROS1 bag support):
```bash
pip install rosbags
```

**opencv-python** (for camera image processing in File Player):
```bash
pip install opencv-python
```

---

## 🏗️ Project Structure

```
ros2_autonav_webui/
├── ros2_autonav_webui/
│   ├── web_server.py              # Main HTTP server, ROS2 node & binary WebSocket (port 8081)
│   ├── kitti_converter.py         # KITTI Raw dataset → ROS2 message converter
│   ├── kaist_converter.py         # KAIST Complex Urban dataset → ROS2 message converter
│   ├── mulran_converter.py        # MulRan dataset → ROS2 message converter
│   └── __init__.py
├── web/
│   ├── index.html                 # Main web interface
│   └── static/
│       ├── script.js              # Main UI logic & API calls
│       ├── plot_manager.js        # Plotly.js plot management
│       ├── plot_tab_manager.js    # Plot tab management
│       ├── plot_tree.js           # PlotJuggler-style tree view
│       ├── threejs_display.js     # Three.js 3D visualization
│       ├── pc2_stream_worker.js   # Web Worker for binary PointCloud2 streaming
│       ├── img_stream_worker.js   # Web Worker for binary Image streaming (JPEG/GPU decode)
│       └── style.css              # UI styling
├── launch/
│   └── ros2_autonav_webui.launch.py  # ROS2 launch configuration
├── package.xml                    # ROS2 package manifest
├── setup.py                       # Python package setup
├── README.md                      # This file
└── LICENSE                        # Apache 2.0 License
```

---

## 🛠️ Development

### Development Workflow

1. **For Python backend changes:**
   ```bash
   # Edit files in ros2_autonav_webui/
   colcon build --packages-select ros2_autonav_webui
   source install/setup.bash
   # Restart the server
   ```

2. **For web frontend changes:**
   ```bash
   # Edit files in web/static/
   colcon build --packages-select ros2_autonav_webui
   # Hard refresh browser (Ctrl + F5)
   ```

3. **For faster iteration (one-time setup):**
   ```bash
   colcon build --symlink-install --packages-select ros2_autonav_webui
   # Now Python and web file changes don't require rebuild
   ```

### Code Organization

**Backend (Python):**
- Add API endpoints in `ros2_autonav_webui/web_server.py`
- Use `do_GET()` for GET requests, `do_POST()` for POST requests
- Follow existing patterns for error handling and responses
- Dataset converters (`kitti_converter.py`, `kaist_converter.py`, `mulran_converter.py`) handle all format-specific parsing

**Frontend (JavaScript):**
- Main UI logic: `web/static/script.js`
- Plot management: `web/static/plot_manager.js`
- Tab management: `web/static/plot_tab_manager.js`
- Tree view: `web/static/plot_tree.js`
- Use `apiCall(endpoint, data)` for backend communication

**UI (HTML/CSS):**
- Layout: `web/index.html`
- Styling: `web/static/style.css`
- Follow existing CSS variable conventions for theming

---

## 🐛 Troubleshooting

### Web Server Won't Start

**Issue**: Port 8080 already in use
```bash
# Check what's using port 8080
sudo lsof -i :8080
# Kill the process or change port in web_server.py
```

### rosbridge Connection Failed

**Issue**: Plot or 3D viewer not working
```bash
# Ensure rosbridge is installed
sudo apt install ros-jazzy-rosbridge-server

# Check if rosbridge is running (it starts automatically with the launch file)
ros2 node list | grep rosbridge
```

### Topics Not Appearing

**Issue**: No topics in plot tree or bag recorder
```bash
# Check ROS2 environment variables
echo $ROS_DOMAIN_ID      # Should be 0
echo $ROS_LOCALHOST_ONLY # Should be 1

# List topics manually
ros2 topic list
```

### Page Refresh Loses Data

**Issue**: Plot configurations not restoring
- Check browser console (F12) for JavaScript errors
- Clear browser cache and LocalStorage if corrupted:
  ```javascript
  // In browser console
  localStorage.clear();
  location.reload();
  ```

### File Player Latency Spikes

**Issue**: `latency` indicator turns red during dataset playback
- Ensure the server is rebuilt and restarted after the latest update
- KITTI/MulRan use background worker threads; first playback may take 1-2 s to warm up
- Reducing the number of active rviz2 subscribers lowers DDS load

### Build Errors

**Issue**: `colcon build` fails
```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild from scratch
colcon build --packages-select ros2_autonav_webui
```

---

## 📚 Related Projects

### SLAM & Localization
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO) - Fast LiDAR-Inertial Odometry
- [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) - ROS2 port of FAST-LIO

### Dataset & Tools
- [ConPR](https://github.com/dongjae0107/ConPR) - ConPR dataset format
- [KITTI](https://www.cvlibs.net/datasets/kitti/) - KITTI Raw dataset
- [KAIST Complex Urban](https://irap.kaist.ac.kr/dataset/) - KAIST Urban dataset
- [MulRan](https://sites.google.com/view/mulran-pr) - MulRan dataset
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler) - Inspiration for plot UI

### Visualization
- [Three.js](https://threejs.org/) - 3D graphics library
- [Plotly.js](https://plotly.com/javascript/) - Interactive plotting library
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) - WebSocket interface to ROS

---

## 📄 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

### Third-Party Licenses
- **Three.js**: MIT License
- **Plotly.js**: MIT License
- **rosbridge_suite**: BSD License

---

## 🙏 Acknowledgments

Special thanks to:
- **FAST-LIO developers** for robust SLAM algorithms
- **ConPR team** for the dataset format
- **PlotJuggler** for UI inspiration
- **ROS2 community** for excellent documentation
- All contributors and users of this project

---

**Made with ❤️ for the ROS2 community**
