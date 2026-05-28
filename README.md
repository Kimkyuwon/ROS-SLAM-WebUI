# ROS SLAM WEBUI

> **An all-in-one web-based GUI for LiDAR SLAM development with ROS**

ROS SLAM WEBUI brings SLAM, localization, data play/record, configuration, and real-time visualization into a single browser-based interface. Instead of running each ROS launch file and tool manually from separate terminals, you can control the full LiDAR SLAM development workflow comfortably from one web GUI on any device (such as Desktop, Tablet, Smartphone).

<p align="center">
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS2-Jazzy-blue.svg" alt="ROS2 Jazzy" /></a>
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/Python-3.10+-green.svg" alt="Python 3.10+" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-Apache%202.0-brightgreen.svg" alt="License Apache 2.0" /></a>
</p>

![ROS SLAM WEBUI](doc/ROS%20SLAM%20WEBUI.gif)
---

## 🌟 Key Features

### 🗺️ SLAM & Localization

> **Important:** To use these features, clone the linked repositories and build them in your ROS2 workspace first.

- **LiDAR SLAM** is based on [FAST-LIO Mapping](https://github.com/Kimkyuwon/fast_lio2_mapping_and_localization) and [Pose Graph Optimization](https://github.com/Kimkyuwon/Pose_Graph_Optimization)
- **Real-time Localization** is based on [FAST-LIO Localization](https://github.com/Kimkyuwon/fast_lio2_mapping_and_localization)
- **Multi-Session SLAM** is based on [long_term_mapping](https://github.com/Kimkyuwon/long_term_mapping)
- Live YAML configuration editing with instant apply
- Real-time terminal output monitoring
- One-click start/stop control
- Async Save Map 

### 📊 Advanced Data Visualization
- **PlotJuggler-style Real-time Plotting**
  - Interactive time-series data visualization
  - Multi-tab interface for organizing plots
  - Drag-and-drop topic selection from tree view
  - Auto-save/restore plot configurations
  - Zoom, pan, play/pause controls
  - Export plots (PNG)
  - XY Plot support

  ![plot](doc/plot.gif)

- **3D Visualization**
  - Real-time PointCloud2 and Livox CustomMsg visualization
  - Path and odometry display
  - TF tree visualization with Fixed Frame support
  - Interactive camera controls
  - Image topic streaming
  - Snapshot export

  ![plot](doc/visualization.gif)

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
  - Record live ROS topics to bag files in both ROS1 `.bag` and ROS2 `.db3` formats
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
  - **Save Bag** in ROS2 or ROS1 format
  - Playback controls: Loop / Skip stop section / Auto start
  - Timeline slider for frame-accurate seeking

### 🌐 Network Tools
- **Latency Monitor** — real-time HTTP round-trip measurement
  - Sequential 3-ping minimum strategy (avoids self-induced contention)
  - Color-coded indicator: 🟢 Green < 50ms / 🟡 Yellow < 150ms / 🔴 Red ≥ 150ms
  - Measures every 3 seconds

---

## 🚀 Quick Start

### Prerequisites

- **Ubuntu**: 22.04 or later
- **ROS2**: Jazzy or later (Desktop Full recommended — includes `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_msgs`, `rosbag2_py`)
  → [Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
- **Python**: 3.10+ (included with ROS2)
- **rosbridge_server**: required for 3D Viewer and Plot features
- **Browser**: any modern browser (Chrome / Firefox recommended)

### Installation

1. **Install rosbridge_server**
   ```bash
   sudo apt install ros-jazzy-rosbridge-server
   ```

2. **Install Python dependencies**
   ```bash
   pip install rosbags ruamel.yaml numpy opencv-python
   ```
   > `rosbags`: ROS1 bag read/write and format conversion  
   > `opencv-python`: camera image processing in File Player

3. **Clone SLAM-related packages**
   The SLAM & Localization features are based on the following packages:
   ```bash
   cd ~/your_workspace/src
   git clone https://github.com/Kimkyuwon/fast_lio2_mapping_and_localization.git
   git clone https://github.com/Kimkyuwon/Pose_Graph_Optimization.git pose_graph_optimization
   git clone https://github.com/Kimkyuwon/long_term_mapping.git
   ```

4. **Livox LiDAR support**
   Required only if you use a Livox LiDAR sensor. Clone [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2):
   ```bash
   git clone https://github.com/Livox-SDK/livox_ros_driver2.git
   ```

5. **Clone and build ros_slam_webui**
   ```bash
   git clone https://github.com/Kimkyuwon/Web-based-GUI-for-ROS2-SLAM-and-File-Player.git ros_slam_webui
   cd ~/your_workspace
   colcon build
   source install/setup.bash
   ```

### Launch

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source ~/your_workspace/install/setup.bash

# Launch the web server
ros2 launch ros_slam_webui ros_slam_webui.launch.py
```

### Access Web Interface

Once the server starts, you'll see:
```
[INFO] [ros_slam_webui_node]: ======================================
[INFO] [ros_slam_webui_node]: Web server started on port 8080
[INFO] [ros_slam_webui_node]: Local access:   http://localhost:8080
[INFO] [ros_slam_webui_node]: Network access: http://YOUR_IP:8080
[INFO] [ros_slam_webui_node]: ======================================
```

Open the URL in your web browser. For network access from another device (e.g. tablet or smartphone), the device must be connected to the same Wi-Fi network (AP) as the host machine.

---

## 📖 Detailed Feature Guide

### 🗺️ LiDAR SLAM

1. **Load Configuration**
   - Click "Config Load" to browse for your SLAM config file
   - Default location is auto-detected from `FAST_LIO_Localization_and_Mapping/config/mapping_config.yaml` in the same ROS workspace
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
   - Currently published topics are listed in the tree view (topic → message fields)
   - Click a topic to subscribe and expand its message fields
   - **Ctrl+click** to select multiple topics simultaneously

3. **Create Plots**
   - Drag leaf nodes (data fields) from tree to plot area
   - Each drag creates a new trace in the plot
   - Multiple traces can be added to a single plot
   - **XY Plot**: Ctrl+click to select 2 leaf fields → right-click → "Create XY Plot"

4. **Manage Tabs**
   - Click "+" to create new plot tabs
   - Double-click tab title to rename
   - Click "×" to close tabs (minimum 1 tab)
   - Each tab maintains independent plots

5. **Plot Controls**
   - **Play/Pause**: Toggle real-time data updates
   - **t0 Mode**: Show relative time from first data point (enabled by default)
   - **Buffer Time**: Adjust visible time window (1-100 seconds)

6. **Interact with Plots**
   - **Zoom**: Scroll wheel (when paused)
   - **Auto Scale**: Right-click on plot → "Auto Scale"
   - **Delete Plot**: Right-click on trace or legend → "Delete plot"
   - **Clear Plot**: Right-click on plot → "Clear Plot"
   - **Export PNG**: Right-click on plot → "Export as PNG"
   - **Auto-save**: Plot configurations save automatically to browser storage
   - **Auto-restore**: Plots restore after page refresh

7. **Filters** (right-click on a trace → "Apply Filter")
   - **Derivative**: Rate of change
   - **Moving Average**: Smoothing
   - **Moving RMS**: Root mean square
   - **Moving Variance**: Variance over window
   - **Scale + Offset**: Linear transform (y = scale × x + offset)

### 🎥 Bag Recorder

1. **Enter Bag Name**
   - Click "Enter Bag Name" to open a file browser dialog 
   - Type the bag name in the filename field at the bottom
   - Click "Save" to confirm — the "Bag Name" field displays the full path (read-only)

2. **Select Format**
   - Toggle "Save as ROS1 .bag" to record in ROS1 format; leave unchecked for ROS2 `.db3`
   - When checked, the Bag Name field automatically appends `.bag` to the filename

3. **Select Topics**
   - Click "Select Topic" button
   - Choose topics to record from the list (requires bag name to be set first)
   - Click "Confirm"

4. **Record**
   - Click "Record" to start recording
   - A badge shows the active recording format (ROS1 .bag / ROS2 bag)
   - Click "Stop" to finish recording
   - Bag file is saved to the directory selected in step 1

### 🎮 Bag Player

1. **Load Bag**
   - Click "Load Bag File" to open a file browser (starts from your home directory)
   - ROS2 bags (`.db3` directory) and ROS1 `.bag` files are both supported
   - A badge (ROS1 Bag / ROS2 Bag) shows the detected format

2. **Select Topics**
   - Click "Select Topic" to filter which topics to play
   - All topics are played by default if none selected

3. **Playback**
   - Click "Play" to start playback
   - Use timeline slider for seeking
   - Use the speed slider to adjust playback rate (applies to both ROS1 and ROS2 bags)
   - Toggle **Loop** checkbox to replay automatically when finished
   - Click "Stop" to stop playback

4. **Format Conversion**
   - **ROS1 bag loaded**: "Convert to ROS2" button appears — converts offline using `rosbags-convert`
   - **ROS2 bag loaded**: "Convert to ROS1" button appears — converts to `.bag` format
   - Direct playback without conversion is available for both formats

### 📂 File Player

The File Player supports direct playback of four dataset formats without conversion.

1. **Select Dataset Format**
   - Use the "Dataset" dropdown to choose: ConPR / KITTI Raw / KAIST Complex Urban / MulRan

2. **Load Directory**
   - Click "Load" to open a file browser (starts from your home directory)
   - Navigate to and select the dataset root directory
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

#### Layout

The 3D Viewer is divided into three panels:
- **Displays** (left): manage active topic subscriptions and Fixed Frame
- **3D View** (center): Three.js canvas
- **Views** (right): camera type and view controls

Each side panel can be collapsed with the **▶** button.

#### 1. Add Display

Click **"+ Add"** in the Displays panel to open the Add Display dialog.  
Select a display type and choose a topic to start visualizing:

| Display Type | ROS Message Type | Description |
|---|---|---|
| **PointCloud2** | `sensor_msgs/PointCloud2` | Colored point cloud |
| **Path** | `nav_msgs/Path` | Connected line segments |
| **Odometry** | `nav_msgs/Odometry` | Arrow + optional trajectory trail |
| **TF** | `tf2_msgs/TFMessage` | Coordinate frame axes |
| **LivoxLidar** | `livox_ros_driver2/CustomMsg` | Livox custom point cloud (By Line / Tag Filter) |
| **Image** | `sensor_msgs/Image` | Live video panels below the 3D view |
| **LaserScan** | `sensor_msgs/LaserScan` | 2D scan points in 3D space |

#### 2. Display Settings

Click a display item in the Displays panel to expand its settings:

- **PointCloud2 / LivoxLidar**
  - **Color Mode**: `Rainbow` (field value → gradient), `By Line` (Livox only), `Solid` (single color), `RGB`
  - **Color Field**: `intensity`, `x`, `y`, `z`, `reflectivity`, …
  - **Point Size**: point rendering size
  - **Decay Time**: seconds before old points are removed (0 = keep latest frame only)
- **Odometry**
  - **Trajectory**: toggle trail on/off, set max trail length
- **TF**
  - Show/hide individual coordinate frames

#### 3. Fixed Frame

- Type or select the reference TF frame in the **Fixed Frame** combo box (default: `map`)
- Click **▾** to show available TF frames detected at runtime
- TF transformations are applied automatically to all displays

#### 4. Camera Controls (3D View)

| Action | Control |
|---|---|
| Rotate | Left mouse drag |
| Zoom | Mouse scroll wheel |
| Pan | Right mouse drag |

#### 5. Views Panel

- **Type**: `Orbit` (free rotation around center) or `TopDown` (bird's-eye view)
- **Zero**: reset camera to default position

#### 6. Image Viewer

- Select an `Image` topic via **+ Add → Image**
- Image panels appear below the 3D canvas; multiple topics can be displayed simultaneously
- Frames stream via binary WebSocket (port 8081) with GPU-accelerated JPEG decoding
- Resize the image panel by dragging the separator bar

#### 7. Snapshot & Reset

- **Snapshot**: saves the current 3D view as a PNG file
- **Reset**: removes all active displays and clears the scene

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

Edit `ros_slam_webui/web_server.py`:
```python
# Change port number (default: 8080)
web_thread = threading.Thread(target=run_web_server, args=(node, 9090), daemon=True)
```

Then rebuild:
```bash
colcon build --packages-select ros_slam_webui
```

---

## 🏗️ Project Structure

```
ros_slam_webui/
├── ros_slam_webui/
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
│   └── ros_slam_webui.launch.py  # ROS2 launch configuration
├── package.xml                    # ROS2 package manifest
├── setup.py                       # Python package setup
├── README.md                      # This file
└── LICENSE                        # Apache 2.0 License
```

---

## 📚 Related Projects

### SLAM & Localization
- [FAST-LIO Mapping & Localization](https://github.com/Kimkyuwon/fast_lio2_mapping_and_localization) - ROS2 FAST-LIO mapping/localization package used by this Web UI
- [Pose Graph Optimization](https://github.com/Kimkyuwon/Pose_Graph_Optimization)
- [long_term_mapping](https://github.com/Kimkyuwon/long_term_mapping)

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

### Third-Party Notices

The following libraries are loaded at runtime and are **not** bundled in this repository:

| Library | Version | License | Usage |
|---|---|---|---|
| [Three.js](https://github.com/mrdoob/three.js) | 0.128.0 | MIT | 3D rendering |
| [Plotly.js](https://github.com/plotly/plotly.js) | 2.27.0 | MIT | Interactive plots |
| [roslibjs](https://github.com/RobotWebTools/roslibjs) | 1.1.0 | BSD-3-Clause | ROS WebSocket bridge |
| [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) | — | BSD-3-Clause | ROS WebSocket server |

The following Python packages are installed separately as runtime dependencies:

| Package | License |
|---|---|
| [rosbags](https://gitlab.com/ternaris/rosbags) | Apache 2.0 |
| [ruamel.yaml](https://sourceforge.net/p/ruamel-yaml) | MIT |
| [numpy](https://numpy.org) | BSD |
| [opencv-python](https://github.com/opencv/opencv-python) | MIT / Apache 2.0 |

### Design Inspiration

- **[PlotJuggler](https://github.com/facontidavide/PlotJuggler)** (LGPL v3.0) — The `PlotJugglerTree` component is an **independent JavaScript implementation** inspired by PlotJuggler's tree-view UI concept. No source code from PlotJuggler is copied or derived. LGPL does not apply to independently re-implemented works.
- **KITTI / KAIST / MulRan / ConPR** — The File Player implements **file format parsers** for these dataset formats. The dataset files themselves are not included or redistributed; their respective dataset licenses apply only to the data.

---

**Made with ❤️ for the ROS2 community**
