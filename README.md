# Pothole Prediction base model version-1 (avoidance is not included)

Building an autonomous system with pothole prediction to reduce the impact of potholes and manage terrain roads for an ADAS-powered L2 car.

Testing for CROBOT (wheeled Manipulator) using Intel RealSense D435 Depth Camera has been completed and results are stored in the provided Drive Files.
Link: https://drive.google.com/drive/folders/1t5VTZ6-SJFxXP-aronKnmvw01V4GU6BO

---

## Table of Contents
- [Project Overview](#project-overview)
- [Project Structure](#project-structure)
- [Package Descriptions](#package-descriptions)
- [Unity Integration with ROS-TCP-Endpoint](#unity-integration-with-ros-tcp-endpoint)
- [Quick Start Guide](#quick-start-guide)

---

## Project Overview

This project implements an autonomous vehicle safety system using ROS2 with the following core components:

1. **Lane Detection** - Computer vision-based lane tracking
2. **Obstacle Detection** - YOLO-based real-time obstacle recognition
3. **Pothole Detection** - Depth-aware pothole identification and avoidance
4. **ROS-TCP Communication** - Bridge between ROS2 and Unity simulation environment

---

## Project Structure

```
Pothole_Prediction_and_Avoidance_Strategy_for_L2_Car/
│
├── README.md                          # Project documentation and setup guide
│
├── lane_detection_cv/                 # Lane Detection Module
│   ├── package.xml                    # ROS2 package metadata
│   ├── setup.py                       # Python package setup
│   ├── setup.cfg                      # Package configuration
│   │
│   └── lane_detection_cv/             # Main package source code
│       ├── __init__.py                # Package initializer
│       ├── lane_detection_cv.py       # Primary lane detection logic
│       ├── lane_detector.py           # Lane detector class implementation
│       └── unity_lane_detection_node.py  # ROS2 node for Unity integration
│
├── obstacle_detection/                # Obstacle Detection Module  
│   ├── package.xml                    # ROS2 package metadata
│   ├── setup.py                       # Python package setup
│   ├── setup.cfg                      # Package configuration
│   │
│   └── obstacle_detection/            # Main package source code
│       ├── __init__.py                # Package initializer
│       ├── yolvo_obstacle_detection.py # YOLO-based obstacle detection
│       ├── autonomous_safety.py       # Safety fusion and decision logic
│       ├── lane_guide_node.py         # Lane guidance decision node
│       ├── node.py                    # Base node implementation
│       ├── path_guard_node.py         # Path safety validation node
│       ├── safety_fusion_node.py      # Multi-sensor safety fusion
│       ├── safety_mux_node.py         # Safety signal multiplexer
│       └── unity_yolo_obstacle_node.py # ROS2 node for Unity integration
│
├── pothole_detection/                 # Pothole Detection Module
│   ├── package.xml                    # ROS2 package metadata
│   ├── setup.py                       # Python package setup
│   ├── setup.cfg                      # Package configuration
│   │
│   └── pothole_detection/             # Main package source code
│       ├── __init__.py                # Package initializer
│       ├── yolo_pothole_detection.py  # YOLO model for pothole detection
│       ├── yolo_pothole_depth.py      # Depth integration for pothole mapping
│       ├── point_cloud_visualizer.py  # 3D point cloud visualization
│       └── unity_pothole_detection_node.py # ROS2 node for Unity integration
│
└── ROS-TCP-Endpoint/                  # ROS2-Unity Communication Bridge
    ├── package.xml                    # ROS2 package metadata
    ├── setup.py                       # Python package setup
    ├── setup.cfg                      # Package configuration
    ├── requirements.txt               # Python dependencies
    ├── LICENSE                        # Package license
    ├── README.md                      # ROS-TCP-Endpoint documentation
    ├── CHANGELOG.md                   # Version history
    │
    ├── launch/
    │   └── endpoint.py                # ROS2 launch file for TCP endpoint
    │
    └── ros_tcp_endpoint/              # Core communication package
        ├── __init__.py                # Package initializer
        ├── client.py                  # TCP client for communication
        ├── server.py                  # TCP server implementation
        ├── communication.py           # Core communication protocol
        ├── publisher.py               # ROS2 publisher wrapper
        ├── subscriber.py              # ROS2 subscriber wrapper
        ├── service.py                 # ROS2 service wrapper
        ├── tcp_sender.py              # TCP message sender
        ├── thread_pauser.py           # Thread management utilities
        ├── default_server_endpoint.py # Default server configuration
        ├── unity_service.py           # Unity-specific service layer
        └── exceptions.py              # Custom exception definitions
```

---

## Package Descriptions

### 1. **lane_detection_cv** - Lane Detection Module

**Purpose:** Detects and tracks road lane markings using computer vision techniques.

**Key Components:**
- `lane_detection_cv.py` - Main detection algorithm implementation
- `lane_detector.py` - Lane detector class with image processing pipeline
- `unity_lane_detection_node.py` - ROS2 node that publishes lane data to Unity via TCP-Endpoint

**Usecase:**
- Provides lane position information for vehicle steering control
- Enables lane-keeping assist (LKA) functionality
- Serves as reference for safe navigation boundaries

**Output:** Lane position data (left/right lane coordinates, curvature)

---

### 2. **obstacle_detection** - Obstacle Detection Module

**Purpose:** Real-time detection of static and dynamic obstacles using YOLO deep learning.

**Key Components:**
- `yolvo_obstacle_detection.py` - YOLO model inference for obstacle detection
- `autonomous_safety.py` - Safety decision logic based on detected obstacles
- `safety_fusion_node.py` - Fuses multiple sensor inputs for safety decisions
- `path_guard_node.py` - Validates safe navigation paths
- `lane_guide_node.py` - Provides lane-based guidance for obstacle avoidance
- `safety_mux_node.py` - Multiplexes safety signals from different sources
- `unity_yolo_obstacle_node.py` - ROS2 node for real-time obstacle visualization in Unity

**Usecase:**
- Detects vehicles, pedestrians, and static obstacles
- Triggers collision avoidance maneuvers
- Provides visual feedback through Unity simulation
- Enables autonomous decision-making for safe path planning

**Output:** Obstacle bounding boxes, distance, and classification data

---

### 3. **pothole_detection** - Pothole Detection & Avoidance Module

**Purpose:** Identifies potholes and road irregularities using YOLO and depth information.

**Key Components:**
- `yolo_pothole_detection.py` - YOLO model for pothole classification
- `yolo_pothole_depth.py` - Depth-based pothole severity assessment
- `point_cloud_visualizer.py` - 3D visualization of road surface
- `unity_pothole_detection_node.py` - ROS2 node for pothole data publication

**Usecase:**
- Detects road damage and potholes ahead of the vehicle
- Calculates pothole depth and severity using depth sensor data
- Predicts optimal avoidance trajectories
- Provides 3D visualization of road surface condition
- Enables autonomous suspension control or lane adjustment

**Output:** Pothole location, depth, severity level, and avoidance recommendations

---

### 4. **ROS-TCP-Endpoint** - ROS2-Unity Communication Bridge

**Purpose:** Provides a TCP communication protocol to bridge ROS2 nodes with Unity simulation environment.

**Key Components:**
- `server.py` - TCP server listening for Unity connections
- `client.py` - TCP client for message exchange
- `communication.py` - Core protocol implementation
- `publisher.py`, `subscriber.py`, `service.py` - ROS2 middleware wrappers
- `unity_service.py` - Unity-specific service implementations
- `tcp_sender.py` - Optimized TCP message transmission

**Usecase:**
- Enables real-time data exchange between ROS2 system and Unity simulation
- Allows sending camera feeds, sensor data, and control commands
- Supports bi-directional communication for closed-loop simulation
- Facilitates integration testing in simulated environment before real-world deployment

**Communication Protocol:** TCP socket-based message passing with serialization

---

## Unity Integration with ROS-TCP-Endpoint

### Camera Feed Integration for Simulation View

To operate the system with Unity simulation environment and receive real-time camera feeds:

#### **Prerequisites:**
1. ROS2 properly configured with this package
2. Unity 2021 LTS or later with ROS2 for Unity plugin installed
3. Network connectivity between ROS2 machine and Unity simulation machine

#### **Setup Steps:**

**1. Start ROS-TCP-Endpoint Server:**
```bash
ros2 launch ros_tcp_endpoint endpoint.py
```

**2. Configure Unity TCP Client:**
- In Unity, create/configure the ROS TCP Connector to connect to the ROS2 server
- Set the server address (IP where ROS2 is running) and port (default: 10000)

**3. Camera Feed Integration:**

**Option A: Using ROS2 Camera Publisher**
```bash
ros2 run lane_detection_cv unity_lane_detection_node
ros2 run obstacle_detection unity_yolo_obstacle_node
ros2 run pothole_detection unity_pothole_detection_node
```

These nodes will:
- Capture camera frames from connected sensors (Intel RealSense, USB cameras, etc.)
- Process them through respective detection models
- Publish results as ROS2 messages via TCP-Endpoint to Unity

**Option B: Direct Camera Stream to Unity**
```bash
ros2 run rclcpp_components component_container
```
Then load camera driver components that publish image data

#### **Message Flow:**
```
Physical Camera/RealSense → ROS2 Node → Detection Processing → 
TCP-Endpoint Server → Network → Unity TCP Client → 3D Visualization
```

#### **Published Topics to Unity:**

| Topic | Type | Description |
|-------|------|-------------|
| `/lane_detection/lanes` | Detection output | Detected lane positions |
| `/obstacle_detection/objects` | Detection output | Detected obstacles with bounding boxes |
| `/pothole_detection/anomalies` | Detection output | Detected potholes with depth data |
| `/camera/image_raw` | Image | Raw camera frames (if camera node is running) |
| `/camera/depth` | PointCloud2 | Depth information from RealSense |

#### **Unity Subscriber Setup:**
1. Create ROS subscribers in Unity that listen to the above topics
2. Parse incoming messages and update 3D scene objects
3. Render detected objects as visual overlays on camera feed
4. Display lane boundaries and pothole locations in real-time

#### **Example Configuration in Unity C# Script:**
```csharp
// Subscribe to detection topics
ROSConnection ros = ROSConnection.GetOrCreateInstance();
ros.Subscribe<DetectionMsg>("/lane_detection/lanes", LaneCallback);
ros.Subscribe<DetectionMsg>("/obstacle_detection/objects", ObstacleCallback);
ros.Subscribe<DetectionMsg>("/pothole_detection/anomalies", PotholeCallback);
```

---

## Quick Start Guide

### **Installation:**

1. **Install ROS2 Humble or later:**
   ```bash
   # Follow official ROS2 installation guide for your OS
   ```

2. **Clone and build this workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone <this-repository>
   cd ..
   colcon build
   source install/setup.bash
   ```

3. **Install Python dependencies:**
   ```bash
   pip install -r ROS-TCP-Endpoint/requirements.txt
   # Install additional YOLO dependencies if needed
   pip install ultralytics opencv-python torch torchvision
   ```

### **Running the System:**

1. **Start ROS-TCP-Endpoint:**
   ```bash
   ros2 launch ros_tcp_endpoint endpoint.py
   ```

2. **Launch detection nodes (in separate terminals):**
   ```bash
   # Terminal 1: Lane Detection
   ros2 run lane_detection_cv unity_lane_detection_node
   
   # Terminal 2: Obstacle Detection
   ros2 run obstacle_detection unity_yolo_obstacle_node
   
   # Terminal 3: Pothole Detection
   ros2 run pothole_detection unity_pothole_detection_node
   ```

3. **Connect Unity simulation** to ROS2 server using the TCP endpoint

4. **Monitor topics:**
   ```bash
   ros2 topic list
   ros2 topic echo /lane_detection/lanes
   ```

---

## Contact & Support

For testing data, real-world validation results, and additional documentation, refer to:
- **Google Drive:** https://drive.google.com/drive/folders/1t5VTZ6-SJFxXP-aronKnmvw01V4GU6BO

---

**Last Updated:** June 2026
