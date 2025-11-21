# Live 3D Change Tracking with Orbbec Camera and OctoMap in ROS 2

This comprehensive guide combines real-time 3D mapping from an Orbbec camera with advanced change tracking capabilities. You'll learn to detect and visualize environmental changes as they happen, making this perfect for dynamic environment monitoring, security applications, and interactive robotics.

## 🧠 Core Concepts: Why This Matters

### What is Change Tracking?
Unlike basic mapping that just builds a static 3D map, **change tracking** continuously monitors your environment and highlights what's different. When someone moves an object, enters the room, or changes the environment, the system immediately detects and visualizes these changes.

### The Power of Live Camera + Change Tracking
By combining:
- **Live Orbbec camera feed** → Real-time 3D point cloud data
- **OctoMap tracking server** → Intelligent change detection
- **RViz2 visualization** → Live visual feedback

You get a system that can:

- Monitor areas for security purposes
- Track object movement in real-time
- Detect when new objects appear or disappear
- Provide instant visual feedback of environmental changes

### Technical Architecture
```
Orbbec Camera → Point Cloud → TF Tree → Tracking Server → Change Detection
     ↓              ↓           ↓            ↓              ↓
  Raw RGB+Depth → 3D Points → Positioning → Map Building → Difference Analysis
```

## 🔧 Prerequisites: Package Installation

Ensure you have all necessary ROS 2 packages. Replace `<distro>` with your ROS 2 version (humble, iron, etc.).

```bash
sudo apt update
sudo apt install ros-<distro>-octomap-server ros-<distro>-pcl-ros ros-<distro>-topic-tools
# If you don't have the Orbbec driver yet:
# Follow Orbbec's official ROS 2 installation guide
```

## ⚙️ Step 1: Configure the Tracking Launch File

Create a specialized launch file for live camera tracking. This combines the camera integration settings from the Orbbec tutorial with the tracking capabilities from the Hello World example.

Create `/home/kakarot/workspace/ros2_ws/src/octomap_mapping/octomap_server/launch/orbbec_tracking.launch.xml`:

```xml
<launch>
    <node pkg="octomap_server" exec="tracking_octomap_server_node" name="octomap_server">
        <!-- Map Resolution and Frames -->
        <param name="resolution" value="0.05" />  <!-- Fine detail for change detection -->
        <param name="frame_id" value="odom" />
        <param name="base_frame_id" value="base_link" />
        <param name="use_sim_time" value="false"/>
        
        <!-- TF Buffer for handling timing issues -->
        <param name="tf_buffer_duration" value="2.0" />
        
        <!-- Sensor Configuration -->
        <param name="sensor_model.max_range" value="5.0" />  <!-- Adjust based on room size -->
        
        <!-- Disable ground filtering for better change detection -->
        <param name="ground_filter/enable" value="false" />
        <param name="point_cloud_min_z" value="-100.0" />
        <param name="point_cloud_max_z" value="100.0" />
        
        <!-- TRACKING CONFIGURATION -->
        <param name="track_changes" value="true"/>          <!-- Enable change detection -->
        <param name="listen_changes" value="false"/>        <!-- Generate own changes -->
        <param name="change_id_frame" value="odom"/>         <!-- Frame for change coordinates -->
        
        <!-- Input topic from Orbbec camera -->
        <remap from="cloud_in" to="camera/depth/points" />
    </node>
</launch>
```

**Why each parameter matters:**
- `resolution="0.05"` → Higher resolution captures smaller changes
- `track_changes="true"` → Enables the core tracking functionality
- `listen_changes="false"` → Server generates its own change detection (not listening to external changes)
- `tf_buffer_duration="2.0"` → Prevents timing issues between camera and tracking server

## 🚀 Step 2: Launch Sequence (Critical Order!)

The order of launching is crucial to prevent race conditions and TF tree errors.

### **Terminal 1: Start Orbbec Camera** ⏰ *Start First*
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
# Replace with your specific Orbbec launch command
```

**Why first?** The camera driver establishes the sensor portion of the TF tree and begins publishing point cloud data. Wait 5-10 seconds for full initialization.

### **Terminal 2: Create World Frame** 🌍
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

**Why this step?** Creates the root `odom` frame where our map will be built. This is our "world coordinate system."

### **Terminal 3: Mount Camera to Robot** 📷
```bash
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link camera_link
```


**Why these numbers?**
- `0.1 0 0.2` → Camera is 10cm forward, 0cm sideways, 20cm up from robot center
- Adjust these values to match your physical camera mounting position
- **Critical:** `camera_link` must match your camera driver's frame name

### **Terminal 4: Launch Tracking Server** 🎯
```bash
ros2 launch octomap_server orbbec_tracking.launch.xml
```

**Why last?** Only launch after TF tree is complete and camera data is flowing.

## 🖥️ Step 3: RViz2 Setup for Change Visualization

Open RViz2 and configure it for optimal change tracking visualization:

```bash
rviz2
```

### Essential RViz2 Configuration:

1. **Set Global Frame:**
   - Global Options → Fixed Frame → `odom`

2. **Add Base Map Display:**
   - Add → By topic → `/octomap_point_cloud_centers` (PointCloud2)
   - Set Size (m) to `0.05`
   - Set Color to blue or white for base map

3. **Add Change Detection Display:**
   - Add → By topic → `/changes` (PointCloud2)
   - Set Size (m) to `0.08` (slightly larger for visibility)
   - Set Color to **red** or **yellow** for changes

4. **Add TF Tree (Optional but Helpful):**
   - Add → By display type → TF
   - Verify complete chain: `odom` → `base_link` → `camera_link` → ... → `camera_color_optical_frame`

5. **Add Raw Camera Data (Optional):**
   - Add → By topic → `/camera/depth/points` (PointCloud2)
   - Set Color to white or gray
   - This shows what the camera sees before processing

## 🔍 Step 4: Testing Change Detection

### Phase 1: Build Initial Environment Map
1. Let the system run for 30-60 seconds to build a stable base map of your environment
2. You should see blue/white points filling in the 3D structure of your room

### Phase 2: Introduce Changes and Watch Detection
Try these change scenarios:

#### **Small Object Changes:**
- Place a book on a table
- Move a chair
- Add/remove a cup

#### **Person Movement:**
- Walk into the camera's field of view
- Wave your arms
- Sit down or stand up

#### **Larger Changes:**
- Open/close a door
- Move furniture
- Add/remove boxes

### What You Should See:
- **Blue/White points:** Stable environment (base map)
- **Red/Yellow points:** Areas that have changed
- **Real-time updates:** Changes appear within 1-2 seconds

## 📊 Step 5: Monitor Change Detection Performance

### Check Topics Are Publishing:
```bash
# Verify all tracking topics are active
ros2 topic list | grep octomap
```

Expected output:
```
/changes
/octomap_binary
/octomap_full
/octomap_point_cloud_centers
```

### Monitor Change Rate:
```bash
# See how often changes are detected
ros2 topic hz /changes
```

### Inspect Change Data:
```bash
# Look at actual change coordinates
ros2 topic echo /changes --field data
```

### Monitor System Performance:
```bash
# Check if any messages are being dropped
# Look at Terminal 4 output for warnings
```

## 💾 Step 6: Save Maps and Changes

### Save Current Complete Map:
```bash
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=$HOME/live_environment_map.bt
```

### Save Change History (Advanced):
If you want to log changes over time, you can record the changes topic:
```bash
ros2 bag record /changes -o environment_changes_$(date +%Y%m%d_%H%M%S)
```

## 🎛️ Step 7: Fine-Tuning for Your Environment

### Adjust Sensitivity:
Edit the launch file parameters based on your needs:

```xml
<!-- For high-sensitivity change detection -->
<param name="resolution" value="0.03" />  <!-- Finer detail -->

<!-- For lower computational load -->
<param name="resolution" value="0.1" />   <!-- Coarser detail -->

<!-- For larger rooms -->
<param name="sensor_model.max_range" value="8.0" />

<!-- For smaller spaces -->
<param name="sensor_model.max_range" value="3.0" />
```

### Optimize Camera Position:
Adjust the transform in Terminal 3 to match your physical setup:
```bash
# Camera mounted higher and angled down
ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 0.2 0 base_link camera_link

# Camera mounted to the side
ros2 run tf2_ros static_transform_publisher 0 0.3 0.2 0 0 1.57 base_link camera_link
```

## 🚨 Troubleshooting Guide

### Issue: No Changes Detected
**Symptoms:** Red change points never appear, even when objects move

**Solutions:**
1. **Check tracking is enabled:**
   ```bash
   ros2 param get /octomap_server track_changes
   ```
   Should return `true`

2. **Verify change topic is publishing:**
   ```bash
   ros2 topic hz /changes
   ```

3. **Check camera range:** Objects might be outside `max_range` parameter

### Issue: Too Many False Changes
**Symptoms:** Red points appear everywhere, even in static areas

**Solutions:**
1. **Increase resolution for more stability:**
   ```xml
   <param name="resolution" value="0.08" />
   ```

2. **Improve camera mounting:** Reduce vibration and movement

3. **Check lighting conditions:** Consistent lighting improves depth accuracy

### Issue: TF Tree Errors
**Symptoms:** `octomap_server` reports frame lookup failures

**Solutions:**
1. **Verify TF chain:**
   ```bash
   ros2 run tf2_ros tf2_echo odom camera_color_optical_frame
   ```

2. **Check camera driver frame names:**
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

3. **Ensure correct camera frame name in Terminal 3**

### Issue: Performance Problems
**Symptoms:** Slow updates, dropped messages, high CPU usage

**Solutions:**
1. **Reduce resolution:**
   ```xml
   <param name="resolution" value="0.1" />
   ```

2. **Limit range:**
   ```xml
   <param name="sensor_model.max_range" value="3.0" />
   ```

3. **Check system resources:**
   ```bash
   htop  # Monitor CPU and memory usage
   ```

## 🎯 Expected Results and Use Cases

After successful setup, you should have:

### **Real-time Change Detection:**
- Immediate visual feedback when environment changes
- Persistent mapping of stable areas
- Clear distinction between static and dynamic elements

### **Practical Applications:**
- **Security Monitoring:** Detect unauthorized changes or intrusions
- **Warehouse Management:** Track inventory movement
- **Healthcare:** Monitor patient activity and room changes  
- **Smart Homes:** Automate responses to environmental changes
- **Research:** Study dynamic environments and object interactions

### **Performance Metrics:**
- **Change Detection Latency:** 1-3 seconds typical
- **Spatial Accuracy:** ~5cm with default resolution
- **Update Rate:** 1-10 Hz depending on system performance

## 📈 Advanced Features and Next Steps

Once you have the basic system working, consider these enhancements:

### **Multi-Camera Setup:**
- Add multiple Orbbec cameras for full room coverage
- Merge point clouds before tracking for comprehensive monitoring

### **Change Classification:**
- Process change data to classify types of changes (person, object, furniture)
- Implement alerts for specific change patterns

### **Historical Analysis:**
- Log changes over time to understand environment patterns
- Create heatmaps of frequently changing areas

### **Integration with Other Systems:**
- Connect to home automation systems
- Trigger actions based on detected changes
- Send notifications or alerts

You now have a complete live 3D change tracking system that combines the reliability of OctoMap with the real-time capabilities of modern depth cameras!