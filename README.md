# Directory Structure

```
.  
├── LICENSE  
├── README.md  
├── package.xml  
├── run_drone_logger.sh  
├── setup.cfg  
├── setup.py  
├── unzip_csv.sh  
├── __init__.py  
├── build/  
│   ├── COLCON_IGNORE  
│   └── data_collector/  
│       ├── colcon_build.rc  
│       ├── colcon_command_prefix_setup_py.sh  
│       ├── colcon_command_prefix_setup_py.sh.env  
│       ├── install.log  
│       ├── build/  
│       ├── data_collector.egg-info/  
│       └── prefix_override/  
├── data_collector/  
│   ├── __init__.py  
│   └── my_node.py  
├── install/  
│   ├── _local_setup_util_ps1.py  
│   ├── _local_setup_util_sh.py  
│   ├── COLCON_IGNORE  
│   ├── local_setup.bash  
│   ├── local_setup.ps1  
│   ├── local_setup.sh  
│   ├── local_setup.zsh  
│   ├── setup.bash  
│   ├── setup.ps1  
│   ├── setup.sh  
│   ├── setup.zsh  
│   └── data_collector/  
│       ├── lib/  
│       └── share/  
├── log/  
│   ├── latest  
│   ├── latest_build  
│   ├── COLCON_IGNORE  
│   └── build_*/  
├── logs/  
│   ├── drone_log_*.csv.gz  
│   └── drone_log_*.csv  
├── README/  
│   ├── MAVROS && SITL  
│   └── RosRunDrone.txt  
├── resource/  
│   └── data_collector  
├── test/  
│   ├── test_copyright.py  
│   ├── test_flake8.py  
│   ├── test_pep257.py  
│   └── logs/  
```

# data_collector

A ROS 2 package for collecting and logging flight data from SITL and AP_DDS sources. Data is saved as CSV or compressed CSV (`.csv.gz`) files for later analysis.

---

## Features

- **Subscribes to key flight telemetry topics** (battery, GPS, IMU, pose, etc.)
- **Logs selected fields** from each message to a buffer and periodically flushes to disk
- **Supports plain CSV and gzip-compressed CSV output** for efficient storage
- **Health monitoring** for message timeouts (warns if no data is received for a configurable period)
- **Start/stop logging via ROS 2 services** for flexible control
- **Thread-safe, non-blocking log writing** using a buffer and thread pool
- **Easily configurable** via a Python dataclass

---

## Topics Subscribed

The node subscribes to the following topics and logs selected fields:

| Topic | Message Type | Fields Logged |
|-------|--------------|---------------|
| `/ap/battery` | `sensor_msgs/BatteryState` | voltage, current |
| `/ap/navsat` | `sensor_msgs/NavSatFix` | latitude, longitude, altitude |
| `/ap/twist/filtered` | `geometry_msgs/TwistStamped` | twist.linear.x, twist.linear.y, twist.linear.z |
| `/ap/imu/experimental/data` | `sensor_msgs/Imu` | angular_velocity (x, y, z), linear_acceleration (x, y, z) |
| `/ap/clock` | `rosgraph_msgs/Clock` |  |
| `/ap/geopose/filtered` | `geographic_msgs/GeoPoseStamped` |  |
| `/ap/gps_global_origin/filtered` | `geographic_msgs/GeoPointStamped` |  |
| `/ap/pose/filtered` | `geometry_msgs/PoseStamped` |  |
| `/ap/tf_static` | `tf2_msgs/TFMessage` |  |
| `/ap/time` | `builtin_interfaces/Time` |  |

---

## Services

- **`start_logging`** (`std_srvs/Trigger`): Start logging data. Returns a success message if logging was started.
- **`stop_logging`** (`std_srvs/Trigger`): Stop logging and flush any buffered data to disk. Returns a success message if logging was stopped.

---

## Output

- Logs are saved in the `logs/` directory by default (can be changed in config)
- Filenames are timestamped, e.g. `drone_log_YYYYMMDD_HHMMSS.csv.gz`
- Each row contains: timestamp (UTC, ISO8601), topic, sequence number, and selected fields from the message
- If compression is enabled, files are written as gzip-compressed CSVs for efficient storage

---

## Usage

### Build

Build the package using colcon:

```bash
colcon build --packages-select data_collector
```

### Source the workspace

Source the workspace setup script:

```bash
source install/setup.bash
```

### Run the logger node

Launch the logger node:

```bash
ros2 run data_collector my_node
```

### Start/Stop Logging

Use the provided services to control logging:

```bash
ros2 service call /start_logging std_srvs/srv/Trigger
ros2 service call /stop_logging std_srvs/srv/Trigger
```

### Example: Decompress a log file

To decompress a `.csv.gz` log file:

```bash
gunzip logs/drone_log_YYYYMMDD_HHMMSS.csv.gz
```

---

## Configuration

Logging behavior can be customized by editing the `LogConfig` dataclass in `my_node.py`:

- `max_buffer_size`: Number of messages to buffer before writing to disk (set to 1 for immediate flush)
- `log_dir`: Directory where logs are saved
- `compress_logs`: Enable/disable gzip compression for logs
- `health_check_interval`: How often to check for missing messages (seconds)
- `message_timeout`: Time to wait before warning about missing messages (seconds)

---

## File Structure

- `data_collector/` — Python package with main node (`my_node.py`)
- `logs/` — Output directory for log files (created automatically)
- `test/` — Test scripts for code quality and compliance
- `run_drone_logger.sh` — Example shell script to launch the logger
- `unzip_csv.sh` — Helper script to decompress `.csv.gz` log files
- `README/` — Additional documentation and usage notes
- `resource/` — ROS resource files

---

## Requirements

- ROS 2 (tested with Humble; **not tested on Foxy**)
- Python 3.7+
- Standard ROS 2 message packages (see `package.xml` for dependencies)

---

## Troubleshooting

- If logs are not being written, ensure the node is running and logging is started via the service call.
- Check the `logs/` directory for output files. If using compression, verify with `file logs/*.csv.gz`.
- For missing fields or errors, review the node's console output for warnings.

---

## License

See [LICENSE](LICENSE)
