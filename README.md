# data_collector

A ROS 2 package for collecting and logging flight data from SITL and AP_DDS sources. Data is saved as CSV or compressed CSV (`.csv.gz`) files for later analysis.

---

## Directory Structure (Short)

```
.  
├── LICENSE  
├── README.md  
├── package.xml  
├── setup.cfg  
├── setup.py  
├── __init__.py  
├── bash_scripts/  
│   ├── run_drone_logger.sh  
│   ├── run_pipeline.sh  
│   └── unzip_csv.sh  
├── zsh_scripts/  
│   ├── run_drone_logger.zsh  
│   ├── run_pipeline.zsh  
│   └── unzip_csv.zsh  
├── data_collector/  
│   ├── __init__.py  
│   └── my_node.py  
├── build/  
├── install/  
├── log/  
├── logs/  
├── README/  
├── resource/  
├── test/  
```

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

## Quickstart

### 1. Build the Package

```bash
colcon build --packages-select data_collector
```

### 2. Source the Workspace

```bash
source install/setup.bash
```

### 3. Run the Logger Node

```bash
ros2 run data_collector my_node
```

### 4. Start/Stop Logging

Use the provided services to control logging:

```bash
ros2 service call /start_logging std_srvs/srv/Trigger
ros2 service call /stop_logging std_srvs/srv/Trigger
```

### 5. Decompress a Log File

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

## File Overview

- `data_collector/` — Python package with main node (`my_node.py`)
- `logs/` — Output directory for log files (created automatically)
- `test/` — Test scripts for code quality and compliance
- `bash_scripts/` and `zsh_scripts/` — Example shell scripts to launch the logger and manage logs
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
- If you encounter permission errors, ensure you have write access to the `logs/` directory.
- For ROS 2 environment issues, make sure you have sourced the correct setup script.

---

## Full Directory Structure

```
.  
├── LICENSE  
├── README.md  
├── package.xml  
├── setup.cfg  
├── setup.py  
├── __init__.py  
├── bash_scripts/  
│   ├── run_drone_logger.sh  
│   ├── run_pipeline.sh  
│   └── unzip_csv.sh  
├── zsh_scripts/  
│   ├── run_drone_logger.zsh  
│   ├── run_pipeline.zsh  
│   └── unzip_csv.zsh  
├── data_collector/  
│   ├── __init__.py  
│   └── my_node.py  
├── build/  
├── install/  
├── log/  
├── logs/  
├── README/  
├── resource/  
├── test/  
│   ├── test_copyright.py  
│   ├── test_flake8.py  
│   ├── test_pep257.py  
│   └── logs/  
```

---

## License

See [LICENSE](LICENSE)
