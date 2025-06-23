#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Callable
from dataclasses import dataclass
from threading import Lock
from concurrent.futures import ThreadPoolExecutor
import socket
import threading
import json
from queue import Queue

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger

TOPIC_CONFIG = {
    '/ap/battery': (BatteryState, ['voltage', 'current']),
    '/ap/navsat': (NavSatFix, ['latitude', 'longitude', 'altitude']),
    '/ap/twist/filtered': (TwistStamped, ['twist.linear.x', 'twist.linear.y', 'twist.linear.z']),
    '/ap/imu/experimental/data': (Imu, [
        'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
        'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z'])
}

@dataclass
class LogConfig:
    max_buffer_size: int = 1  # Effectively disables buffering, flushes on every message
    log_dir: str = "data_collector/logs"
    compress_logs: bool = False # Enable gzip compression for logs
    health_check_interval: float = 5.0
    message_timeout: float = 10.0
    tcp_stream_rate: str = '1.0'  # Can be float (Hz) or 'live' for instant

    def __post_init__(self):
        # Ensure log_dir is always an absolute path
        self.log_dir = str(Path(self.log_dir).absolute())

class TCPLogServer:
    def __init__(self, host='0.0.0.0', port=9000):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.clients = []
        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._accept_clients, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def _accept_clients(self):
        while self.running:
            try:
                client_sock, _ = self.server_socket.accept()
                with self.lock:
                    self.clients.append(client_sock)
            except Exception:
                break

    def send(self, data: dict):
        msg = json.dumps(data) + '\n'
        with self.lock:
            for client in self.clients[:]:
                try:
                    client.sendall(msg.encode('utf-8'))
                except Exception:
                    self.clients.remove(client)
                    try:
                        client.close()
                    except Exception:
                        pass

    def send_batch(self, data_list):
        # Send a list of dicts as JSON lines
        with self.lock:
            for data in data_list:
                msg = json.dumps(data) + '\n'
                for client in self.clients[:]:
                    try:
                        client.sendall(msg.encode('utf-8'))
                    except Exception:
                        self.clients.remove(client)
                        try:
                            client.close()
                        except Exception:
                            pass

    def send_instant(self, data):
        # Send a single dict instantly (for 'live' mode)
        msg = json.dumps(data) + '\n'
        with self.lock:
            for client in self.clients[:]:
                try:
                    client.sendall(msg.encode('utf-8'))
                except Exception:
                    self.clients.remove(client)
                    try:
                        client.close()
                    except Exception:
                        pass

    def stop(self):
        self.running = False
        try:
            self.server_socket.close()
        except Exception:
            pass
        with self.lock:
            for client in self.clients:
                try:
                    client.close()
                except Exception:
                    pass
            self.clients.clear()

class DroneLogger(Node):
    def __init__(self, config: LogConfig = LogConfig()):
        super().__init__('drone_logger')
        self.config = config
        self.log_buffer = []
        self.buffer_lock = Lock()
        self.logging_active = False
        self.log_dir = Path(self.config.log_dir)
        self.log_dir.mkdir(exist_ok=True, parents=True)
        self.log_file_path = None
        self.header_written = False
        self.last_message_time = {'/ap/battery': datetime.now()}
        self.health_timer = self.create_timer(self.config.health_check_interval, self.check_health)
        self.thread_pool = ThreadPoolExecutor(max_workers=2)
        self.flush_in_progress = False
        self._field_cache = {}
        self.tcp_server = TCPLogServer(host='0.0.0.0', port=9000)
        self.tcp_server.start()
        self.latest_topic_data = {}  # topic: latest data dict
        self.tcp_stream_rate = self.config.tcp_stream_rate
        self.tcp_stream_timer_active = True
        if self.tcp_stream_rate != 'live':
            self.tcp_stream_timer = threading.Thread(target=self._tcp_stream_loop, daemon=True)
            self.tcp_stream_timer.start()
        # Use make_callback for topics to enable CSV logging
        self.create_subscription(BatteryState, '/ap/battery', self.make_callback('/ap/battery', ['voltage', 'current']), qos_profile=self.qos_profile)
        self.create_subscription(NavSatFix, '/ap/navsat', self.make_callback('/ap/navsat', ['latitude', 'longitude', 'altitude']), qos_profile=self.qos_profile)
        self.create_subscription(TwistStamped, '/ap/twist/filtered', self.make_callback('/ap/twist/filtered', ['twist.linear.x', 'twist.linear.y', 'twist.linear.z']), qos_profile=self.qos_profile)
        self.create_subscription(Imu, '/ap/imu/experimental/data', self.make_callback('/ap/imu/experimental/data', [
            'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
            'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z']), qos_profile=self.qos_profile)
        self.create_subscription(Clock, '/ap/clock', self.clock_callback, qos_profile=self.qos_profile)
        self.create_subscription(GeoPoseStamped, '/ap/geopose/filtered', self.geopose_filtered_callback, qos_profile=self.qos_profile)
        self.create_subscription(GeoPointStamped, '/ap/gps_global_origin/filtered', self.gps_global_origin_filtered_callback, qos_profile=self.qos_profile)
        self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_filtered_callback, qos_profile=self.qos_profile)
        self.create_subscription(TFMessage, '/ap/tf_static', self.tf_static_callback, qos_profile=self.qos_profile)
        self.create_subscription(Time, '/ap/time', self.time_callback, qos_profile=self.qos_profile)
        self.create_service(Trigger, 'start_logging', self._toggle_logging(True))
        self.create_service(Trigger, 'stop_logging', self._toggle_logging(False))
        self.get_logger().info('Logger node initialized and listening...')

    qos_profile = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT
    )

    def _toggle_logging(self, start: bool):
        def callback(request, response):
            if self.logging_active == start:
                response.success = False
                response.message = f'Logging is already {"active" if start else "inactive"}.'
            else:
                self.logging_active = start
                if start:
                    self._open_log_file()
                else:
                    self._flush_buffer()
                    self._close_log_file()
                response.success = True
                response.message = f'Logging {"started" if start else "stopped and buffer flushed"}.'
                self.get_logger().info(response.message)
            return response
        return callback

    def _open_log_file(self):
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = self.log_dir / f'drone_log_{ts}.csv'
        self.header_written = False
        self.get_logger().info(f"Log file opened: {self.log_file_path}")

    def _close_log_file(self):
        self.get_logger().info(f"Log file closed: {self.log_file_path}")
        self.log_file_path = None
        self.header_written = False

    def battery_callback(self, msg: BatteryState):
        # Example: log voltage and current
        self.get_logger().info(f"Battery: voltage={msg.voltage}, current={msg.current}")

    def clock_callback(self, msg: Clock):
        self.get_logger().info(f"Clock: sec={msg.clock.sec}, nanosec={msg.clock.nanosec}")

    def geopose_filtered_callback(self, msg: GeoPoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.get_logger().info(
            f"GeoPose: lat={pos.latitude}, lon={pos.longitude}, alt={pos.altitude}, "
            f"orientation=({ori.x}, {ori.y}, {ori.z}, {ori.w})"
        )

    def gps_global_origin_filtered_callback(self, msg: GeoPointStamped):
        pos = msg.position
        self.get_logger().info(
            f"GPS Origin: lat={pos.latitude}, lon={pos.longitude}, alt={pos.altitude}"
        )

    def imu_experimental_data_callback(self, msg: Imu):
        av = msg.angular_velocity
        la = msg.linear_acceleration
        self.get_logger().info(
            f"IMU: ang_vel=({av.x}, {av.y}, {av.z}), lin_acc=({la.x}, {la.y}, {la.z})"
        )

    def navsat_callback(self, msg: NavSatFix):
        self.get_logger().info(
            f"NavSat: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
        )

    def pose_filtered_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.get_logger().info(
            f"Pose: x={pos.x}, y={pos.y}, z={pos.z}, orientation=({ori.x}, {ori.y}, {ori.z}, {ori.w})"
        )

    def tf_static_callback(self, msg: TFMessage):
        for t in msg.transforms:
            tr = t.transform.translation
            rot = t.transform.rotation
            self.get_logger().info(
                f"TF: {t.header.frame_id} -> {t.child_frame_id}, "
                f"trans=({tr.x}, {tr.y}, {tr.z}), rot=({rot.x}, {rot.y}, {rot.z}, {rot.w})"
            )

    def time_callback(self, msg: Time):
        self.get_logger().info(f"Time: sec={msg.sec}, nanosec={msg.nanosec}")

    def twist_filtered_callback(self, msg: TwistStamped):
        lin = msg.twist.linear
        ang = msg.twist.angular
        self.get_logger().info(
            f"Twist: lin=({lin.x}, {lin.y}, {lin.z}), ang=({ang.x}, {ang.y}, {ang.z})"
        )

    def make_callback(self, topic: str, fields: list) -> Callable:
        def cb(msg):
            if not self.logging_active:
                return
            self.last_message_time[topic] = datetime.now()
            try:
                data = self._extract_fields(msg, topic, fields)
                flush_needed = False
                with self.buffer_lock:
                    self.log_buffer.append(data)
                    if len(self.log_buffer) >= self.config.max_buffer_size and not self.flush_in_progress:
                        self.flush_in_progress = True
                        flush_needed = True
                # Store latest data for TCP streaming
                self.latest_topic_data[topic] = data
                # If in 'live' mode, send instantly
                if self.tcp_stream_rate == 'live':
                    self.tcp_server.send_instant(data)
                if flush_needed:
                    self.thread_pool.submit(self._flush_buffer)
            except Exception as e:
                self.get_logger().error(f'Error processing message from {topic}: {str(e)}')
        return cb

    def _tcp_stream_loop(self):
        import time
        try:
            rate = float(self.tcp_stream_rate)
        except Exception:
            rate = 1.0
        while self.tcp_stream_timer_active:
            if self.logging_active and self.latest_topic_data:
                batch = list(self.latest_topic_data.values())
                self.tcp_server.send_batch(batch)
            time.sleep(1.0 / rate)

    def _extract_fields(self, msg, topic: str, fields: list) -> Dict[str, Any]:
        # Helper to extract nested fields from a message using dot notation
        def get_field(val, path):
            if path in self._field_cache:
                attrs = self._field_cache[path]
            else:
                attrs = path.split('.')
                self._field_cache[path] = attrs
            for attr in attrs:
                val = getattr(val, attr, 'N/A')
                if val == 'N/A':
                    break
            return val
        # Prepare the base data dictionary with timestamp, topic, and sequence number
        data = {
            'timestamp': datetime.utcnow().isoformat(timespec='milliseconds') + 'Z',
            'topic': topic,
            'sequence': getattr(getattr(msg, 'header', None), 'seq', 0)
        }
        # Extract each requested field and handle invalid or missing values
        for field in fields:
            val = get_field(msg, field)
            if isinstance(val, (int, float)) and not isinstance(val, bool):
                # Check for extremely large values or NaN
                if abs(val) > 1e10 or (isinstance(val, float) and not val == val):
                    self.get_logger().warn(f'Invalid value for {field}: {val}')
                    val = 'N/A'
            if val == 'N/A':
                self.get_logger().warn(f'Field extraction failed for {field}')
            data[field] = val
        return data

    def _get_log_filename(self):
        # Generate a timestamped log filename, using .csv.gz if compression is enabled
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        if self.config.compress_logs:
            base = self.log_dir / f'drone_log_{ts}.csv.gz'
        else:
            base = self.log_dir / f'drone_log_{ts}.csv'
        return str(base)

    def _flush_buffer(self):
        with self.buffer_lock:
            if not self.log_buffer or not self.log_file_path:
                self.flush_in_progress = False
                return
            logs_to_save = self.log_buffer.copy()
            self.log_buffer.clear()
        try:
            file_exists = self.log_file_path.exists()
            with open(self.log_file_path, 'a', newline='') as f:
                self._write_csv(f, logs_to_save, write_header=not file_exists or not self.header_written)
                self.header_written = True
            self.get_logger().info(f'Saved {len(logs_to_save)} records to {self.log_file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save logs: {str(e)}')
        finally:
            with self.buffer_lock:
                self.flush_in_progress = False

    def _write_csv(self, file_obj, logs: list, write_header=True):
        if not logs:
            return
        # Always use the union of all possible fields from all topics
        all_fields = set(['timestamp', 'topic', 'sequence'])
        for topic, (_, fields) in TOPIC_CONFIG.items():
            all_fields.update(fields)
        fieldnames = sorted(all_fields)
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        for row in logs:
            # Ensure all fields are present in each row
            for field in fieldnames:
                if field not in row:
                    row[field] = ''
            writer.writerow(row)

    def check_health(self):
        now = datetime.now()
        # Only check /ap/battery
        last_time = self.last_message_time['/ap/battery']
        if (now - last_time).total_seconds() > self.config.message_timeout:
            self.get_logger().warn(f'No messages received from /ap/battery in {self.config.message_timeout} seconds')

    def destroy_node(self):
        self.tcp_stream_timer_active = False
        self.logging_active = False
        if hasattr(self, 'health_timer'):
            self.health_timer.cancel()
        self._flush_buffer()
        if hasattr(self, 'thread_pool'):
            self.thread_pool.shutdown(wait=True)
        if hasattr(self, 'tcp_server'):
            self.tcp_server.stop()
        try:
            self.get_logger().info('Logger node destroyed.')
        except Exception:
            pass  # Suppress any logging errors on shutdown
        super().destroy_node()


def main(args=None):
    import rclpy.executors
    import warnings
    rclpy.init(args=args)
    node = DroneLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.get_logger().info('Keyboard interrupt received...')
        except Exception:
            pass
    except rclpy.executors.ExternalShutdownException:
        pass  # Clean shutdown, suppress all errors
    finally:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            try:
                node.destroy_node()
            except Exception:
                pass  # Suppress any shutdown errors