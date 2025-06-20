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

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
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
    max_buffer_size: int = 1000
    log_dir: str = "logs"
    compress_logs: bool = False  # Changed default to False for plain CSV
    health_check_interval: float = 5.0
    message_timeout: float = 10.0

class DroneLogger(Node):
    def __init__(self, config: LogConfig = LogConfig()):
        super().__init__('drone_logger')
        self.config = config
        self.log_buffer = []
        self.buffer_lock = Lock()
        self.logging_active = False
        self.log_dir = Path(self.config.log_dir)
        self.log_dir.mkdir(exist_ok=True, parents=True)
        # Only track /ap/battery
        self.last_message_time = {'/ap/battery': datetime.now()}
        self.health_timer = self.create_timer(self.config.health_check_interval, self.check_health)
        self.thread_pool = ThreadPoolExecutor(max_workers=2)
        self.flush_in_progress = False
        self._field_cache = {}
        self.create_subscription(BatteryState, '/ap/battery', self.battery_callback, qos_profile=self.qos_profile)
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
                if not start:
                    self._flush_buffer()
                response.success = True
                response.message = f'Logging {"started" if start else "stopped and buffer flushed"}.'
                self.get_logger().info(response.message)
            return response
        return callback

    def battery_callback(self, msg: BatteryState):
        # if not self.logging_active:
        #     return
        self.last_message_time['/ap/battery'] = datetime.now()
        data = {
            'timestamp': datetime.utcnow().isoformat(timespec='milliseconds') + 'Z',
            'topic': '/ap/battery',
            'header_stamp_sec': getattr(msg.header.stamp, 'sec', 'N/A'),
            'header_stamp_nanosec': getattr(msg.header.stamp, 'nanosec', 'N/A'),
            'header_frame_id': getattr(msg.header, 'frame_id', 'N/A'),
            'voltage': msg.voltage,
            'temperature': msg.temperature,
            'current': msg.current,
            'charge': msg.charge,
            'capacity': msg.capacity,
            'design_capacity': msg.design_capacity,
            'percentage': msg.percentage,
            'power_supply_status': msg.power_supply_status,
            'power_supply_health': msg.power_supply_health,
            'power_supply_technology': msg.power_supply_technology,
            'present': msg.present,
            'cell_voltage': list(msg.cell_voltage) if hasattr(msg, 'cell_voltage') else [],
            'cell_temperature': list(msg.cell_temperature) if hasattr(msg, 'cell_temperature') else [],
            'location': msg.location,
            'serial_number': msg.serial_number,
        }
        self.get_logger().info(f'Received battery data: {data}')
        with self.buffer_lock:
            self.log_buffer.append(data)
            if len(self.log_buffer) >= self.config.max_buffer_size and not self.flush_in_progress:
                self.flush_in_progress = True
                self.thread_pool.submit(self._flush_buffer)

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
                if flush_needed:
                    self.thread_pool.submit(self._flush_buffer)
            except Exception as e:
                self.get_logger().error(f'Error processing message from {topic}: {str(e)}')
        return cb

    def _extract_fields(self, msg, topic: str, fields: list) -> Dict[str, Any]:
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
        data = {
            'timestamp': datetime.utcnow().isoformat(timespec='milliseconds') + 'Z',
            'topic': topic,
            'sequence': getattr(getattr(msg, 'header', None), 'seq', 0)
        }
        for field in fields:
            val = get_field(msg, field)
            if isinstance(val, (int, float)) and not isinstance(val, bool):
                if abs(val) > 1e10 or (isinstance(val, float) and not val == val):
                    self.get_logger().warn(f'Invalid value for {field}: {val}')
                    val = 'N/A'
            if val == 'N/A':
                self.get_logger().warn(f'Field extraction failed for {field}')
            data[field] = val
        return data

    def _get_log_filename(self):
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        base = self.log_dir / f'drone_log_{ts}.csv'
        return str(base)  # Always return .csv, not .gz

    def _flush_buffer(self):
        with self.buffer_lock:
            if not self.log_buffer:
                self.flush_in_progress = False
                return
            logs_to_save = self.log_buffer.copy()
            self.log_buffer.clear()
        filename = self._get_log_filename()
        try:
            with open(filename, 'w', newline='') as f:
                self._write_csv(f, logs_to_save)
            self.get_logger().info(f'Saved {len(logs_to_save)} records to {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save logs: {str(e)}')
        finally:
            with self.buffer_lock:
                self.flush_in_progress = False

    def _write_csv(self, file_obj, logs: list):
        if not logs:
            return
        fieldnames = sorted({k for d in logs for k in d})
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(logs)

    def check_health(self):
        now = datetime.now()
        # Only check /ap/battery
        last_time = self.last_message_time['/ap/battery']
        if (now - last_time).total_seconds() > self.config.message_timeout:
            self.get_logger().warn(f'No messages received from /ap/battery in {self.config.message_timeout} seconds')

    def destroy_node(self):
        self.logging_active = False
        if hasattr(self, 'health_timer'):
            self.health_timer.cancel()
        self._flush_buffer()
        if hasattr(self, 'thread_pool'):
            self.thread_pool.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received...')
    finally:
        node.destroy_node()