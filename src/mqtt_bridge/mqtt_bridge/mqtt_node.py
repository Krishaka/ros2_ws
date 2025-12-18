#!/usr/bin/env python3

import json
import math
import threading
from datetime import datetime, timezone
from typing import Any, Dict

import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


MQTT_BROKER_IP = "192.168.0.3"
MQTT_PORT = 1883

BOT_ID = "bot-001"

# ROS -> MQTT (position)
ROS_ODOM_TOPIC = "/rtabmap/odom"
MQTT_POSITION_TOPIC = f"warehouse/bots/{BOT_ID}/position"

# MQTT -> ROS (task command)
MQTT_TASK_CMD_TOPIC = f"warehouse/bots/{BOT_ID}/task/command"
ROS_TASK_CMD_TOPIC = "/warehouse/task/command"   # other ROS nodes subscribe to this


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_iso8601_utc(sec: int, nanosec: int) -> str:
    t = sec + nanosec * 1e-9
    return datetime.fromtimestamp(t, tz=timezone.utc).isoformat().replace("+00:00", "Z")


class MqttRosBridge(Node):
    def __init__(self):
        super().__init__("mqtt_ros_bridge")

        # ROS subscriber: /rtabmap/odom -> MQTT position
        self.odom_sub = self.create_subscription(Odometry, ROS_ODOM_TOPIC, self.on_odom, 10)

        # ROS publisher: MQTT task command -> /warehouse/task/command
        self.task_pub = self.create_publisher(String, ROS_TASK_CMD_TOPIC, 10)

        # MQTT client (runs callbacks in paho thread)
        self.mqtt = mqtt.Client(client_id=f"ros2_mqtt_bridge_{BOT_ID}", clean_session=True)
        self.mqtt.on_connect = self.on_mqtt_connect
        self.mqtt.on_disconnect = self.on_mqtt_disconnect
        self.mqtt.on_message = self.on_mqtt_message

        self.mqtt.connect(MQTT_BROKER_IP, MQTT_PORT, keepalive=60)
        self.mqtt.loop_start()

    # ---------------- ROS -> MQTT ----------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        theta = yaw_from_quat(q.x, q.y, q.z, q.w)
        ts = stamp_to_iso8601_utc(msg.header.stamp.sec, msg.header.stamp.nanosec)

        payload = {
            "position": {"x": float(p.x), "y": float(p.y), "theta": float(theta)},
            "timestamp": ts,
        }

        # MQTT QoS = 1
        self.mqtt.publish(MQTT_POSITION_TOPIC, json.dumps(payload), qos=1, retain=False)

    # ---------------- MQTT -> ROS ----------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected rc={rc}")
        # subscribe with QoS 1
        client.subscribe(MQTT_TASK_CMD_TOPIC, qos=1)

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def on_mqtt_message(self, client, userdata, message):
        # Called from paho thread; publishing to ROS is safe in rclpy for simple publishers,
        # but keep it minimal and exception-safe.
        try:
            raw = message.payload.decode("utf-8", errors="strict")
            data: Dict[str, Any] = json.loads(raw)

            # Minimal validation / normalization
            if data.get("command") != "assign_task":
                return
            if "task" not in data or not isinstance(data["task"], dict):
                return

            out = String()
            out.data = json.dumps(data)  # forward entire command JSON to ROS nodes
            self.task_pub.publish(out)

        except Exception as e:
            self.get_logger().error(f"MQTT message parse/publish failed: {e}")

    def destroy_node(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
