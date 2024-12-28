#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import threading
import paho.mqtt.client as mqtt
import json
import time

# MQTT 配置
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_USERNAME = "avp_manager"
MQTT_PASSWORD = "123456"
client_id = "avp_manager_id_test01"

# MQTT 主题
topic_perception_vehicles = "/perception/vehicles"
topic_pv_cmd_online = "/pv/cmd/online"  # 车辆在中心云上线

# Flag to indicate if any message is received and to control publishing
message_received = False
mqtt_publishing_enabled = False

# Global variables to hold x, y, yaw values
vehicle_data = {"x": 0.0, "y": 0.0, "yaw": 0.0}

# Sequence counters for MQTT messages
seq_ips = 0
seq_pv_cmd_online = 0

def callback(data):
    """
    Callback function for the /simple_vehicles_info topic.
    Updates the global vehicle_data dictionary with the latest x, y, z values.
    """
    global message_received, mqtt_publishing_enabled, vehicle_data
    message_received = True
    mqtt_publishing_enabled = True  # Enable publishing when new data is received
    vehicle_data["x"] = data.x
    vehicle_data["y"] = data.y
    vehicle_data["yaw"] = data.z  # Interpreted as yaw

    rospy.loginfo(f"Vehicle Info -> x: {data.x}, y: {data.y}, yaw (z): {data.z}")

def monitor_subscription():
    """
    Monitors the subscription to detect if messages stop coming.
    If no messages are received within a timeout, publishing is disabled.
    """
    global message_received, mqtt_publishing_enabled
    timeout = 5  # seconds
    rate = rospy.Rate(1)  # Check every second

    while not rospy.is_shutdown():
        if not message_received:
            mqtt_publishing_enabled = False
            rospy.logwarn("No messages received. MQTT publishing disabled.")
        else:
            message_received = False  # Reset flag to monitor next interval
        rate.sleep()

def mqtt_publisher(client):
    """
    Periodically publish vehicle data to the MQTT broker.
    Publishing is only enabled if messages are being received.
    """
    global seq_ips
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if mqtt_publishing_enabled:
            vehicle = {
                "LtoRear": 0.85,
                "id": 1,
                "l": 4.6,
                "rz": vehicle_data["yaw"],
                "speed": 0.0,
                "status": 1,
                "steer": 0.0,
                "w": 1.9,
                "wb": 2.73,
                "x": vehicle_data["x"],
                "y": vehicle_data["y"],
                "yaw": vehicle_data["yaw"]+180
            }

            vobj = {
                "payload": [vehicle],
                "sender": "hc001",
                "seq": seq_ips,
                "time": round(time.time(), 3),
            }

            vdoc = json.dumps(vobj, indent=4)
            result = client.publish(topic_perception_vehicles, vdoc)
            rospy.loginfo(f"Published to MQTT {topic_perception_vehicles}: {vdoc}, Result: {result.rc}")
            seq_ips += 1

        rate.sleep()

def publish_pv_cmd_online(client):
    """
    Publish the /pv/cmd/online message to the MQTT broker.
    """
    global seq_pv_cmd_online

    timestamp_nano = int(time.time() * 1e9)
    vin = "testvin011"
    lpn = "AB12311"
    key = "111"

    obj_header = {
        "seq": seq_pv_cmd_online,
        "time": timestamp_nano,
        "sender": "ParkingVehicle",
        "model_version": "1.0.0",
    }

    obj_payload = {
        "vin": vin,
        "lpn": lpn,
        "vkey": key,
    }

    obj = {
        "header": obj_header,
        "payload": obj_payload,
    }

    cmd_doc = json.dumps(obj, indent=4)
    result = client.publish(topic_pv_cmd_online, cmd_doc)
    rospy.loginfo(f"Published to MQTT {topic_pv_cmd_online}: {cmd_doc}, Result: {result.rc}")
    seq_pv_cmd_online += 1

def mqtt_setup():
    """
    Setup and return an MQTT client.
    """
    client = mqtt.Client(client_id=client_id)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        rospy.logerr(f"Failed to connect to MQTT broker: {e}")

    client.loop_start()
    return client

def listener():
    """
    Initializes the ROS node and subscribes to the /simple_vehicles_info topic.
    Starts MQTT publishing.
    """
    rospy.init_node('vehicle_info_listener', anonymous=True)

    # Setup MQTT client
    client = mqtt_setup()

    # Start the status publisher thread
    threading.Thread(target=monitor_subscription, daemon=True).start()

    # Start the MQTT publisher thread
    threading.Thread(target=mqtt_publisher, args=(client,), daemon=True).start()

    # Publish the /pv/cmd/online message once
    publish_pv_cmd_online(client)

    # Subscribe to the /simple_vehicles_info topic
    rospy.Subscriber("/simple_vehicles_info", Point, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
