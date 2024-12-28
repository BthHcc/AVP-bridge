#!/usr/bin/env python
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import Float32MultiArray

# 全局变量
control_pub = None
MQTT_BROKER = "10.180.210.20"  # MQTT 代理的 IP 地址 --> 天台电脑
MQTT_PORT = 1883  # MQTT 代理的端口号
MQTT_TOPIC = "/ec/cmd/vehicle_control/testvin011/111"  # 订阅的主题
MQTT_USERNAME = "avp_manager"  # MQTT 用户名
MQTT_PASSWORD = "123456"  # MQTT 密码

client_id = "avp_manager_id_test1"  # 客户端 ID

# 当连接到 MQTT broker 时的回调函数
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Successfully connected to MQTT broker")
        client.subscribe(MQTT_TOPIC)  # 订阅主题
    else:
        rospy.logerr(f"Failed to connect to MQTT broker, return code {rc}")

# 当从 MQTT broker 接收到消息时的回调函数
def on_message(client, userdata, msg):
    try:
        # 解析 JSON 格式的消息
        message = json.loads(msg.payload.decode())
        # 获取 payload 中的速度、转向角和刹车信息
        speed = message.get("payload", {}).get("speed", 0.0)  # 默认速度为 0.0
        steer = message.get("payload", {}).get("steer", 0.0)  # 默认转向角为 0.0
        brake = message.get("payload", {}).get("brake", 0.0)  # 默认刹车为 0.0

        # 创建 Float32MultiArray 消息
        control_msg = Float32MultiArray()
        control_msg.data = [speed, steer, brake]  # 按顺序传递速度、转向角和刹车

        # 发布控制指令
        control_pub.publish(control_msg)
        rospy.loginfo(f"Published control message: speed={speed}, steer={steer}, brake={brake}")

    except Exception as e:
        rospy.logerr(f"Error processing message: {e}")

# 初始化 ROS 节点，并创建 MQTT 客户端
def main():
    # 初始化 ROS 节点
    rospy.init_node('mqtt_ros_control_node')

    # 创建 ROS 发布者
    global control_pub
    control_pub = rospy.Publisher('/carla/ego_vehicle/control_cmd', Float32MultiArray, queue_size=10)

    # 创建 MQTT 客户端
    client = mqtt.Client(client_id=client_id)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)  # 设置用户名和密码

    # 设置回调函数
    client.on_connect = on_connect
    client.on_message = on_message

    # 连接到 MQTT broker
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        rospy.logerr(f"Failed to connect to MQTT broker: {e}")
        return

    # 启动 MQTT 客户端循环
    client.loop_start()

    # 保持 ROS 节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
