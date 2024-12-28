#!/usr/bin/env python3

import rospy
from clustering_and_tracking.msg import ObstacleArray, VehicleArray, Vehicle
import rosbag
import threading

# 全局变量
obstacle_data = None
vehicle_data = None
use_bag_file = False  # 是否使用ROS bag文件
bag_path = "SEIEE_vehicle_info.bag"  # 默认ROS bag路径

def callback_obstacle(data):
    global obstacle_data
    obstacle_data = data

def callback_vehicle(data):
    global vehicle_data
    vehicle_data = data

def read_bag_file():
    global obstacle_data, vehicle_data

    # 使用ROS bag文件读取数据
    rospy.loginfo("Reading data from ROS bag file...")
    bag = rosbag.Bag(bag_path)
    rate = rospy.Rate(10)  # 模拟读取频率为10Hz

    try:
        for topic, msg, t in bag.read_messages(topics=["/obstacles_info", "/vehicles_info"]):
            if rospy.is_shutdown():
                break
            if topic == "/obstacles_info":
                obstacle_data = msg
            elif topic == "/vehicles_info":
                vehicle_data = msg
            rate.sleep()
    finally:
        bag.close()
        rospy.loginfo("Finished reading ROS bag file.")

def publisher_node():
    # 初始化ROS节点
    rospy.init_node('topic_publisher', anonymous=True)

    # 订阅主题（实时模式下）
    if not use_bag_file:
        rospy.Subscriber("/obstacles_info", ObstacleArray, callback_obstacle)
        rospy.Subscriber("/vehicles_info", VehicleArray, callback_vehicle)

    # 发布主题
    pub = rospy.Publisher("topic_perception_vehicles", VehicleArray, queue_size=10)

    # 定义发布频率
    rate = rospy.Rate(10)  # 10Hz

    global obstacle_data, vehicle_data

    while not rospy.is_shutdown():
        if obstacle_data is not None and vehicle_data is not None:
            # 整合数据
            merged_payload = []

            for obs in obstacle_data.payload:
                # 将障碍物数据处理成车辆形式（假设简单映射逻辑）
                vehicle = Vehicle()
                vehicle.id = obs.id
                vehicle.x = obs.x
                vehicle.y = obs.y
                vehicle.speed = (obs.vx ** 2 + obs.vy ** 2) ** 0.5
                vehicle.yaw = 0.0  # 假设障碍物没有方向，设置为0
                vehicle.status = 0  # 默认状态为0
                merged_payload.append(vehicle)

            for veh in vehicle_data.payload:
                # 将车辆数据直接加入
                merged_payload.append(veh)

            # 创建并发布消息
            output_msg = VehicleArray()
            output_msg.header = vehicle_data.header  # 使用车辆数据的header
            output_msg.payload = merged_payload

            pub.publish(output_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        # 检查是否使用ROS bag文件
        rospy.init_node('topic_publisher', anonymous=True)
        use_bag_file = rospy.get_param('~use_bag_file', False)
        bag_path = rospy.get_param('~bag_path', bag_path)

        if use_bag_file:
            # 如果使用bag文件，启动一个线程读取bag数据
            bag_thread = threading.Thread(target=read_bag_file)
            bag_thread.start()

        # 启动发布节点
        publisher_node()

    except rospy.ROSInterruptException:
        pass
