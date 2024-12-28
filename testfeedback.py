import sys
import paho.mqtt.client as mqtt
import json
import time
import random
from PyQt5.QtCore import QTimer, QCoreApplication
import signal

# MQTT 配置
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_USERNAME = "avp_manager"
MQTT_PASSWORD = "123456"
client_id_feedback = "avp_manager_id_test03"

# 要发布的主题
topic_vehicle_feedback = "/pv/ind/vehicle_feedback/testvin011/111"

# 要订阅的主题
topic_request_switch_mode = "/ec/req/switch_driving_mode/testvin011/111"
topic_request_change_gear = "/ec/cmd/change_gear/testvin011/111"
topic_vehicle_control = "/ec/cmd/vehicle_control/testvin011/111"

requested_mode = 0 # 车辆模式
POIid = 0 # 全局车位信息/接驳区域
gear = 0 # 车辆档位
steer = 0.0
speed = 0.0


# MQTT 消息处理回调函数
def on_message(client, userdata, msg):
    global requested_mode, POIid, gear, steer, speed  # 全局变量声明

    received_obj = json.loads(msg.payload.decode())

    if msg.topic == topic_request_switch_mode:
        payload = received_obj.get("payload", {})
        requested_mode = payload.get("requested_mode", 0)
        POIid = payload.get("POIid", 0)
        print(f"收到新的请求模式: {requested_mode}, 接收到的泊车区域：{POIid}")

    if msg.topic == topic_request_change_gear:
        payload = received_obj.get("payload", {})
        control_gear = payload.get("gear", 0)  # 控制档位
        # 映射控制档位到反馈档位
        gear = map_control_to_feedback_gear(control_gear)
        print(f"收到新的请求档位 (控制): {control_gear} -> (反馈): {gear}")

    if msg.topic == topic_vehicle_control:
        payload = received_obj.get("payload", {})
        steer = payload.get("steer", 0)
        speed = payload.get("speed", 0)
        print(f"车辆steer控制: {steer}, 车辆speed控制: {speed}")

# 控制档位与反馈档位的映射函数
def map_control_to_feedback_gear(control_gear):
    """
    将控制档位映射为反馈档位
    """
    if control_gear == 0:  # N档
        return 2
    elif control_gear == 1:  # D档
        return 1
    elif control_gear == 2:  # P档
        return 4
    elif control_gear == 3:  # R档
        return 3
    else:  # invalid
        return 0

# 发送车辆反馈信息的函数
def send_vehicle_feedback(client):
    # global requested_mode  # 全局车辆模式
    # global POIid  # 全局车位信息/接驳区域
    # global gear # 全局车辆档位 11111
    # global steer, speed # 全局车辆控制信息

    timestamp_nano = int(time.time() * 1e9)
    obj_header = {
        "seq": 0,
        "time": timestamp_nano,
        "sender": "hc003",
        "model_version": "1.0.0"
    }
    obj_payload = {
        "vin": "testvin011",
        "lpn": "AB12311",
        "driving_mode": requested_mode,  # 使用接收到的 requested_mode
        "status": 0,
        "task": 6,
        "POIid": POIid,
        "speed": speed,
        "acc": -9.8,
        "torque": 0.0,
        "steer": steer,
        "gear": gear,
        "blink": 0,
        "epb": False,
        "battery": 100,
        "lat": 31.1,
        "lon": 121.5,
        "delay_to_broker": random.uniform(0, 100)
    }

    obj = {
        "header": obj_header,
        "payload": obj_payload
    }
    # 发送 MQTT 消息
    result = client.publish(topic_vehicle_feedback, json.dumps(obj))
    # print(f"发送反馈消息 MQTT: 话题: {topic_vehicle_feedback}, 结果: {obj}")

# PyQt5 应用类
class MyApp:
    def __init__(self, client):
        # 创建一个 QTimer 实例
        self.feedback_timer = QTimer()

        # 连接定时器的timeout信号到槽函数
        self.feedback_timer.timeout.connect(lambda: send_vehicle_feedback(client))

        # 设置定时器间隔为100ms (10Hz)
        self.feedback_timer.setInterval(100)

        # 启动定时器
        self.feedback_timer.start()

# 主函数
def main():
    # 初始化 MQTT 客户端
    client = mqtt.Client(client_id=client_id_feedback)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    # 设置 MQTT 消息处理回调函数
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"连接到代理失败: {e}")
        return

    # 订阅话题
    client.subscribe(topic_request_switch_mode)
    client.subscribe(topic_request_change_gear)
    client.subscribe(topic_vehicle_control)
    # 启动 MQTT 客户端的循环
    client.loop_start()

    # 启动 PyQt5 应用
    app = QCoreApplication(sys.argv)
    my_app = MyApp(client)

    # 使用 signal 捕获 Ctrl+C
    signal.signal(signal.SIGINT, lambda signal, frame: exit_gracefully(app, client))

    try:
        # 启动 PyQt5 事件循环
        app.exec_()
    except KeyboardInterrupt:
        print("捕获到中断信号，退出程序...")
    finally:
        # 确保退出时停止 MQTT 客户端循环和断开连接
        client.loop_stop()
        client.disconnect()
        print("程序已停止.")

def exit_gracefully(app, client):
    print("退出ing")
    client.loop_stop()  # 停止 MQTT 客户端的循环
    client.disconnect()  # 断开与 MQTT 代理的连接
    app.quit()  # 退出 PyQt5 应用
    print("程序已停止.")
    sys.exit(0)

if __name__ == "__main__":
    main()