import paho.mqtt.client as mqtt
import json
import time
import threading
import queue

# MQTT 配置
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_USERNAME = "avp_manager"
MQTT_PASSWORD = "123456"
client_id_main = "avp_manager_id_test02"

# 要发布的主题
topic_vehicle_entry = "/cc/res/vehicle_entry/966658472/test"
topic_response_switch_mode = "/pv/res/switch_driving_mode/testvin011/111"

# 要订阅的主题
topic_vehicle_request = "/ec/req/vehicle_entry/966658472/test"
topic_request_switch_mode = "/ec/req/switch_driving_mode/testvin011/111"

# 独立的发布队列
switch_mode_queue = queue.Queue()
vehicle_entry_queue = queue.Queue()

# 发送车辆进入响应信息的函数
def send_mqtt_vin(platenumber):
    payload = {
        "pid": "966658472",
        "avp_enabled": True,
        "vin": f"testvin0{platenumber[-2:]}",
        "lpn": platenumber,
        "vkey": "111",
        "make": "Hycan_sim",
        "model": "Z03-apa",
        "length": 4.6,
        "width": 1.9,
        "wheel_base": 2.73,
        "length_to_rear": 0.85,
        "steer_ratio": 15.0
    }
    obj = {
        "name": "mini_app",
        "seq": 0,
        "time": time.time(),
        "sender": "hc002",
        "payload": payload
    }
    vehicle_entry_queue.put((topic_vehicle_entry, json.dumps(obj)))

# 发送驾驶模式切换响应的函数
def send_res_switch_driving_mode(mode, vin):
    timestamp_nano = int(time.time() * 1e9)
    obj_header = {
        "seq": 0,
        "time": timestamp_nano,
        "sender": "VP80012345",
        "model_version": "1.0.0"
    }
    obj_payload = {
        "vin": vin,
        "driving_mode": mode,
        "requested_mode": mode,
        "result": True,
        "error": ""
    }
    obj = {
        "header": obj_header,
        "payload": obj_payload
    }
    switch_mode_queue.put((topic_response_switch_mode, json.dumps(obj)))

# 独立处理队列的线程
def process_vehicle_entry_queue(client):
    while True:
        topic, message = vehicle_entry_queue.get()
        result = client.publish(topic, message)
        print(f"发送车辆进入消息 MQTT: 话题: {topic}, 结果: {result.rc}")
        vehicle_entry_queue.task_done()

def process_switch_mode_queue(client):
    while True:
        topic, message = switch_mode_queue.get()
        result = client.publish(topic, message)
        print(f"发送切换模式消息 MQTT: 话题: {topic}, 结果: {result.rc}")
        switch_mode_queue.task_done()

# 连接回调函数
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(topic_vehicle_request)  # 订阅车辆进入请求主题
    client.subscribe(topic_request_switch_mode)  # 订阅驾驶模式切换请求主题

# 消息接收回调函数
def on_message(client, userdata, msg):
    print(f"接收到消息: 主题: {msg.topic}, 消息: {msg.payload.decode()}")
    received_obj = json.loads(msg.payload.decode())
    
    if msg.topic == topic_vehicle_request:
        send_mqtt_vin("AB12311")
    elif msg.topic == topic_request_switch_mode:
        payload = received_obj.get("payload", {})
        mode = payload.get("requested_mode", 0)
        vin = payload.get("vin", "testvin011")
        send_res_switch_driving_mode(mode, vin)

# 主函数
def main():
    client = mqtt.Client(client_id=client_id_main)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message  # 设置消息回调

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"连接到代理失败: {e}")

    # 启动独立的队列处理线程
    threading.Thread(target=process_vehicle_entry_queue, args=(client,), daemon=True).start()
    threading.Thread(target=process_switch_mode_queue, args=(client,), daemon=True).start()

    client.loop_start()  # 主客户端循环

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
