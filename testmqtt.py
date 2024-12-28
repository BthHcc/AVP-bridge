import paho.mqtt.client as mqtt
import json
import time
import threading

# MQTT 配置
MQTT_BROKER = "127.0.0.1" # 10.180.210.20
MQTT_PORT = 1883
MQTT_USERNAME = "avp_manager"
MQTT_PASSWORD = "123456"
client_id = "avp_manager_id_test01"
seq_ips = 0
seq_pv_cmd_online = 0

# 要发布的主题
topic_perception_vehicles = "/perception/vehicles"
topic_pv_cmd_online = "/pv/cmd/online" # 车辆在中心云上线

# 标志变量：控制 /pv/cmd/online 的发布
pv_cmd_online_published = False

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("成功连接到 MQTT 代理")
        client.subscribe(topic_perception_vehicles)
        client.subscribe(topic_pv_cmd_online)

def publish_vehicle_data(client):
    global seq_ips, pv_cmd_online_published

    # 构造 /perception/vehicles 数据
    vehicle = {
        "LtoRear": 0.85,
        "id": 0,
        "l": 4.6,
        "rz": 0.0,
        "speed": 0.0,
        "status": 1,
        "steer": 0.0,
        "w": 1.9,
        "wb": 2.73,
        "x": 39, # 40  1.45 -131.0 || SEIEE AVP: 39
        "y": -20, # -145.5  1.0 40.0|| SEIEE AVP: -20
        "yaw": 108 # 0.0 -90.0 || SEIEE AVP: 108
    }

    vobj = {
        "payload": [vehicle],
        "sender": "hc001",
        "seq": seq_ips,
        "time": round(time.time(), 3),
    }
    seq_ips += 1

    vdoc = json.dumps(vobj, indent=4)
    result = client.publish(topic_perception_vehicles, vdoc)
    print(f"发布的消息到 {topic_perception_vehicles}: {vdoc}, 结果: {result.rc}")

    # 在 /perception/vehicles 发布后第一次触发 /pv/cmd/online
    if not pv_cmd_online_published:
        publish_pv_cmd_online(client)
        pv_cmd_online_published = True

    # 每 0.1 秒定时发布
    threading.Timer(0.1, publish_vehicle_data, [client]).start()

def publish_pv_cmd_online(client):
    global seq_pv_cmd_online

    # 构造 /pv/cmd/online 数据
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
    seq_pv_cmd_online += 1

    cmd_doc = json.dumps(obj, indent=4)
    result = client.publish(topic_pv_cmd_online, cmd_doc)
    print(f"发布的消息到 {topic_pv_cmd_online}: {cmd_doc}, 结果: {result.rc}")

def main():
    client = mqtt.Client(client_id=client_id)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"连接到代理失败: {e}")

    client.loop_start()
    publish_vehicle_data(client)  # 开始 /perception/vehicles 的发布线程

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
