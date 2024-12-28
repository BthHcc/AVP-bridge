import xml.etree.ElementTree as ET

# 读取原始角点数据（可以从XML文件中读取，下面是手动输入的示例）
data = [
    {"x": 1.2142321563213495e+02, "y": 3.1016158481439852e+01, "z": 1.6792297363281250e-02},
    {"x": 1.2142327549236572e+02, "y": 3.1016158250576289e+01, "z": 1.6792297363281250e-02},
    {"x": 1.2142327562133845e+02, "y": 3.1016183054910890e+01, "z": 1.6792297363281250e-02},
    {"x": 1.2142321576109219e+02, "y": 3.1016183285774510e+01, "z": 1.6792297363281250e-02}
]

# 计算中心点
def calculate_center(data):
    x_center = sum([point['x'] for point in data]) / len(data)
    y_center = sum([point['y'] for point in data]) / len(data)
    return {"x": x_center, "y": y_center}

# 旋转90度（绕Z轴）并返回新的坐标
def rotate_point(x, y, center_x, center_y):
    x_new = center_x + (y - center_y)
    y_new = center_y - (x - center_x)
    return {"x": x_new, "y": y_new}

# 计算旋转后的四个角点
def rotate_corners(data):
    center = calculate_center(data)
    rotated_data = []
    for point in data:
        rotated_point = rotate_point(point['x'], point['y'], center['x'], center['y'])
        rotated_point['z'] = point['z']  # 保持z坐标不变
        rotated_data.append(rotated_point)
    return rotated_data

# 输出旋转后的结果为XML格式
def output_xml(data):
    for point in data:
        print(f'<cornerGlobal x="{point["x"]:.16e}" y="{point["y"]:.16e}" z="{point["z"]:.16e}"/>')

# 旋转角点并输出结果
rotated_corners = rotate_corners(data)
output_xml(rotated_corners)
