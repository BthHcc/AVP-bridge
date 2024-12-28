# 第一组数据
group1 = [
    {"x": 121.42335704403375, "y": 31.016246242030537},
    {"x": 121.42335699220361, "y": 31.016296337585793},
    {"x": 121.4233281925672, "y": 31.016296315480041},
    {"x": 121.4233282444124, "y": 31.016246219924795}
]

# 第二组数据
group2 = [
    {"x": 121.4232333399973, "y": 31.016200762773266},
    {"x": 121.42323310913375, "y": 31.016140902542503},
    {"x": 121.42325791346835, "y": 31.016140773569774},
    {"x": 121.42325814433195, "y": 31.016200633816027}
]

# 计算矩形的边长
def calculate_rectangle_dimensions(group):
    # 获取 x 和 y 的最小值和最大值
    x_coords = [point['x'] for point in group]
    y_coords = [point['y'] for point in group]
    
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)
    
    # 计算矩形的宽度和高度
    width = x_max - x_min
    height = y_max - y_min
    
    return width, height

# 计算两组数据的矩形边长
width1, height1 = calculate_rectangle_dimensions(group1)
width2, height2 = calculate_rectangle_dimensions(group2)

# 输出结果
print(f"第一组数据构成的矩形边长：宽 = {width1:.6f}, 高 = {height1:.6f}")
print(f"第二组数据构成的矩形边长：宽 = {width2:.6f}, 高 = {height2:.6f}")
