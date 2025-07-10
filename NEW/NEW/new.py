import cv2
import numpy as np
import serial
import time

# 初始化串口连接
try:
    ser = serial.Serial('COM10', 115200, timeout=1)
    print(f"Serial port {ser.name} opened successfully")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    ser = None

# 颜色编码映射
COLOR_CODES = {
    "red": '1',
    "yellow": '2',
    "green": '3'
}

# 定义HSV颜色范围（所有颜色都使用列表格式）
COLOR_RANGES = {
    "red": [
        ([0, 100, 100], [10, 255, 255]),  # 红色范围1 (0-10)
        ([170, 100, 100], [180, 255, 255])  # 红色范围2 (170-180)
    ],
    "yellow": [
        ([20, 100, 100], [30, 255, 255])  # 黄色范围
    ],
    "green": [
        ([40, 100, 100], [80, 255, 255])  # 绿色范围
    ]
}

# 预处理内核
kernel = np.ones((5, 5), np.uint8)

# 存储每个颜色的状态
color_status = {}

COOLDOWN_PERIOD = 60
# 稳定检测参数
STABILITY_FRAMES = 35  # 需要连续检测到的帧数
MAX_FRAMES_WITHOUT_DETECTION = 100  # 最大允许丢失检测的帧数
POSITION_THRESHOLD = 10  # 位置变化阈值（像素）


def format_coordinates(x, y):
    """将坐标格式化为6位字符串，前3位X，后3位Y"""
    return f"{x:03d}{y:03d}"


def detect_colors(frame):
    # 转换为HSV颜色空间
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 创建组合掩膜
    combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    # 为每种颜色创建掩膜
    masks = {}
    for color, range_list in COLOR_RANGES.items():
        color_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for (lower, upper) in range_list:
            mask_part = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
            color_mask = cv2.bitwise_or(color_mask, mask_part)

        # 形态学操作
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
        masks[color] = color_mask
        combined_mask = cv2.bitwise_or(combined_mask, color_mask)

    # 查找轮廓
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 按颜色分组检测结果，只保留每个颜色中面积最大的物体
    color_detections = {}
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # 过滤小面积噪声
            # 计算边界框
            x, y, w, h = cv2.boundingRect(contour)

            # 确定颜色
            detected_color = None
            for color, mask in masks.items():
                if np.any(mask[y:y + h, x:x + w]):
                    detected_color = color
                    break

            if detected_color is None:
                continue

            # 计算中心点
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])

                # 保存检测结果（只保留每个颜色中面积最大的物体）
                if detected_color not in color_detections or area > color_detections[detected_color]["area"]:
                    color_detections[detected_color] = {
                        "center": (cx, cy),
                        "area": area,
                        "rect": (x, y, w, h)
                    }

    # 绘制检测结果并生成最终检测列表
    detections = []
    for color, detection in color_detections.items():
        cx, cy = detection["center"]
        x, y, w, h = detection["rect"]

        # 绘制边界框和中心点
        color_values = {
            "red": (0, 0, 255),
            "yellow": (0, 255, 255),
            "green": (0, 255, 0)
        }
        cv2.rectangle(frame, (x, y), (x + w, y + h), color_values.get(color, (0, 0, 0)), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
        cv2.putText(frame, color, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(frame, f"{cx}, {cy}", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(frame, f"Area: {detection['area']}", (x, y + h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 在中心点上方显示颜色编码
        color_code = COLOR_CODES.get(color, '0')
        cv2.putText(frame, f"Code: {color_code}", (cx - 30, cy - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        detections.append({
            "color": color,
            "center": (cx, cy),
            "area": detection["area"],
            "color_code": color_code
        })

    return frame, detections


# 尝试打开摄像头
try:
    # 使用本地摄像头
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Could not open camera")
        exit()
    else:
        print("Camera opened successfully")

        # 设置摄像头分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
except Exception as e:
    print(f"Camera initialization error: {e}")
    exit()

# 初始化FPS计数器
prev_time = time.time()
fps = 0

# 初始化颜色状态
for color in COLOR_RANGES.keys():
    color_status[color] = {
        "state": "idle",  # 状态: idle(空闲), tracking(跟踪中), stable(稳定), cooldown(冷却)
        "last_position": None,  # 上一帧位置
        "stable_count": 0,  # 稳定帧计数
        "sent": False,  # 是否已发送
        "last_sent_time": 0,  # 上次发送时间
        "frames_missing": 0  # 连续丢失帧数
    }

while True:
    # 读取摄像头帧
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame")
        break

    # 计算FPS
    current_time = time.time()
    fps = 0.9 * fps + 0.1 * (1 / (current_time - prev_time))
    prev_time = current_time

    # 在画面上显示FPS
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 进行颜色识别
    result_frame, detections = detect_colors(frame)

    # 创建检测到的颜色字典
    detected_colors = {}
    for d in detections:
        detected_colors[d["color"]] = {
            "center": d["center"],
            "color_code": d["color_code"]
        }

    # 处理颜色状态更新
    for color, state in color_status.items():
        if color in detected_colors:
            # 重置丢失帧计数器
            state["frames_missing"] = 0

            # 获取当前位置和颜色编码
            current_pos = detected_colors[color]["center"]
            color_code = detected_colors[color]["color_code"]

            # 状态机处理
            if state["state"] == "idle":
                # 开始跟踪新物体
                state["state"] = "tracking"
                state["last_position"] = current_pos
                state["stable_count"] = 0

            elif state["state"] == "tracking":
                # 检查位置稳定性
                if state["last_position"] is not None:
                    dx = abs(current_pos[0] - state["last_position"][0])
                    dy = abs(current_pos[1] - state["last_position"][1])

                    # 如果位置变化在阈值内，增加稳定计数
                    if dx < POSITION_THRESHOLD and dy < POSITION_THRESHOLD:
                        state["stable_count"] += 1

                        # 在画面上显示稳定状态
                        cv2.putText(result_frame, f"{color} stable: {state['stable_count']}/{STABILITY_FRAMES}",
                                    (10, 60 + 30 * list(COLOR_RANGES.keys()).index(color)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

                        # 达到稳定帧数要求
                        if state["stable_count"] >= STABILITY_FRAMES:
                            state["state"] = "stable"
                    else:
                        # 位置变化太大，重置稳定计数
                        state["stable_count"] = 0

                # 更新上一帧位置
                state["last_position"] = current_pos

            elif state["state"] == "stable":
                # 稳定状态下发送坐标
                if not state["sent"] and ser is not None and ser.is_open:
                    # 发送格式: 颜色码 + 坐标 (共7位)
                    cx, cy = current_pos
                    data_to_send = color_code + format_coordinates(cx, cy) + "\n"

                    try:
                        ser.write(data_to_send.encode())
                        print(f"Sent: {data_to_send.strip()} (Color: {color})")
                        state["sent"] = True
                        state["last_sent_time"] = current_time
                        state["state"] = "cooldown"  # 进入冷却状态
                    except serial.SerialException as e:
                        print(f"Serial write error: {e}")
                        ser = None  # 标记串口连接失败

            elif state["state"] == "cooldown":
                # 检查冷却时间是否结束
                if current_time - state["last_sent_time"] > COOLDOWN_PERIOD:
                    state["state"] = "idle"
                    state["sent"] = False
        else:
            # 未检测到该颜色
            state["frames_missing"] += 1

            # 如果连续多帧未检测到，重置状态（不输出日志）
            if state["frames_missing"] > MAX_FRAMES_WITHOUT_DETECTION:
                if state["state"] != "cooldown":  # 冷却状态保持
                    state["state"] = "idle"
                    state["stable_count"] = 0

    # 显示结果帧
    cv2.imshow("Color Detection", result_frame)

    # 按下 'q' 键退出循环
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
    elif key & 0xFF == ord(' '):  # 空格键暂停/继续
        cv2.waitKey(0)
    elif key & 0xFF == ord('r'):  # 按'r'键重置所有颜色的发送状态
        for color in color_status:
            color_status[color] = {
                "state": "idle",
                "last_position": None,
                "stable_count": 0,
                "sent": False,
                "last_sent_time": 0,
                "frames_missing": 0
            }
        print("Reset all color states")

# 释放资源
cap.release()
cv2.destroyAllWindows()
if ser is not None and ser.is_open:
    ser.close()
    print("Serial port closed")