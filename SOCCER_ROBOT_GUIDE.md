# TurboPI Soccer Robot - Key Functions Guide

This guide highlights the main functions and components you'll need for building an autonomous soccer-playing robot.

---

## 📋 Table of Contents
1. [Motor Control](#motor-control)
2. [Ultrasonic Distance Detection](#ultrasonic-distance-detection)
3. [Image Detection (YOLO)](#image-detection-yolo)
4. [Object Tracking](#object-tracking)
5. [PID Control](#pid-control)
6. [Autonomous Navigation](#autonomous-navigation)
7. [Key ROS2 Topics](#key-ros2-topics)

---

## 🚗 Motor Control

### Primary Motor Control Files
- **File**: `src/peripherals/peripherals/joystick_control.py`
- **File**: `src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py`
- **SDK**: `src/driver/sdk/sdk/mecanum.py`

### Key Functions

#### 1. **Direct Motor Speed Control** (ROS2)
```python
# Publish to: '/ros_robot_controller/set_motor_speeds'
# Message Type: MotorsSpeedControl

motor_speed_msg = MotorsSpeedControl()
motor_speed_msg.data = [
    MotorSpeedControl(id=1, speed=-v1),  # Motor 1 (reverse speed)
    MotorSpeedControl(id=2, speed=v2),   # Motor 2
    MotorSpeedControl(id=3, speed=-v3),  # Motor 3 (reverse speed)
    MotorSpeedControl(id=4, speed=v4)    # Motor 4
]
self.motor_pub.publish(motor_speed_msg)
```

#### 2. **Mecanum Wheel Control** (Omnidirectional Movement)
```python
# From: src/driver/sdk/sdk/mecanum.py

class MecanumChassis:
    def set_velocity(self, velocity, direction, angular_rate, fake=False):
        """
        Use polar coordinates to control moving
        :param velocity: mm/s
        :param direction: Moving direction 0~360deg, 180deg<--- ↑ ---> 0deg
        :param angular_rate: The speed at which the chassis rotates
        """
        rad_per_deg = math.pi / 180
        vx = velocity * math.cos(direction * rad_per_deg)
        vy = velocity * math.sin(direction * rad_per_deg)
        vp = -angular_rate * (self.a + self.b)
        
        v1 = int(vy + vx - vp) 
        v2 = int(vy - vx + vp)
        v3 = int(vy - vx - vp)
        v4 = int(vy + vx + vp)
        
        board.set_motor_duty([[1, -v1], [2, v2], [3, -v3], [4, v4]])
    
    def translation(self, velocity_x, velocity_y, fake=False):
        """
        Translate using X/Y velocity components
        """
        velocity = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
        # Calculate direction from velocity components
        if velocity_x == 0:
            direction = 90 if velocity_y >= 0 else 270
        else:
            direction = math.atan(velocity_y / velocity_x) * 180 / math.pi
            if velocity_x < 0:
                direction += 180
            elif velocity_y < 0:
                direction += 360
        return self.set_velocity(velocity, direction, 0)
```

#### 3. **Twist Control** (Standard ROS2 Control)
```python
# Publish to: '/cmd_vel'
# Message Type: Twist

twist = Twist()
twist.linear.x = forward_speed    # Forward/backward movement
twist.linear.y = strafe_speed     # Left/right (Mecanum only)
twist.angular.z = rotation_speed  # Rotation
self.mecanum_pub.publish(twist)
```

#### 4. **Low-Level Motor Control** (SDK)
```python
# From: src/driver/sdk/sdk/ros_robot_controller_sdk.py

board = Board()

# Set motor speeds (float values)
board.set_motor_speed([[1, speed1], [2, speed2], [3, speed3], [4, speed4]])

# Set motor duty cycle
board.set_motor_duty([[1, duty1], [2, duty2], [3, duty3], [4, duty4]])
```

### Soccer Application
- **Chase ball**: Forward motion using `twist.linear.x`
- **Strafe to position**: Use `twist.linear.y` for side movement
- **Orient toward goal**: Use `twist.angular.z` for rotation
- **Quick response**: Use direct motor control for fastest reactions

---

## 🔊 Ultrasonic Distance Detection

### Primary File
- **File**: `src/peripherals/peripherals/sonar_controller_node.py`
- **SDK**: `src/driver/sdk/sdk/sonar.py`

### Key Functions

#### 1. **Get Distance Reading**
```python
# From: src/peripherals/peripherals/sonar_controller_node.py

class SonarController(Node):
    def __init__(self, name):
        self.sonar = Sonar()
        self.distance_pub = self.create_publisher(Int32, '~/get_distance', 1)
    
    def pub_callback(self):
        while self.running:
            data = self.sonar.getDistance()  # Returns distance in mm
            msg = Int32()
            msg.data = data
            self.distance_pub.publish(msg)
```

#### 2. **Subscribe to Distance Data**
```python
# Subscribe to: 'sonar_controller/get_distance'
# Message Type: Int32

def distance_callback(self, msg):
    distance_cm = msg.data / 10.0  # Convert mm to cm
    
    if distance_cm < threshold:
        # Object detected close by
        pass
```

#### 3. **Low-Level Distance Reading** (SDK)
```python
# From: src/driver/sdk/sdk/sonar.py
# Hiwonder iic ultrasonic library (幻尔科技iic超声波库)

class Sonar:
    def getDistance(self):
        """
        Returns distance in millimeters
        Reading range typically 20-4000mm
        """
        dist = 99999
        try:
            with SMBus(self.i2c) as bus:
                msg = i2c_msg.read(self.i2c_addr, 2)
                bus.i2c_rdwr(msg)
                dist = int.from_bytes(bytes(list(msg)), 'big')
        except:
            pass
        return dist
```

### Soccer Application
- **Detect ball distance**: Know when ball is close enough to kick
- **Wall/boundary detection**: Avoid going out of bounds
- **Opponent detection**: Detect when opponent robot is nearby
- **Goal positioning**: Determine distance to goal

---

## 📷 Image Detection (YOLO)

### Primary Files
- **File**: `src/yolov11_detect/yolov11_detect/yolov11_node.py`
- **File**: `src/yolov11_detect/yolov11_detect/yolov11_detect.py`

### Key Functions

#### 1. **YOLO Detection Node**
```python
# From: src/yolov11_detect/yolov11_detect/yolov11_node.py

class yoloNode(Node):
    def __init__(self, name):
        # Load YOLO model
        MODEL_PATH = f'/home/ubuntu/ros2_ws/src/yolov11_detect/models/{model_name}.xml'
        self.net = self.core.compile_model(MODEL_PATH, device_name="AUTO")
        
        # Publishers
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
    
    def image_proc(self):
        """Process images and detect objects"""
        while self.running:
            bgr_image = self.image_queue.get()
            
            # Run YOLO detection
            input_tensor = self.preprocess_image(bgr_image)
            self.ir.infer([input_tensor])
            results = self.ir.get_output_tensor(0).data
            
            # Parse detections
            detections = self.postprocess(results, bgr_image.shape)
            
            # Publish detected objects
            objects_msg = ObjectsInfo()
            for det in detections:
                obj = ObjectInfo()
                obj.class_name = self.classes[det['class_id']]
                obj.box = det['box']  # [x, y, width, height]
                obj.score = det['confidence']
                objects_msg.objects.append(obj)
            
            self.object_pub.publish(objects_msg)
```

#### 2. **Object Classes**
```python
# YOLO can detect these classes (you can train for custom objects like "soccer_ball"):
classes = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    # ... more classes
]
```

#### 3. **Subscribe to Detections**
```python
# Subscribe to: 'yolov11_node/object_detect'
# Message Type: ObjectsInfo

def object_detect_callback(self, msg):
    for obj in msg.objects:
        if obj.class_name == "sports ball":  # Soccer ball
            x, y, w, h = obj.box.x, obj.box.y, obj.box.width, obj.box.height
            confidence = obj.score
            
            # Calculate ball position in image
            ball_center_x = x + w / 2
            ball_center_y = y + h / 2
            
            # Use this to control robot
```

### Soccer Application
- **Ball detection**: Detect "sports ball" class
- **Goal detection**: Train custom model to detect goals
- **Opponent detection**: Detect other robots
- **Custom training**: Train YOLO on specific soccer ball colors/types

---

## 🎯 Object Tracking

### Primary File
- **File**: `src/app/app/tracking.py`

### Key Functions

#### 1. **Color-Based Tracking**
```python
# From: src/app/app/tracking.py
# Color tracking + pan-tilt tracking + chassis tracking + RGB light control
# (颜色跟踪 + 云台追踪 + 车体追踪 + RGB灯控制)

class ObjectTracker:
    def __init__(self, color, node, set_color, set_status=False):
        # PID for servo control (云台PID参数)
        self.servo_x_pid = pid.PID(P=0.25, I=0.05, D=0.009)
        self.servo_y_pid = pid.PID(P=0.25, I=0.05, D=0.009)
        
        # Servo parameters (舵机参数)
        self.servo_x = 1500
        self.servo_y = 1500
        
    def update_pid(self, x, y, img_w, img_h):
        """
        Update servo PID, control servo position, and return PID output for robot movement
        (更新云台PID，控制舵机位置，并返回PID输出用于小车运动)
        """
        # X-axis PID (horizontal direction) (X轴PID（水平方向）)
        self.servo_x_pid.SetPoint = img_w / 2
        self.servo_x_pid.update(x)
        servo_x_output = int(self.servo_x_pid.output)
        self.servo_x += servo_x_output
        
        # Y-axis PID (vertical direction) (Y轴PID（垂直方向）)
        self.servo_y_pid.SetPoint = img_h / 2
        self.servo_y_pid.update(y)
        servo_y_output = int(self.servo_y_pid.output)
        self.servo_y -= servo_y_output
        
        return self.servo_x, self.servo_y, self.servo_x_pid.output, self.servo_y_pid.output
    
    def __call__(self, image, result_image, threshold):
        """
        Process image, detect target and return result
        (处理图像，检测目标并返回结果)
        """
        # Convert to LAB color space
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
        image = cv2.GaussianBlur(image, (5, 5), 5)
        
        # Create mask based on color threshold
        mask = cv2.inRange(image, tuple(min_color), tuple(max_color))
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        
        # Find contours
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        
        # Find largest contour (target object)
        if len(contour_area) > 0:
            contour, area = max(contour_area, key=lambda c_a: c_a[1])
            circle = cv2.minEnclosingCircle(contour)
            (x, y), r = circle
            target_pos = (x, y)
            target_radius = r
        
        return result_image, target_pos, target_radius
```

#### 2. **Chassis Following** (for tracked objects)
```python
class OjbectTrackingNode(Node):
    def __init__(self, name):
        # Chassis PID (底盘)
        self.pid_yaw = pid.PID(0.008, 0.003, 0.0001)  # Chassis Yaw axis PID (底盘Yaw轴PID)
        self.pid_dist = pid.PID(0.004, 0.003, 0.00001)  # Chassis distance PID (底盘距离PID)
    
    def chassis_tracking(self, target_x, target_y, target_radius):
        """Move robot to follow tracked object"""
        # Calculate error from center
        error_x = target_x - (image_width / 2)
        error_y = target_y - (image_height / 2)
        
        # Update PIDs
        self.pid_yaw.SetPoint = 0
        self.pid_yaw.update(error_x)
        
        self.pid_dist.SetPoint = desired_radius
        self.pid_dist.update(target_radius)
        
        # Control robot
        twist = Twist()
        twist.angular.z = self.pid_yaw.output  # Rotate to center object
        twist.linear.x = self.pid_dist.output  # Move forward/back to maintain distance
        self.mecanum_pub.publish(twist)
```

### Soccer Application
- **Track orange/white ball**: Use color tracking to follow soccer ball
- **Center ball in view**: Use PID to keep ball centered
- **Maintain optimal distance**: Keep ball at kicking distance
- **Combine with YOLO**: Use both color and shape detection

---

## 🎮 PID Control

### Primary File
- **SDK**: `src/driver/sdk/sdk/pid.py`

### Key Functions

```python
# From: src/driver/sdk/sdk/pid.py

class PID:
    """PID Controller"""
    
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P  # Proportional gain
        self.Ki = I  # Integral gain
        self.Kd = D  # Derivative gain
        self.SetPoint = 0.0
        self.output = 0.0
    
    def update(self, feedback_value):
        """
        Calculates PID value for given reference feedback
        u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de/dt
        """
        error = self.SetPoint - feedback_value
        
        # Proportional term
        self.PTerm = self.Kp * error
        
        # Integral term (with windup guard)
        self.ITerm += error * delta_time
        if self.ITerm < -self.windup_guard:
            self.ITerm = -self.windup_guard
        elif self.ITerm > self.windup_guard:
            self.ITerm = self.windup_guard
        
        # Derivative term
        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = (error - self.last_error) / delta_time
        
        # Calculate output
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    
    def setKp(self, proportional_gain):
        """Set Proportional Gain"""
        self.Kp = proportional_gain
    
    def setKi(self, integral_gain):
        """Set Integral Gain"""
        self.Ki = integral_gain
    
    def setKd(self, derivative_gain):
        """Set Derivative Gain"""
        self.Kd = derivative_gain
```

### Soccer Application
- **Ball centering**: PID to keep ball centered in camera view
- **Distance control**: PID to maintain optimal distance to ball
- **Heading control**: PID to orient robot toward goal
- **Speed smoothing**: PID for smooth acceleration/deceleration

---

## 🤖 Autonomous Navigation

### Primary File
- **File**: `src/app/app/avoidance_node.py`

### Key Functions

#### 1. **Obstacle Avoidance**
```python
# From: src/app/app/avoidance_node.py
# Sonar ros2 package (避障节点)

class AvoidanceNode(Node):
    def __init__(self, name):
        self.threshold = 30  # cm
        self.speed = 0.4
        self.turn = True
        self.forward = True
        
        # Subscribe to sonar data
        # (摄像头订阅 - subscribe to the camera)
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        # (超声波订阅 - subscribe to sonar)
        self.sonar_sub = self.create_subscription(Int32, 'sonar_controller/get_distance', self.distance_callback, 10)
        
        # Chassis control publisher (底盘控制 - chassis control)
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)
    
    def distance_callback(self, msg):
        """Handle distance readings and avoid obstacles"""
        self.distance = msg.data
        
        twist = Twist()
        if self.is_running:
            # Check if distance threshold is reached
            # (检测是否达到距离阈值 - check if distance threshold is reached)
            if self.distance / 10.0 <= self.threshold:
                # Turn to avoid obstacle
                twist.angular.z = 11.0
                
                # Implement a check to prevent duplicate commands
                # (做一个判断防止重复发指令)
                if self.turn:
                    self.turn = False
                    self.forward = True
            else:
                # Move forward
                twist.linear.x = float(self.speed)
                twist.angular.z = 0.0
                
                # Implement a check to prevent duplicate commands
                # (做一个判断防止重复发指令)
                if self.forward:
                    self.turn = True
                    self.forward = False
            
            self.mecanum_pub.publish(twist)
```

#### 2. **Service Callbacks**
```python
def enter_srv_callback(self, request, response):
    """Enter the game (进入玩法)"""
    self.get_logger().info('avoidance enter')
    self.reset_value()
    response.success = True
    return response

def exit_srv_callback(self, request, response):
    """Exit the game (退出玩法)"""
    self.get_logger().info('avoidance exit')
    self.is_running = False
    self.mecanum_pub.publish(Twist())  # Stop robot
    response.success = True
    return response

def set_running_srv_callback(self, request, response):
    """Start the game (开启玩法)"""
    self.is_running = request.data
    if not self.is_running:
        self.mecanum_pub.publish(Twist())  # Stop robot
    response.success = True
    return response

def set_parameters_srv_callback(self, request, response):
    """
    Set obstacle avoidance threshold and speed parameters
    (设置避障阈值，速度参数)
    """
    new_threshold, new_speed = request.data
    
    if not 10 <= new_threshold <= 50:
        response.success = False
        return response
    
    self.threshold = new_threshold
    self.speed = round(new_speed / 80, 1)
    response.success = True
    return response
```

### Soccer Application
- **Boundary avoidance**: Stay within field boundaries
- **Obstacle detection**: Avoid walls and other robots
- **Smart navigation**: Combine with ball tracking for intelligent movement
- **Defensive positioning**: Use to maintain position near goal

---

## 📡 Key ROS2 Topics

### Motor Control
```
/ros_robot_controller/set_motor_speeds    # MotorsSpeedControl - Direct motor control
/cmd_vel                                   # Twist - Standard velocity control
```

### Sensors
```
/sonar_controller/get_distance             # Int32 - Distance in mm
/image_raw                                 # Image - Camera feed
/ros_robot_controller/imu_raw              # Imu - IMU data
```

### Vision
```
/yolov11_node/object_detect                # ObjectsInfo - Detected objects
/yolov11_node/object_image                 # Image - Annotated image
```

### Tracking
```
/tracking_node/image_result                # Image - Tracking visualization
```

### Services
```
/avoidance_node/enter                      # Trigger - Start avoidance
/avoidance_node/exit                       # Trigger - Stop avoidance  
/avoidance_node/set_running                # SetBool - Enable/disable
/avoidance_node/set_param                  # SetFloat64List - Set parameters
/yolov11_node/start                        # Trigger - Start YOLO detection
/yolov11_node/stop                         # Trigger - Stop YOLO detection
```

---

## 🏆 Soccer Robot Strategy Examples

### 1. **Basic Ball Chasing**
```python
def soccer_strategy_basic(self):
    """Chase and kick ball"""
    # 1. Detect ball using YOLO or color tracking
    ball_detected, ball_x, ball_y, ball_size = self.detect_ball()
    
    if ball_detected:
        # 2. Center ball in view using PID
        error_x = ball_x - (image_width / 2)
        self.pid_rotation.SetPoint = 0
        self.pid_rotation.update(error_x)
        
        # 3. Maintain optimal kicking distance
        self.pid_distance.SetPoint = optimal_distance
        self.pid_distance.update(ball_size)
        
        # 4. Move robot
        twist = Twist()
        twist.angular.z = self.pid_rotation.output
        twist.linear.x = self.pid_distance.output
        self.cmd_vel_pub.publish(twist)
    else:
        # Search for ball
        self.search_pattern()
```

### 2. **Advanced: Goal-Oriented Play**
```python
def soccer_strategy_advanced(self):
    """Strategic play toward opponent goal"""
    # 1. Detect ball and goal
    ball_pos = self.detect_ball()
    goal_pos = self.detect_goal()
    opponent_distance = self.get_sonar_distance()
    
    if ball_pos and goal_pos:
        # 2. Calculate angle to goal
        angle_to_goal = self.calculate_angle(ball_pos, goal_pos)
        
        # 3. Position robot between ball and goal
        if self.is_aligned_with_goal(angle_to_goal, threshold=15):
            # Kick toward goal
            self.approach_and_kick(ball_pos)
        else:
            # Reposition for better shot angle
            self.circle_ball_toward_goal(ball_pos, goal_pos)
    
    # 4. Avoid obstacles
    if opponent_distance < 30:  # cm
        self.evade_opponent()
```

### 3. **Defensive Strategy**
```python
def soccer_strategy_defense(self):
    """Guard own goal"""
    goal_pos = self.own_goal_position
    ball_pos = self.detect_ball()
    
    if ball_pos:
        # Stay between ball and own goal
        optimal_pos = self.calculate_defensive_position(ball_pos, goal_pos)
        self.move_to_position(optimal_pos)
        
        # If ball is close, clear it
        if self.distance_to_ball(ball_pos) < 20:  # cm
            self.kick_ball_away_from_goal()
```

---

## 🔧 Hardware Specifications

### Motors
- **Type**: 4x Mecanum wheels (omnidirectional)
- **Control**: PWM duty cycle or speed control
- **IDs**: Motors 1-4
- **Direction**: Motors 1 & 3 reversed

### Ultrasonic Sensor
- **Type**: I2C ultrasonic sensor
- **Range**: 20-4000mm (2-400cm)
- **I2C Address**: 0x77
- **Output**: Distance in millimeters

### Camera
- **Topic**: `/image_raw`
- **Type**: RGB camera
- **Use**: YOLO detection, color tracking

### Servos (Pan-Tilt)
- **Type**: PWM servos
- **IDs**: 1-4
- **Range**: typically 500-2500 PWM
- **Use**: Camera gimbal control

---

## 📝 Chinese to English Translations

### Common Terms Found in Code

| Chinese | English | Usage |
|---------|---------|-------|
| 创建一个LaunchDescription对象 | Create a LaunchDescription object | Launch files |
| 加载并设置舵机偏移量从 YAML 文件 | Load and set servo offsets from YAML file | Servo calibration |
| 从 YAML 文件中读取舵机偏差设置 | Read servo deviation settings from YAML file | Configuration |
| 确保config是字典 | Ensure config is a dictionary | Data validation |
| 配置文件格式错误 | Configuration file format error | Error messages |
| 应为字典格式 | Should be dictionary format | Error messages |
| 遍历ID1到ID4并设置偏移量 | Iterate through ID1 to ID4 and set offsets | Servo setup |
| 如果未找到，默认偏移量为0 | If not found, default offset is 0 | Default values |
| 已设置舵机的偏移量为 | Set servo offset to | Success message |
| 设置舵机偏移量时出错 | Error setting servo offset | Error handling |
| 配置文件未找到 | Configuration file not found | File error |
| 解析 YAML 文件时出错 | Error parsing YAML file | Parse error |
| 颜色跟踪 | Color tracking | Tracking |
| 云台追踪 | Pan-tilt tracking | Camera gimbal |
| 车体追踪 | Chassis tracking | Robot body movement |
| 图像处理分辨率 | Image processing resolution | Vision |
| 颜色检测阈值 | Color detection threshold | Parameters |
| 云台PID参数 | Pan-tilt PID parameters | Control |
| 舵机参数 | Servo parameters | Hardware |
| 云台X轴死区 | Pan-tilt X-axis deadzone | Control |
| 云台Y轴死区 | Pan-tilt Y-axis deadzone | Control |
| 更新云台PID，控制舵机位置 | Update pan-tilt PID, control servo position | Control loop |
| 水平方向 | Horizontal direction | Orientation |
| 垂直方向 | Vertical direction | Orientation |
| 处理图像，检测目标并返回结果 | Process image, detect target and return result | Vision pipeline |
| 根据手动或大模型设置的目标颜色 | Based on manual or AI model set target color | Detection |
| 确定阈值范围 | Determine threshold range | Parameters |
| 图像处理：生成掩膜、腐蚀、膨胀、轮廓检测 | Image processing: generate mask, erode, dilate, contour detection | CV operations |
| 检测到有效轮廓时，选择目标 | When valid contour detected, select target | Object selection |
| 绘制圆圈并更新位置 | Draw circle and update position | Visualization |
| 底盘控制 | Chassis control | Motor control |
| 心跳包 | Heartbeat package | System health |
| 进入玩法 | Enter the game/mode | Mode control |
| 退出玩法 | Exit the game/mode | Mode control |
| 开启玩法 | Start the game/mode | Mode control |
| 参数设置 | Parameter setting | Configuration |
| 摄像头订阅 | Subscribe to camera | ROS topics |
| 超声波订阅 | Subscribe to sonar | ROS topics |
| 设置避障阈值，速度参数 | Set obstacle avoidance threshold, speed parameters | Configuration |
| 检测是否达到距离阈值 | Check if distance threshold is reached | Logic |
| 做一个判断防止重复发指令 | Implement a check to prevent duplicate commands | Optimization |
| 通信协议的格式 | Communication protocol format | Protocol |
| 可通过串口实现的控制功能 | Control functions implemented via serial port | Hardware interface |
| LED控制 | LED control | Output |
| 蜂鸣器控制 | Buzzer control | Output |
| 电机控制 | Motor control | Actuators |
| PWM舵机控制 | PWM servo control | Servos |
| 总线舵机控制 | Bus servo control | Servos |
| 按键获取 | Button acquisition | Input |
| IMU获取 | IMU acquisition | Sensors |
| 手柄获取 | Gamepad acquisition | Input |
| 航模遥控获取 | RC controller acquisition | Input |
| OLED显示内容设置 | OLED display content setting | Display |
| 设置RGB颜色 | Set RGB color | LEDs |
| 按键的不同状态 | Different button states | Input states |
| 校验 | Checksum/verification | Data integrity |
| 幻尔科技iic超声波库 | Hiwonder IIC ultrasonic library | Hardware SDK |

---

## 🎯 Quick Start for Soccer Robot

### Step 1: Test Motors
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

### Step 2: Test Ultrasonic
```bash
ros2 topic echo /sonar_controller/get_distance
```

### Step 3: Test Camera
```bash
ros2 topic echo /image_raw
```

### Step 4: Start YOLO Detection
```bash
ros2 service call /yolov11_node/start std_srvs/srv/Trigger
```

### Step 5: Monitor Object Detection
```bash
ros2 topic echo /yolov11_node/object_detect
```

---

## 📚 Additional Resources

### Launch Files
- **Avoidance**: `src/app/launch/avoidance_node.launch.py`
- **Object Tracking**: `src/app/launch/object_tracking_node.launch.py`
- **YOLO Detection**: `src/yolov11_detect/launch/yolov11_detect.launch.py`
- **Gesture Control**: `src/app/launch/gesture_control_node.launch.py`

### Configuration Files
- **Servo Offsets**: `/home/ubuntu/software/Servo_upper_computer/servo_config.yaml`
- **Color Lab Values**: Check `sdk.yaml_handle` module

### Message Definitions
- **Motor Control**: `ros_robot_controller_msgs/msg/MotorSpeedControl.msg`
- **Object Detection**: `interfaces/msg/ObjectInfo.msg`
- **Servo Control**: `ros_robot_controller_msgs/msg/SetPWMServoState.msg`

---

## 🤝 Tips for Soccer Robot Development

1. **Start Simple**: Get basic motor control and ball detection working first
2. **Tune PIDs**: Spend time tuning PID parameters for smooth control
3. **Test Incrementally**: Test each subsystem independently before integration
4. **Use Simulation**: Test strategies in simulation before hardware
5. **Log Everything**: Use ROS2 logging to debug issues
6. **Handle Edge Cases**: Plan for ball loss, boundary detection, opponent blocking
7. **Optimize Speed**: Soccer requires fast reaction times - minimize processing delays
8. **Train Custom Models**: Train YOLO on your specific ball and goal
9. **Multi-Sensor Fusion**: Combine camera, ultrasonic, and IMU for robust detection
10. **Practice Strategies**: Test different offensive and defensive strategies

---

**Good luck building your soccer robot! 🏆⚽🤖**
