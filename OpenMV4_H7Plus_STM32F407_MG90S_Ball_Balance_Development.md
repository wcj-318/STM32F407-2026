# OpenMV4 H7 Plus + STM32F407 + MG90S 钢球平衡控制系统开发文档

## 1. 项目概述

项目目标：

利用 OpenMV4 H7 Plus 视觉检测 PPR 摆杆中的钢球位置，通过 UART 将坐标发送给 STM32F407VET6，STM32 运行 PID 控制算法，通过 PWM 驱动 MG90S 金属舵机调整摆杆角度，实现钢球稳定保持在指定位置。

系统组成：

- OpenMV4 H7 Plus：视觉检测端
- STM32F407VET6：主控制器
- MG90S 金属舵机：执行机构
- 3D 打印摆杆机构 + PPR 管：滚球平台
- UART：视觉数据通信
- OLED/VOFA+：调试扩展

---

# 2. 系统架构

OpenMV4 H7 Plus
↓
摄像头采集
↓
钢球识别与滤波
↓
UART发送 ball_x、ball_y
↓
STM32F407
↓
误差计算
↓
PID控制
↓
PWM输出
↓
MG90S舵机
↓
摆杆角度变化
↓
钢球运动反馈

形成视觉闭环控制。

---

# 3. 硬件方案

## 3.1 OpenMV4 H7 Plus

功能：

- 摄像头采集
- PPR管ROI检测
- 钢球Blob/Circle检测
- 位置滤波
- 坐标输出

已有程序：

end.py

输出：

BALL,x,y,r


## 3.2 STM32F407VET6

负责：

- UART接收
- PID计算
- PWM控制
- OLED显示
- 参数调节

开发：

STM32CubeMX + HAL库 + Keil5


## 3.3 MG90S舵机

控制方式：

50Hz PWM

脉宽：

1000us：一侧极限
1500us：中心位置
2000us：另一侧极限


---

# 4. 软件开发流程

必须严格按照阶段开发。

---

# Stage0 舵机基础控制

目标：

STM32输出PWM控制MG90S。

实现：

Servo_Init()

Servo_SetPWM()

Servo_SetAngle()


测试：

1500us：
摆杆水平

1000us：
观察方向

2000us：
观察方向


确认机械方向。


---

# Stage1 OpenMV UART通信

通信参数：

115200bps

8N1


协议：

AA
X_H
X_L
Y_H
Y_L
CRC
55


发送：

钢球X坐标

钢球Y坐标


---

# Stage2 STM32接收坐标

USART1：

PA9 TX

PA10 RX


解析：

ball_x

ball_y


OLED显示：

BALL X:
160


BALL Y:
120


---

# Stage3 坐标误差计算


目标中心：

BALL_TARGET_X = 160


error:

target_x - ball_x


---

# Stage4 PID控制


公式：

output =
Kp*error
+
Ki*sum(error)
+
Kd*(error-last_error)


初始参数：

Kp=0.15

Ki=0.001

Kd=0.05


---

# Stage5 舵机闭环控制


PWM：

servo_pwm =
1500 + pid_output


限制：

1000~2000us


---

# Stage6 滤波优化


采用一阶低通：

filter =
0.7*old
+
0.3*new


降低视觉抖动。


---

# Stage7 整机测试


测试1：

静止钢球中心保持。


测试2：

指定位置控制：

+5cm

-5cm


测试3：

小车运动：

钢球保持中心。


---

# 5. STM32工程结构


Ball_Balance_Control

Core

- main.c


Drivers

- uart.c
- servo.c
- oled.c


PID

- pid.c
- pid.h


OpenMV

- openmv_uart.c


Config

- config.h


---

# 6. 参数管理

config.h


#define BALL_TARGET_X 160

#define PID_KP 0.15f

#define PID_KI 0.001f

#define PID_KD 0.05f

#define SERVO_CENTER 1500

#define SERVO_MIN 1000

#define SERVO_MAX 2000


---

# 7. 调试规则


响应慢：

增加Kp


震荡：

降低Kp

增加Kd


无法回中心：

增加Ki


舵机抖动：

增加滤波

降低Kp


---

# 8. Codex开发要求


严格按Stage0~Stage7执行。

每完成一个阶段必须输出：

1.完整代码
2.接线说明
3.测试步骤
4.预期现象
5.问题分析


代码要求：

- STM32 HAL库
- 模块化
- 参数集中
- 注释完整
- 支持Keil5
- 支持后续循迹扩展


最终完成：

OpenMV视觉

+

STM32 PID

+

MG90S舵机

+

钢球平衡闭环系统
