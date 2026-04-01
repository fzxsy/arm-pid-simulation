import numpy as np
import matplotlib.pyplot as plt


# 设置中文字体（以黑体 SimHei 为例，Windows 常用）
plt.rcParams['font.sans-serif'] = ['SimHei']   # 指定默认字体为黑体
plt.rcParams['axes.unicode_minus'] = False     # 解决负号 '-' 显示为方块的问题

# 机械臂物理参数
J = 0.5        # 转动惯量 kg·m²
b = 0.1        # 关节摩擦系数
m_arm = 1.0    # 臂质量 kg
g = 9.8        # 重力加速度
L = 0.5        # 臂长 m

dt = 0.01
t = np.arange(0, 20, dt)
target = np.pi / 4    # 目标角度：45°

# PID参数（先用这组）
Kp, Ki, Kd = 20, 5, 7

# 初始化变量
alpha1 = 0.0
omega1 = 0.0
theta1 = 0.0

alpha2 = 0.0
omega2 = 0.0
theta2 = 0.0

theta_list_no_ff = []
theta_list_ff = []

class PIDController:

    def __init__(self, Kp, Ki, Kd):
        # 把参数存起来
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # 初始化内部状态
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        # 三项计算（和主循环里一模一样）
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # 更新状态
        self.prev_error = error 
        return output

pid_no_ff = PIDController(20, 5, 7)
pid_ff = PIDController(20, 5, 7)

for _ in t:
    error1 = target - theta1
    tau_gravity1 = m_arm * g * L * np.cos(theta1)
    tau = pid_no_ff.update(error1, dt)

    alpha1 = (tau - tau_gravity1 - b* omega1)/J
    omega1 += alpha1 * dt
    theta1 += omega1 * dt

    theta_list_no_ff.append(theta1)

for _ in t:
    error2 = target - theta2
    tau_gravity2 = m_arm * g * L * np.cos(theta2)
    tau = pid_ff.update(error2, dt) + tau_gravity2

    alpha2 = (tau - tau_gravity2- b* omega2)/J
    omega2 += alpha2 * dt
    theta2 += omega2 * dt

    theta_list_ff.append(theta2)

plt.plot(t, theta_list_no_ff, label = "无前馈")
plt.plot(t, theta_list_ff, label = "有前馈")
plt.axhline(target, color='red', linestyle='--', label='target')
plt.xlabel("时间（秒）")
plt.ylabel("角度（rads）")
plt.grid(True)
plt.legend()
plt.show()