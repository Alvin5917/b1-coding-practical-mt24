# uuv_mission/controller.py

class PDController:
    """
    一个简单的离散时间 PD 控制器。
    控制律: u[t] = KP * e[t] + KD * (e[t] - e[t-1])
    """
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0

    def reset(self):
        """在每次仿真前重置状态"""
        self.prev_error = 0.0

    def step(self, error: float) -> float:
        """
        输入: 当前误差 e[t]
        输出: 控制信号 u[t]
        """
        derivative = error - self.prev_error
        u = self.kp * error + self.kd * derivative
        self.prev_error = error
        return u
    
class PIDController:
    """
    一个简单的离散时间 PID 控制器。
    控制律: u[t] = KP * e[t] + KI * sum(e[0]...e[t]) + KD * (e[t] - e[t-1])
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        """在每次仿真前重置状态"""
        self.prev_error = 0.0
        self.integral = 0.0

    def step(self, error: float) -> float:
        """
        输入: 当前误差 e[t]
        输出: 控制信号 u[t]
        """
        self.integral += error
        derivative = error - self.prev_error
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return u