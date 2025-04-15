import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# ===== 电机-配流阀系统（第二阶模型） =====
class MotorValveSystem:
    def __init__(self, J, B, Kt):
        self.J = J            # 电机转动惯量
        self.B = B            # 阻尼系数
        self.Kt = Kt          # 电机力矩常数
        self.theta = 0.0      # 阀芯当前角度 (rad)
        self.omega = 0.0      # 阀芯当前角速度 (rad/s)

    def step(self, U, dt):
        # 欧拉法积分更新角度和角速度
        omega_dot = (self.Kt * U - self.B * self.omega) / self.J  # 角加速度
        self.omega += omega_dot * dt
        self.theta += self.omega * dt
        return self.theta

# ===== 液压执行器系统（第一阶模型） =====
class HydraulicActuator:
    def __init__(self, K_h, tau_h, spool_max, initial_phi=0.0):
        self.K_h = K_h            # 液压增益常数
        self.tau_h = tau_h        # 时间常数
        self.spool_max = spool_max  # 阀芯最大偏转角 (rad)
        self.phi = initial_phi    # 当前钻头偏转角 (rad)

    def step(self, spool_angle, dt):
        # 归一化输入
        u_f = np.clip(spool_angle, -self.spool_max, self.spool_max) / self.spool_max
        # 一阶系统微分方程
        phi_dot = (self.K_h / self.tau_h) * u_f - (1.0 / self.tau_h) * self.phi
        self.phi += phi_dot * dt
        return self.phi

# ===== 组合系统（电机-阀与液压执行器串联，三阶系统） =====
class CombinedSystem:
    def __init__(self, J, B, Kt, K_h, tau_h, spool_max, initial_phi=0.0):
        self.motor = MotorValveSystem(J, B, Kt)
        self.hydr = HydraulicActuator(K_h, tau_h, spool_max, initial_phi)
        self.spool_max = spool_max

    def step(self, U, dt):
        # 更新电机-阀系统状态
        theta = self.motor.step(U, dt)
        # 限制阀芯角度在物理范围内
        if theta > self.spool_max:
            theta = self.spool_max
            self.motor.theta = theta
        if theta < -self.spool_max:
            theta = -self.spool_max
            self.motor.theta = theta
        # 更新液压执行器状态
        phi = self.hydr.step(theta, dt)
        return phi

# ===== PID控制器 =====
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, output_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_limit = output_limit  # 输出限幅
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        derivative = (error - self.prev_error) / self.dt
        new_integral = self.integral + error * self.dt
        output = self.Kp * error + self.Ki * new_integral + self.Kd * derivative
        if self.output_limit is not None:
            if output > self.output_limit:
                output = self.output_limit
                if error > 0:
                    new_integral = self.integral
            elif output < -self.output_limit:
                output = -self.output_limit
                if error < 0:
                    new_integral = self.integral
        self.integral = new_integral
        self.prev_error = error
        return output

# ===== 模糊控制器 =====
class FuzzyController:
    def __init__(self):
        # 定义输入变量：误差及其变化率
        self.error = ctrl.Antecedent(np.linspace(-0.3, 0.3, 61), 'error')
        self.error_dot = ctrl.Antecedent(np.linspace(-0.2, 0.2, 61), 'error_dot')
        # 定义输出变量：阀芯目标角
        self.alpha_cmd = ctrl.Consequent(np.linspace(-0.5, 0.5, 61), 'alpha_cmd')

        # 隶属函数定义
        self.error['neg'] = fuzz.trimf(self.error.universe, [-0.3, -0.2, -0.05])
        self.error['zero'] = fuzz.trimf(self.error.universe, [-0.1, 0.0, 0.1])
        self.error['pos'] = fuzz.trimf(self.error.universe, [0.05, 0.2, 0.3])

        self.error_dot['neg'] = fuzz.trimf(self.error_dot.universe, [-0.2, -0.05, 0.0])
        self.error_dot['zero'] = fuzz.trimf(self.error_dot.universe, [-0.05, 0.0, 0.05])
        self.error_dot['pos'] = fuzz.trimf(self.error_dot.universe, [0.0, 0.05, 0.2])

        self.alpha_cmd['neg_big'] = fuzz.trimf(self.alpha_cmd.universe, [-0.5, -0.5, -0.25])
        self.alpha_cmd['neg_small'] = fuzz.trimf(self.alpha_cmd.universe, [-0.3, -0.15, 0.0])
        self.alpha_cmd['zero'] = fuzz.trimf(self.alpha_cmd.universe, [-0.1, 0.0, 0.1])
        self.alpha_cmd['pos_small'] = fuzz.trimf(self.alpha_cmd.universe, [0.0, 0.15, 0.3])
        self.alpha_cmd['pos_big'] = fuzz.trimf(self.alpha_cmd.universe, [0.25, 0.5, 0.5])

        # 构建模糊规则
        rules = [
            ctrl.Rule(self.error['pos'] & self.error_dot['pos'], self.alpha_cmd['pos_big']),
            ctrl.Rule(self.error['pos'] & self.error_dot['zero'], self.alpha_cmd['pos_small']),
            ctrl.Rule(self.error['pos'] & self.error_dot['neg'], self.alpha_cmd['neg_small']),
            ctrl.Rule(self.error['neg'] & self.error_dot['neg'], self.alpha_cmd['neg_big']),
            ctrl.Rule(self.error['neg'] & self.error_dot['zero'], self.alpha_cmd['neg_big']),
            ctrl.Rule(self.error['neg'] & self.error_dot['pos'], self.alpha_cmd['neg_small']),
            ctrl.Rule(self.error['zero'] & self.error_dot['pos'], self.alpha_cmd['pos_small']),
            ctrl.Rule(self.error['zero'] & self.error_dot['zero'], self.alpha_cmd['zero']),
            ctrl.Rule(self.error['zero'] & self.error_dot['neg'], self.alpha_cmd['pos_small']),
            ctrl.Rule(self.error['pos'] & self.error_dot['pos'], self.alpha_cmd['pos_big']),
            ctrl.Rule(self.error['neg'] & self.error_dot['neg'], self.alpha_cmd['neg_big']),
            ctrl.Rule(self.error['zero'] & self.error_dot['zero'], self.alpha_cmd['zero']),
            ctrl.Rule(self.error['pos'] & self.error_dot['pos'], self.alpha_cmd['pos_big']),
            ctrl.Rule(self.error['neg'] & self.error_dot['pos'], self.alpha_cmd['pos_small']),
            ctrl.Rule(self.error['pos'] & self.error_dot['neg'], self.alpha_cmd['neg_small'])
        ]

        control_system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(control_system)

    def compute(self, error_value, error_dot_value):
        error_value = np.clip(error_value, -0.3, 0.3)
        error_dot_value = np.clip(error_dot_value, -0.2, 0.2)
        self.sim.reset()
        self.sim.input['error'] = error_value
        self.sim.input['error_dot'] = error_dot_value
        self.sim.compute()
        print(f"Input: error={error_value}, error_dot={error_dot_value}")
        print(f"Computed Output: {self.sim.output}")
        if 'alpha_cmd' not in self.sim.output:
            print(f"Error: The output 'alpha_cmd' was not found. Current input values: error={error_value}, error_dot={error_dot_value}")
            return 0.0
        return self.sim.output['alpha_cmd']

# 创建全局模糊控制器实例
fuzzy_controller = FuzzyController()

# ===== 仿真运行函数 =====
def run_simulation(scenario, Kp, Ki, Kd,
                   J=0.01, B=0.1, Kt=1.0,
                   K_h=0.2, tau_h=0.5, spool_max=0.5,
                   t_end=5.0, dt_sim=0.001, update_progress=None):
    # 根据工况设置初始条件
    if scenario == 1:
        initial_phi = np.deg2rad(10.0)
    else:
        initial_phi = 0.0

    system = CombinedSystem(J, B, Kt, K_h, tau_h, spool_max, initial_phi)
    pid = PIDController(Kp, Ki, Kd, dt_sim, output_limit=10.0)

    phi_des = 0.0              # 目标姿态角 (rad)
    phi_des_step = np.deg2rad(5.0) if scenario == 3 else 0.0
    disturbance_time = 2.0     # 扰动时间 (s)
    disturbance_value = np.deg2rad(2.86)  # 扰动幅度

    time_list = []
    phi_list = []
    theta_list = []
    phi_des_list = []
    prev_error = phi_des - system.hydr.phi

    steps = int(t_end / dt_sim)

    for i in range(steps + 1):
        t = i * dt_sim

        if scenario == 2 and abs(t - disturbance_time) < 1e-9:
            system.hydr.phi += disturbance_value
        if scenario == 3 and t >= disturbance_time:
            phi_des = phi_des_step

        error = phi_des - system.hydr.phi
        d_error = (error - prev_error) / dt_sim
        prev_error = error

        # 使用模糊控制器计算目标角
        alpha_cmd = fuzzy_controller.compute(error, d_error)

        theta_current = system.motor.theta
        error_theta = alpha_cmd - theta_current
        U = pid.compute(error_theta)

        phi_val = system.step(U, dt_sim)

        time_list.append(t)
        phi_list.append(phi_val)
        theta_list.append(system.motor.theta)
        phi_des_list.append(phi_des)

        if update_progress:
            progress = (i + 1) / (steps + 1) * 100
            update_progress(progress)

    return time_list, phi_list, theta_list, phi_des_list
