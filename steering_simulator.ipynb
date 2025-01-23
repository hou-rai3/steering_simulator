import math
import time
import pygame
import pygame.locals
import sys

# オーバーシュート防止用のclamp関数
def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

# 角度を-180°〜180°にとどめる関数
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# PID計算機（角度）
def pid_angle_calculation(target, current, kp, ki, kd, integral, pre_error, dt):
    error = normalize_angle(target - current)  # 角度差を正規化
    integral += error * dt  # 積分値を更新
    deriv = (error - pre_error) / dt  # 微分値を計算
    pid_output = kp * error + ki * integral + kd * deriv  # PID出力を計算
    pid_output = max(min(pid_output, 10.0), -10.0)
    current = normalize_angle(current + pid_output)  # 現在の角度を更新
    return current, integral, error

# PID計算機（速度）
def pid_calculation(target, current, kp, ki, kd, integral, pre_error, dt):
    error = target - current  # 誤差を計算
    integral += error * dt  # 積分値を更新
    deriv = (error - pre_error) / dt  # 微分値を計算
    pid_output = kp * error + ki * integral + kd * deriv  # PID出力を計算
    pid_output = max(min(pid_output, 10.0), -10.0)
    current += pid_output  # 現在の速度を更新
    return current, integral, error

# 車体およびタイヤの状態を管理するクラス
class Vehicle:
    def __init__(self):
        # 初期位置
        self.x = 320.0  # 中心のx座標
        self.y = 240.0  # 中心のy座標

        # 初期ステア角度と速度
        self.left_wheel_angle = 0.0  # ステア角
        self.left_wheel_speed = 0.0  # タイヤ速度

        # PID制御用目標位置
        self.target_x = self.x
        self.target_y = self.y

        # PID制御用ゲイン
        self.kp_angle, self.ki_angle, self.kd_angle = 1.09, 0.00, 0.001  # 角度制御
        self.kp_speed, self.ki_speed, self.kd_speed = 0.19, 0.00, 0.00   # 速度制御

        # PID制御用内部状態
        self.integral_angle = 0.0
        self.integral_speed = 0.0
        self.pre_error_angle = 0.0
        self.pre_error_speed = 0.0

    def update_target(self, target_x, target_y):
        """目標位置を更新する"""
        self.target_x = target_x
        self.target_y = target_y

    def calculate_target(self):
        """目標角度と速度を計算する"""
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        target_angle = math.degrees(math.atan2(dy, dx))  # 目標角度（度）
        target_speed = math.sqrt(dx**2 + dy**2) / 30.0  # 距離に基づく目標速度
        return target_angle, target_speed

    def update_state(self, dt):
        """タイヤのステア角度と速度をPID制御で更新する"""
        target_angle, target_speed = self.calculate_target()

        # ステア角度をPID制御で計算
        self.left_wheel_angle, self.integral_angle, self.pre_error_angle = pid_angle_calculation(
            target_angle, self.left_wheel_angle, self.kp_angle, self.ki_angle, self.kd_angle,
            self.integral_angle, self.pre_error_angle, dt)

        # タイヤ速度をPID制御で計算
        self.left_wheel_speed, self.integral_speed, self.pre_error_speed = pid_calculation(
            target_speed, self.left_wheel_speed, self.kp_speed, self.ki_speed, self.kd_speed,
            self.integral_speed, self.pre_error_speed, dt)

        # 現在の位置を速度に基づいて更新
        self.x += math.cos(math.radians(self.left_wheel_angle)
                           ) * self.left_wheel_speed
        self.y += math.sin(math.radians(self.left_wheel_angle)
                           ) * self.left_wheel_speed

# Pygame初期化
pygame.init()
pygame.display.set_mode((640, 480))
pygame.display.set_caption("Independent Steering Simulation")
surface = pygame.display.get_surface()
surface.fill((0, 128, 0))

font = pygame.font.SysFont(None, 24)

# 車体インスタンスの作成
vehicle = Vehicle()

clock = pygame.time.Clock()

while True:
    dt = clock.tick(60) / 1000.0  # フレーム間の時間を秒単位で計算
    for event in pygame.event.get():
        if event.type == pygame.locals.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.locals.MOUSEMOTION:
            vehicle.update_target(*event.pos)

    # 車体の状態を更新
    vehicle.update_state(dt)

    # 画面描画
    surface.fill((0, 128, 0))

    # タイヤの相対座標
    wheel_offsets = [(-30, -30), (-30, 30), (30, -30), (30, 30)]

    # 車体中央とタイヤを描画
    for offset_x, offset_y in wheel_offsets:
        tire_x = vehicle.x + offset_x
        tire_y = vehicle.y + offset_y
        pygame.draw.circle(surface, (255, 255, 255),
                           (int(tire_x), int(tire_y)), 10)  # タイヤの描画

        # 各タイヤから赤い線を描画
        line_end_x = tire_x + \
            math.cos(math.radians(vehicle.left_wheel_angle)) * 30
        line_end_y = tire_y + \
            math.sin(math.radians(vehicle.left_wheel_angle)) * 30
        pygame.draw.line(surface, (255, 0, 0), (int(tire_x), int(
            tire_y)), (int(line_end_x), int(line_end_y)), 3)

    # デバッグ情報を表示
    debug_info = [
        f"Target: ({vehicle.target_x:.2f}, {vehicle.target_y:.2f})",
        f"Position: ({vehicle.x:.2f}, {vehicle.y:.2f})",
        f"Target Angle: {vehicle.calculate_target()[0]:.2f}°",
        f"Wheel Angle: {vehicle.left_wheel_angle:.2f}°",
        f"Target Speed: {vehicle.calculate_target()[1]:.2f}",
        f"Wheel Speed: {vehicle.left_wheel_speed:.2f}",
    ]
    for i, text in enumerate(debug_info):
        debug_surface = font.render(text, True, (255, 255, 255))
        surface.blit(debug_surface, (10, 10 + i * 20))

    pygame.display.update()
