"""Servo simulation domain model and parameter catalog."""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass
class ServoModel:
    position: float = 0.0
    speed: float = 0.0
    current: float = 0.0
    voltage: float = 48.2
    temperature: float = 33.0
    target: float = 90.0
    speed_limit: float = 800.0
    enabled: bool = False
    connected: bool = False
    phase: float = 0.0

    def step(self, interval_ms: int, mode: str) -> None:
        dt = interval_ms / 1000.0
        if not (self.connected and self.enabled):
            self.speed *= 0.82
            if abs(self.speed) < 0.05:
                self.speed = 0.0
            self.position += self.speed * 6.0 * dt
            self.current *= 0.70
            self.temperature = max(30.0, self.temperature - 0.008)
            self.phase = (self.phase + self.speed * math.tau * dt / 60.0) % math.tau
            return

        if mode == "速度模式":
            requested_speed = clamp(self.target, -self.speed_limit, self.speed_limit)
            speed_delta = clamp(requested_speed - self.speed, -2200.0 * dt, 2200.0 * dt)
            self.speed += speed_delta
            self.position += self.speed * 6.0 * dt
            self.current = clamp(0.35 + abs(speed_delta) * 0.012 + abs(self.speed) / 2400.0, 0.0, 12.0)
        elif mode == "电流模式":
            requested_current = clamp(self.target, -12.0, 12.0)
            self.current += clamp(requested_current - self.current, -18.0 * dt, 18.0 * dt)
            acceleration = self.current * 260.0 - self.speed * 0.35
            self.speed = clamp(self.speed + acceleration * dt, -self.speed_limit, self.speed_limit)
            self.position += self.speed * 6.0 * dt
        else:
            error = self.target - self.position
            requested_speed = clamp(error * 2.8, -self.speed_limit, self.speed_limit)
            speed_delta = clamp(requested_speed - self.speed, -2600.0 * dt, 2600.0 * dt)
            self.speed += speed_delta
            if abs(error) < 0.02 and abs(self.speed) < 0.5:
                self.position = self.target
                self.speed = 0.0
            else:
                self.position += self.speed * 6.0 * dt
            self.current = clamp(0.25 + abs(speed_delta) * 0.014 + abs(error) * 0.002, 0.0, 12.0)

        self.voltage = 48.0 + math.sin(self.phase * 0.3) * 0.45
        self.temperature = clamp(self.temperature + abs(self.current) * 0.0025 - 0.006, 30.0, 88.0)
        self.phase = (self.phase + self.speed * math.tau * dt / 60.0) % math.tau


PARAMETERS = [
    ("1000", "00", "控制模式", "位置模式", "位置模式", "模式", "应用配置"),
    ("1001", "00", "控制方向", "CCW", "CCW", "方向", "应用配置"),
    ("1010", "00", "最大转速", "3000", "3000", "rpm", "速度参数"),
    ("1011", "00", "加速度", "1200", "1200", "rpm/s", "速度参数"),
    ("1012", "00", "减速度", "1200", "1200", "rpm/s", "速度参数"),
    ("1020", "00", "位置环比例增益", "42", "42", "0.1 Hz", "位置环"),
    ("1021", "00", "位置到达误差", "20", "20", "pulse", "位置环"),
    ("1022", "00", "位置到达等待时间", "80", "80", "ms", "位置环"),
    ("1030", "00", "速度环比例增益", "36", "36", "0.1 Hz", "速度环"),
    ("1031", "00", "速度环积分时间", "18", "18", "ms", "速度环"),
    ("1040", "00", "电流环比例增益", "28", "28", "0.1 Hz", "电流环"),
    ("1041", "00", "最大输出电流", "8.0", "8.0", "A", "电流参数"),
    ("1050", "00", "编码器线数", "4096", "4096", "pulse", "编码器"),
    ("1051", "00", "电机极对数", "4", "4", "pair", "电机参数"),
    ("1060", "00", "电子齿轮分子", "1", "1", "", "脉冲输入"),
    ("1061", "00", "电子齿轮分母", "1", "1", "", "脉冲输入"),
    ("2000", "00", "RS485 地址", "1", "1", "HEX", "通讯参数"),
    ("2001", "00", "CAN 节点 ID", "1", "1", "", "通讯参数"),
    ("2002", "00", "通讯超时", "500", "500", "ms", "通讯参数"),
    ("3000", "00", "驱动器温度", "33.0", "33.0", "°C", "监视参数"),
]
