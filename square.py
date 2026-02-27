#!/usr/bin/python3
import serial
import struct
import time
import sys
import math
import numpy as np
from crc8 import crc8

# Параметры робота (как в ваших файлах)
PORT = "/dev/ttyAMA0"
SPEED = 115200
WHEEL_BASE = 0.128      # м
WHEEL_RADIUS = 0.021    # м

# Движение
TARGET_LINEAR_SPEED = 0.10          # м/с
DRIVE_OMEGA = TARGET_LINEAR_SPEED / WHEEL_RADIUS  # рад/с на колесо
TURN_MOTOR_OMEGA = 4.0              # рад/с на колесо (для поворота)

# Состояние: [theta, x, y]
state = [0.0, 0.0, 0.0]

def send_motor_command(ser, left_omega, right_omega):
    buf = struct.pack("<ff", left_omega, right_omega)
    c = crc8()
    c.update(buf)
    packet = b"\x7E" + buf + c.digest()
    ser.write(packet)
    ser.flush()

def eval_odometry(dth_l, dth_r, st):
    # st[0]=theta, st[1]=x, st[2]=y
    st[0] += (dth_r - dth_l) * (WHEEL_RADIUS / WHEEL_BASE)
    ds = (dth_r + dth_l) * (0.5 * WHEEL_RADIUS)
    st[1] += ds * np.cos(st[0])
    st[2] += ds * np.sin(st[0])

def read_wheels(ser):
    """
    Ожидаем ваш формат кадра длиной 17 байт, как в test1/test2:
    unpack("<ffffx") и берём pos_l, pos_r = values[0], values[1]
    """
    chunk = ser.read_until(b"\x7E")
    if len(chunk) != 17:
        return None
    try:
        values = struct.unpack("<ffffx", chunk)
        return values[0], values[1]
    except struct.error:
        return None

def move_straight_by_position(ser, target_distance_m, motor_omega):
    if abs(target_distance_m) < 1e-4:
        return

    direction = 1.0 if target_distance_m > 0 else -1.0
    start_pos_l = None
    start_pos_r = None
    x0, y0 = state[1], state[2]

    while True:
        rr = read_wheels(ser)
        if rr is None:
            continue
        pos_l, pos_r = rr

        if start_pos_l is None:
            start_pos_l, start_pos_r = pos_l, pos_r
            send_motor_command(ser, direction * motor_omega, direction * motor_omega)

        dth_l = pos_l - start_pos_l
        dth_r = pos_r - start_pos_r
        start_pos_l, start_pos_r = pos_l, pos_r

        eval_odometry(dth_l, dth_r, state)

        dist_done = math.hypot(state[1] - x0, state[2] - y0)
        if dist_done >= abs(target_distance_m):
            break

        time.sleep(0.005)

    send_motor_command(ser, 0.0, 0.0)
    time.sleep(0.1)

def turn_in_place_by_position(ser, target_angle_rad, motor_omega):
    if abs(target_angle_rad) < 1e-3:
        return

    direction = 1.0 if target_angle_rad > 0 else -1.0
    start_pos_l = None
    start_pos_r = None
    theta0 = state[0]

    while True:
        rr = read_wheels(ser)
        if rr is None:
            continue
        pos_l, pos_r = rr

        if start_pos_l is None:
            start_pos_l, start_pos_r = pos_l, pos_r
            # Поворот на месте: левое назад, правое вперёд (как в test1)
            send_motor_command(ser, -direction * motor_omega, direction * motor_omega)

        dth_l = pos_l - start_pos_l
        dth_r = pos_r - start_pos_r
        start_pos_l, start_pos_r = pos_l, pos_r

        eval_odometry(dth_l, dth_r, state)

        ang_done = abs(state[0] - theta0)
        if ang_done >= abs(target_angle_rad):
            break

        time.sleep(0.005)

    send_motor_command(ser, 0.0, 0.0)
    time.sleep(0.1)

def drive_square(ser, side_m=0.20, turn_left=True):
    angle = (math.pi / 2.0) * (1.0 if turn_left else -1.0)

    for i in range(4):
        print(f"Сторона {i+1}/4: еду {side_m*100:.0f} см")
        move_straight_by_position(ser, side_m, DRIVE_OMEGA)

        print("Поворот 90°")
        turn_in_place_by_position(ser, angle, TURN_MOTOR_OMEGA)

    print("Квадрат завершён")

def main():
    with serial.Serial(PORT, SPEED, timeout=1) as ser:
        time.sleep(0.5)
        try:
            drive_square(ser, side_m=0.20, turn_left=True)
        except KeyboardInterrupt:
            print("\nЭкстренная остановка")
            send_motor_command(ser, 0.0, 0.0)
        except Exception as e:
            print(f"\nОшибка: {e}")
            send_motor_command(ser, 0.0, 0.0)
            sys.exit(1)

if __name__ == "__main__":
    main()