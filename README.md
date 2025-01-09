# STM32 Drone Project

## Overview
- 이 프로젝트의 목표는 STM32 마이크로컨트롤러를 사용하여 맞춤형 자작 드론을 설계하고 제작하는 것입니다. 기본 비행 제어, 안정화, 무선 통신 구현에 중점을 두었습니다.

## Objective
- STM32 프로그래밍을 학습하고 드론 하드웨어 제어에 적용.
- 기본 비행 안정화 알고리즘 개발.
- 원격 제어를 위한 무선 통신 모듈 통합.
- 드론 조립 및 테스트를 통해 실습 경험 획득.

## Hardware
- MCU: STM32F103RBT6 / 72 MHz, up to 1 Mbyte of Flash with motor control, USB and CAN
- Frame: ABS 플라스틱 재질, F330 4-Axis Frame
- Motor: BLDC, A2212 1400KV
- Propeller: 8045 plastic propeller for Quardcopter
- ESC: Hobbywing skywalker 20A
- Battery: 3S, 1800mAh Lipo battery
- Sensor
  - IMU: MPU6050
  - Wireless Communication: HC-06 (Bluetooth)

## Tool and Framework

|     |     |
| --- | --- |
| IDE | STM32CubeIDE |
| Firmware Development | STM32 HAL Library |
| Programming Language | C |
| Communication Protocols | I2C(IMU), UART(HC-06, Debugging) | 
| Drone Control Algorithms | PID Control |

## Features
- IMU와 PID Control을 융합한 비행 안정화
- Kalman filter를 통한 비행 안정화
- HC-06 Bluetooth Module을 이용한 스마트폰 원격 조종
- 일정 시간 신호 부재시 모터 정지

## Implements

### Hardware Setting
- 프레임에 STM32 Board를 부착.
- HC-06 및 전원 회로는 Mini Bread Board를 부착하여 연결.
- 

### Hardware Setting
