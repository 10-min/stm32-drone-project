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

### Apperance
- 정면

![Image](https://github.com/user-attachments/assets/cfe03464-59e9-4ab9-bc23-78a2e3c0fdc4)

- 측면

![Image](https://github.com/user-attachments/assets/74171508-84fb-4b4b-98e0-7a98583cf79e)

- 후면

![Image](https://github.com/user-attachments/assets/60a75172-8516-42a1-8555-1d4aa9b604ef)

- 상면

![Image](https://github.com/user-attachments/assets/8979f5b1-a0ef-4509-a445-30b8a83ee2ab)

### Hardware Setting ([Blog](https://blog.naver.com/xoals5315/223570628156))
- STM32 Board는 프레임 상단에 양면 테이프로 고정

- HC-06 및 전원 회로는 Mini Bread Board를 부착하여 연결
  ![Image](https://github.com/user-attachments/assets/5b4b6803-4beb-4a02-8e72-967122745600)
  
- ESC와 Motor는 납땜으로 직접 연결
  
  ![Image](https://github.com/user-attachments/assets/64ba00f4-a834-482f-9c94-db5eb0f02722)
  
- 배터리 전원부는 XT-60 connector로 부착
  
  ![Image](https://github.com/user-attachments/assets/6223dab9-2b10-46a3-8f53-b9ba77b814dc)
  
- MPU6050은 진동 감쇠를 위해 프레임 하단부에 고무 패드 부착후 댐퍼 추가하여 연결
  
  ![Image](https://github.com/user-attachments/assets/16e2486b-a67b-4371-93da-8a5d111b2399)
  
- 프로펠러는 좌측정면, 우측후면은 CW, 우측정면, 좌측후면은 CCW 방향으로 설정

### Software Implements
- **Wireless Bluetooth Communication Android Application**
  - Home
  
    ![Image](https://github.com/user-attachments/assets/a49efaca-13c4-42ba-a80b-8222a5e203b2)
    
  - Controller
  
    ![Image](https://github.com/user-attachments/assets/88d10396-d4c6-42d1-a53d-7c047e3798a7)

  - Android Studio 짐
