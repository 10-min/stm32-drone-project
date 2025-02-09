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

  - Android Studio 사용
  - Controller는 Canvas를 사용해 구현하였으며, 2개의 조이스틱을 구현
  - 좌측은 Pitch, Roll, 우측은 Throttle, Yaw 제어
  
- **ESC Calibration**
  - STM32 PWM 제어 400Hz 세팅
  - 1-2ms 신호를 최소에서 최대값으로 감지
  - 드론에 사용된 ESC는 다음과 같은 Calibration 과정이 필요
    > 1. Throttle 최대인 상태 유지
    > 2. 전원을 키고 2초간 대기
    > 3. Beep-Beep tone이 방출됨
    > 4. 다시 Throttle을 최소로 내리면 Beep tone이 여러번 울림
    > 5. Beep tone이 길게 울리면 최소-최대 범위가 확인됨

- **MPU-6050**
  - Stm32 SPI 통신을 위해 Open Source Library 사용 [Git](https://github.com/mokhwasomssi/stm32_hal_mpu6050.git)
  - DLPF(Digital Low Pass Filter)를 사용하기 위해 set_DLPF 함수 추가

- **PID Control**
  - 안정화를 위해 P-PID Dual PID 제어를 사용
  - P는 Target angle과 current angle과의 diff
  - Dual P는 P * stabilize_kp와 각속도와의 차이
 
- **Filter**
  - 개발 초기 Low pass filter와 DLPF를 사용했으며, DLPF 정도를 높여서 안정성을 높였음
  - DLPF 사용 시 안정성은 높아졌지만, 반응 속도가 느려 다른 방안을 탐구함
  - 이후 Kalman filter를 적용하여 안정화를 한 층 더 높일 수 있었음
 
## Test
- 첫 번째 테스트

[![Test1](https://github.com/user-attachments/assets/9ce4805e-5410-4fc6-8129-97b6fcc0fa93)](https://www.youtube.com/watch?v=l1rKK0Qbktg)

- Frame 손상 이후 랜딩 기어를 추가하여 다시 테스트

[![Test2](https://github.com/user-attachments/assets/cefb9f4e-6dd9-4ad4-b9e6-57b4cffc879e)](https://www.youtube.com/watch?v=jNpG6de5bn8)
