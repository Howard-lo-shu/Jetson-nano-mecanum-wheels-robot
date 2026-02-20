## Jetson-nano-mecanum-wheels-robot
本專案旨在打造一套基於 **NVIDIA Jetson Nano** 與 **ROS (Robot Operating System)** 的  <span style="color:#4285F4; font-weight:bold;"> 智慧型麥克納姆輪全向移動平台 </span>

透過 Jetson Nano GPIO 控制驅動板與馬達，並結合 ROS 運動控制架構，使平台能達成 **全方位移動、自動導航與智慧控制**。

---

## 專案目標

### 硬體控制
- 使用 Jetson Nano 的 GPIO 腳位控制馬達驅動板
<img width="779" height="484" alt="小車" src="https://github.com/user-attachments/assets/76cc4aa1-fd22-4802-b961-a5cd1cfb034d" />
透過 PCA9685 驅動 L298N 馬達控制四顆 12V 馬達，使車體移動；同時 MPU6050 提供 IMU 姿態資料（特別是 Z 軸角度），與馬達推算出的 X、Y 位移一起組成 /odom 里程資訊，再回傳給 PC 端。
Jetson Nano 本身雖然可以輸出 PWM，但可用的硬體 PWM 腳位非常有限，而且若用軟體模擬 PWM，會受到系統負載影響，造成頻率與佔空比不穩，進而讓馬達轉速不一致、車體運動不平順。PCA9685 是一顆透過 I²C 控制的 16 通道硬體 PWM 控制晶片，能獨立產生精準且穩定的 PWM 訊號，不會占用 Jetson CPU 資源。


### ROS 運動控制
- 建立 ROS 節點讀取 `/cmd_vel` 指令  
- 將速度指令轉換為四輪驅動控制訊號

### 麥克納姆輪全向運動
- 前進 / 後退
- 左右平移
- 原地旋轉
- 斜向移動

---
