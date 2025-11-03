## Jetson-nano-mecanum-wheels-robot
本專案旨在打造一套基於 **NVIDIA Jetson Nano** 與 **ROS (Robot Operating System)** 的  <span style="color:#4285F4; font-weight:bold;"> 智慧型麥克納姆輪全向移動平台 </span>

透過 Jetson Nano GPIO 控制驅動板與馬達，並結合 ROS 運動控制架構，使平台能達成 **全方位移動、自動導航與智慧控制**。

---

## 專案目標

### 硬體控制
- 使用 Jetson Nano 的 GPIO 腳位控制馬達驅動板  

### ROS 運動控制
- 建立 ROS 節點讀取 `/cmd_vel` 指令  
- 將速度指令轉換為四輪驅動控制訊號

### 麥克納姆輪全向運動
- 前進 / 後退
- 左右平移
- 原地旋轉
- 斜向移動

---
