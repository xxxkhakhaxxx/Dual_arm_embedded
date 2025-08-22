# 🤖 Dual_arm_embedded
Control a dual-arm robotic system via CAN Bus using STM32 microcontrollers.

<p align="center">
  <img width="435" 
       height="231"
       alt="image"
       src="https://github.com/user-attachments/assets/c4bcd18a-4ee3-4f55-b150-ca67c0c89d87" />
  <br>
  <em>Figure 1: Coordinates of a 3-DOF Dual-arm</em>
</p>


## 🧠 Overview

This project provides embedded firmware for controlling a dual-arm robot using STM32 MCUs. It enables synchronized motion, real-time communication via CAN Bus, and modular control logic for scalable development.

---

## 🚀 Features

- ✅ Trajectory Planning: Task-space and Joint-space
- ✅ Dual-arm: FK, IK and Dynamics
- ✅ Controllers: PD, SMC, Sync-PD, Sync-SMC
- ✅ BLDC control: Torque mode and Position mode
---

## 🛠️ Hardware overview

<p align="center">
  <img width="625"
       height="432"
       alt="image"
       src="https://github.com/user-attachments/assets/2b0048c6-f7b0-4013-b958-1d01a6d20f30" />
  <br>
  <em>Figure 2: System block diagram and its function</em>
</p>

---

## 📁 Project Structure
Dual_arm_embedded/<br>
├─ 📂 LATN_DUAL_ARM_MASTER: main.c<br>
├─ 📂 LATN_DUAL_ARM_SLAVE: main.c<br>
├─ 📂 Library_HH_Master<br>
│ &nbsp; &nbsp; ├─ 📂 Application<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppCommunicate: UART protocol<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppConfig     : Controller setting - Robot Dynamics <br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppControl    : Controller algorithm<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppData       : Flag management<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppPeriod     : State machine management<br>
│ &nbsp; &nbsp; ├─ 📂 Middleware<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 ApiMacro<br>
│ &nbsp; &nbsp; └─── LibraryHHInterface_Master.h: header management<br>
├─ 📂 Library_HH_SLAVE<br>
│ &nbsp; &nbsp; ├─ 📂 Application<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppCommunicate: CAN + UART protocol<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppConfig     : Robot Kinematics<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppData       : Flag management<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 AppPeriod     : State machine management<br>
│ &nbsp; &nbsp; ├─ 📂 Middleware<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 ApiMacro<br>
│ &nbsp; &nbsp; │ &nbsp; &nbsp;   ├─ 📂 ApiProtocol: BLDC protocol <br>
│ &nbsp; &nbsp; └─── LibraryHHInterface_Slave.h: header management<br>
└── README.md
