# 🌱 ESP32 Hoverboard Lawnmower

A **smart autonomous lawnmower** powered by an **ESP32** and a **hacked hoverboard motherboard** running **Emmanuel Feru’s FOC firmware**.  
It drives two hoverboard motors for traction, a high-RPM **3542 brushless motor** for cutting, and uses four ultrasonic sensors for obstacle avoidance. The obstacle avoidance part is quite wacky and i dont advise using that mode. The ultrasonic sensors seems to be detecting tall grasses giving the lawnmower issues. So the best mode right now would be manually controlling with phone or the  spiral option. Another feature that i would advise one to add is the use of perimiter wire for boundary detection. This will be added subsequently as the projects progresses. 

You'll need to update the firmware on the hoverboard"s motherboard or driver to emmanuelm feru's foc hack and youll an st link v2 for that. You'll find detailed guidelines on how to do this in his repo, the link is below.
The entire system runs off a **42 V hoverboard battery pack**.
There's a part on the hoverboard driver board that supplies 12v, this is where the lm2596s 5v voltage regulator is connected, to suppkly the esp32 with 5v. 
For the cutting motor, you can connect a relay to Gpio 5 which i did or use pwm to directly control the esc. There's another relay connected to Gpio 4 that controls a dc fan for cooling the driver.

---

## 🚜 Overview

This project combines affordable hoverboard hardware with the flexibility of the ESP32 to create a compact, powerful, and modular robotic lawnmower.  
It can operate in both **manual** and **autonomous** modes, and is designed for experimentation in robotics, control systems, and FOC motor control.

---

## 🧩 Features

- Dual hoverboard motor drive using **Emmanuel Feru’s FOC hack**
- Cutting motor: **3542 brushless motor + ESC**
- **Four ultrasonic sensors** for obstacle avoidance
- **ESP32** main controller with Wi-Fi and Bluetooth connectivity
- Powered by **42 V hoverboard lithium battery**
- Modular firmware for manual or autonomous operation
- Expansion-ready for GPS, IMU, or vision modules

---

## ⚙️ Hardware Configuration

| Component | Description |
|------------|-------------|
| **Main Controller** | ESP32 dev module |
| **Drive Motors** | 2 × Hoverboard BLDC motors |
| **Motor Driver** | Hoverboard motherboard ([Feru’s FOC firmware](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC)) |
| **Cutting Motor** | 3542 brushless outrunner + 40 amps brushless ESC |
| **Sensors** | 2 × HC-SR04 ultrasonic sensors |
| **Sensors** | 2 x AJ-SR04M ultrasonic sensors |
| **Auxiliary Wheels** | 2 × Caster wheels ([Caster wheels](https://www.aliexpress.com/item/1005007705467517.html?spm=a2g0o.order_list.order_list_main.334.48ef1802W3nwAa))|
| **Power Source** | 42 V hoverboard Li-ion battery |
| **St link V2** | To flash the firmware to the hoverboard motherboard |
| **Power Source(cutting motor)** | 12 V hoverboard Li-polymer battery |
| **Blades** | Weed wacker blades  ([Aliexpress](https://www.aliexpress.com/item/1005006967445757.html?spm=a2g0o.order_list.order_list_main.339.48ef1802W3nwAa)) |

Note that most of these components can be found inside a full hoverboard so you can get one of those used hoverboards off ebay or amazon for cheap. This will be cheaper than buying the hoverboard motors and circuitry seperately.


---

## 🧾 Pinout 

| Function | ESP32 Pin | Notes |
|-----------|-----------|-------|
| **UART TX → Hoverboard RX** | GPIO 16 | Sends control data to FOC board |
| **UART RX ← Hoverboard TX** | GPIO 17 | Receives telemetry (optional) |
| **ESC PWM Signal** | GPIO 5 | Controls 3542 brushless motor speed |
| **Ultrasonic Front Right Echo** | GPIO 35 | Adjustable |
| **Ultrasonic Front Left Echo** | GPIO 34 | Adjustable |
| **Ultrasonic Side Right Trigger** | GPIO 25 | Adjustable |
| **Ultrasonic Side Right Echo** | GPIO 26 | Adjustable |
| **Ultrasonic Side Left Trigger** | GPIO 32 | Adjustable |
| **Ultrasonic Side Left Echo** | GPIO 33 | Adjustable |
| **Spare I/O Pins** | GPIO 2, 4, 5, 18, 19, 21, 22, 23 | Reserved for extensions |

> 🛠️ You can easily edit these pin  later in main

---

## 🔌 Connections Summary

| Device | Connection | Notes |
|--------|-------------|-------|
| Hoverboard motherboard | UART (16/17) | Controlled via serial commands |
| ESC (cutting motor) | PWM (25) | Standard servo-type signal |
| Ultrasonic sensors | GPIOs (see pinout) | Non-blocking measurement using `millis()` |
| ESP32 logic level | 5V | Powered from 5 V regulator (BEC or DC-DC buck) |

---

## 🧠 Software Architecture


