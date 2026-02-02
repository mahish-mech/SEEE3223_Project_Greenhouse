# 🌿 SmartGreenhouse-STM32: Automated Environmental Control System

**SmartGreenhouse-STM32** is an intelligent embedded monitoring system designed on the **STM32F4** microcontroller. It features real-time temperature and humidity sensing, automated ventilation control, and a priority-based alarm system to ensure optimal growing conditions.

## ✨ Key Features

* **Precision Sensing** – Continuously monitors environment data using the **DHT11 Temperature & Humidity Sensor**.
* **Automated Ventilation** – Intelligent **Servo Motor** control that automatically opens vents when the temperature exceeds the critical threshold (32°C).
* **Panic & Safety Protocol** – Features a **Panic Button** hardware interrupt that immediately triggers a system-wide emergency mode (Full Open Vent + Continuous Alarm).
* **Hysteresis Logic** – Implements a "cool-down" buffer (1°C hysteresis) to prevent the motor from jittering between open/close states near the threshold.
* **Real-Time Status Display** – Live updates of temperature, humidity, and system status via an **I2C LCD Display**.
* **Reset Override** – A manual override switch to force the system back to a safe "Normal" state, regardless of sensor readings.

## 🛠️ How It Works

* **Sensing Stage** – The STM32 polls the DHT11 sensor every 1 second (1000ms) to gather fresh environmental data.
* **Logic Processing** – The system uses a **Finite State Machine (FSM)** to switch between **NORMAL**, **ALARM**, and **PANIC** modes based on sensor inputs and button interrupts.
* **Actuation Stage** – 
    * If **Temp > 32°C**: The Servo rotates to 90° (Open Vent) and the Buzzer pulses.
    * If **Panic Button Pressed**: The Servo opens fully and the Buzzer sounds continuously.
* **User Feedback** – The LCD updates instantly to show specific error messages ("DHT FAIL") or current mode status ("MODE: ALARM").

## 📊 Logic & Feedback Table

| System State | Servo Position (Vent) | Buzzer (PA0) | LCD Status | Logic Priority |
| :--- | :--- | :--- | :--- | :--- |
| **🟢 Normal Mode** | Closed (0°) | 🔇 Silent | `T:28C H:60%` | 💤 Lowest |
| **🔴 Alarm Mode** | Open (90°) | 🔊 Beeping (1s cycle) | `MODE: ALARM` | ⭐ Standard |
| **⚠️ Panic Mode** | Open (90°) | 🔊 Continuous | `MODE: PANIC` | 🏆 Highest (Interrupt) |
| **🔵 Reset Override** | Closed (0°) | 🔇 Silent | `RESET OVERRIDE` | ⚡ Manual |

## 🚀 Potential Upgrades

* **IoT Integration** – Interfacing an **ESP8266** module to send temperature data to a cloud dashboard (Blynk/ThingSpeak) for remote monitoring.
* **Soil Moisture Regulation** – Adding a capacitive soil moisture sensor to automatically trigger a water pump relay when the soil is dry.
* **RTC Scheduling** – Using the internal Real-Time Clock (RTC) to set specific ventilation schedules for day vs. night cycles.

## 🎯 Why This Project?

* **Industrial Control Logic** – Demonstrates how to use **Hysteresis** loops to create stable control systems, a standard technique in industrial automation.
* **Interrupt-Driven Safety** – Showcases how to use hardware interrupts (GPIO_EXTI) for emergency stops, ensuring the system reacts instantly to user input.
* **Multi-Module Integration** – Successfully integrates 4 distinct protocols/methods: **PWM** (Servo), **I2C** (LCD), **One-Wire** (DHT11), and **GPIO** (Buzzer/Buttons).

---

### 👨‍💻 Project Team

**Developed by:**
* Mahish (Lead & System Architect)
* [Mugilan]
* [Ali Alfian]
* [Shabiq]

**Supervised by:**
* **Dr. Fauzan Khairi Che Harun**
