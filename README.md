# ESP32 BLE Environmental Monitoring System

A high-efficiency IoT node based on **ESP32** and **ESP-IDF** framework that monitors environmental data and transmits via **Bluetooth Low Energy (BLE)**.

## 🚀 Key Features
- **GATT Server Implementation**: Custom BLE profile for Temperature, Humidity, and Light sensing.
- **Data Standardization**: Uses **IEEE-11073 11073-20601** float format for medical/industrial grade data transmission.
- **Ultra-Low Power**: Optimized with **Deep Sleep** and **GPIO/Timer Wakeup** to extend battery life.
- **Hardware Control**: Integrated **Power Amplifier (PA)** control via GPIO to save energy during idle states.
- **Real-time Metrics**: Tracks Round Trip Time (RTT) and Wakeup Latency for performance benchmarking.

## 🛠 Hardware
- **MCU**: ESP32
- **Sensors**: DHT11 (Temperature & Humidity), LDR (Light).
- **External Components**: Power Amplifier (optional), Wakeup buttons.

## 📈 System Workflow
The system reads sensor data, establishes a BLE connection, transmits data packets, and immediately enters Deep Sleep to conserve power until the next cycle.