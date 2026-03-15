# RangeSensePro
# RangeSensePro: High-Reliability Ultrasonic Sensing Library

`RangeSensePro` is an object-oriented C++ library designed for the **HC-SR04** ultrasonic distance sensor. Unlike standard implementations, this library focuses on signal integrity and noise rejection, making it suitable for robotics and industrial prototypes.

## 🚀 Key Engineering Features

### 1. Hybrid Digital Filtering Pipeline
To ensure stable readings, the library implements a multi-stage filtering process:
* **Outlier Rejection:** Immediately discards unrealistic values (e.g., 0 or >400cm) to prevent system shocks.
* **Median Filter:** Uses a sliding window to eliminate "spikes" caused by ultrasonic reflections or electrical interference.
* **EMA (Exponential Moving Average):** Smooths the data for a stable output without the heavy memory overhead of a large moving average buffer.

### 2. Real-Time Diagnostics (Stability Metric)
The library continuously calculates the **Standard Deviation (SD)** of the sensor readings. 
* **Low SD:** High confidence in data.
* **High SD:** Potential hardware failure, loose wiring, or turbulent environment.

### 3. Non-Blocking Architecture
By utilizing `millis()` instead of `delay()`, the library ensures the CPU remains free to handle other tasks (like motor control or communication), enabling true multitasking.

## 🛠 Technical Specifications
* **Language:** C++ (Object-Oriented)
* **Platform:** Arduino / ESP32 / AVR
* **Filtering Latency:** ~60ms (adjustable)
* **Memory Footprint:** Optimized for embedded systems.

## 📦 Installation & Usage

1. Copy the `RangeSensePro.ino` code into your project.
2. Define your pins:
```cpp
UltrasonicSensor distanceSensor(9, 10); // TRIG, ECHO

uint16_t dist = distanceSensor.updateAndGetDistance();

Emre Ucar Electrical and Electronics Engineering Student Focused on Embedded Systems, Signal Processing, and IoT Architecture.
