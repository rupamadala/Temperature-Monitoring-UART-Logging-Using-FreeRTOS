Temperature Monitoring & UART Logging System using FreeRTOS

A real-time embedded application built on STM32L4 using FreeRTOS, featuring periodic temperature acquisition from a DHT11 sensor and asynchronous UART logging.
This project demonstrates RTOS task scheduling, inter-task communication with queues, and microsecond timing via ARM DWT cycle counter.

📌 Project Overview
This project implements a simple but robust real-time pipeline:

Task 1 — Sensor Task
Runs every 2 seconds, reads the DHT11 sensor using precise GPIO timing.
Uses the DWT cycle counter for microsecond-level protocol accuracy.

Task 2 — Logging Task
Receives sensor data from a FreeRTOS queue and pushes formatted messages over UART for monitoring or debugging.


🧰 Features
✔ Periodic temperature and humidity sampling
✔ Accurate microsecond timing using ARM DWT cycle counter
✔ Two-task FreeRTOS architecture
✔ Inter-task communication via FreeRTOS Queue
✔ UART-based logging (115200 baud default)
✔ Modular driver for DHT11 (Start, Response, Bit-by-bit reading)
✔ Clean task separation for scalability
✔ Fully compatible with STM32CubeIDE and CMSIS-RTOS2


+-------------------+          +-------------------+
|   Sensor Task     |          |   Logging Task    |
| (runs every 2 sec)| --Q-->   | (UART TX)         |
| Reads DHT11       |          | Dequeues data     |
| Uses DWT timing   |          | Sends via UART    |
+-------------------+          +-------------------+
Q- Queue Handling

⚙️ Hardware Setup
Component	Purpose
STM32L4 MCU	Main microcontroller
DHT11 Sensor	Temperature & humidity measurement
UART-TTL Converter	Data logging to PC
USB Cable	Debug + power
Breadboard / Wires	Sensor connections


DHT11 Wiring
DHT11 Pin	STM32 Pin
VCC	3.3V
GND	GND
DATA	GPIO (configurable, e.g., PA5)

🧵 FreeRTOS Tasks
1. Sensor Task
Executes every 2 seconds using vTaskDelayUntil()
Initiates DHT11 start signal
Reads 40 bits of sensor data
Uses DWT cycle counter for accurate µs delays
Sends structured data to queue

2. Logging Task
Blocks on queue receive
Formats data into UART strings
Prints via HAL_UART_Transmit()

🧪 Testing & Validation
Verified sensor timing using DWT cycle timestamps
Stress-tested task scheduling under artificial CPU load
Confirmed queue behavior for both fast & slow logging conditions
Checked DHT11 correctness against known environment values

📄 License
This project is released under the MIT License.
