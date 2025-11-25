# MAD-CHESS  
### Automated Chessboard using STM32F767ZI, LDR Sensors, and TFT LCD
This project is inspired by the concepts presented in the  
**Automated Chessboard** project on Instructables:  
https://www.instructables.com/Automated-Chessboard/
## Overview
MAD-CHESS is an automated physical chessboard that detects chess piece movements in real time and can synchronize with a digital game interface. Inspired by the classic automated chessboard concept, this version is redesigned with modern hardware such as the STM32F767ZI microcontroller, an optical LDR sensing system, and a TFT LCD display for UI visualization.

The system identifies piece positions using a matrix of LDR sensors under each square and manages game logic, display, and communication through the STM32 platform.

> This project replaces reed switches, multiplexers, and Arduino-based logic from the original reference design with more robust STM32 peripherals and an optical detection method.

---

## Features
- **Real-time piece detection** using 64 LDR sensors (one per square).
- **High-performance STM32F767ZI** microcontroller for fast scanning and processing.
- **Color TFT LCD Display** for game status, board view, menus, and debugging.
- **Modular sensor board** placed under the chessboard.
- **Support for human vs human and human vs computer (future expansion)**.
- **Non-contact light-based detection**, more stable than reed switches.
- **Future-ready architecture** for adding motors/XY mechanism if needed.

---

## Hardware Architecture

### 1. Microcontroller  
- **STM32F767ZI (ARM Cortex-M7 @ 216MHz)**
- Handles:
  - LDR scanning (ADC + multiplexer logic)
  - Game logic + piece state changes
  - TFT LCD UI rendering
  - Communication (UART/SPI/I2C depending on configuration)

### 2. Sensors (Piece Detection)
- 64 × **LDR (Light Dependent Resistor)** under each square  
- Each square covered by:
  - LDR
  - Bias resistor forming voltage divider
  - Piece presence reduces light → voltage changes → ADC reads

Advantages:
- No moving parts  
- No magnetic interference  
- Works with any piece style  

### 3. Display
- **TFT LCD 240x320 / 480x320** (ILI9341 or equivalent)  
- Shows:
  - Real-time board state
  - Move history
  - Selected options
  - Debugging graphics  

### 4. Optional Future Hardware
- CoreXY mechanism for automated movement  
- Electromagnet module  
- RS-485 network (if integrating multi-board communication)

---

## Software Architecture

### STM32 Firmware (C / HAL)
Main modules:
- **LDR Scan Engine**
  - Polling or DMA-based ADC scanning
  - Noise filtering
  - Light threshold calibration per square

- **Board State Engine**
  - Detects piece movement by comparing old/new state
  - Handles illegal movement filtering
  - Detects captures

- **Game Logic**
  - Turn management
  - Move logs
  - Event callbacks for UI

- **TFT LCD Drawing**
  - Grid rendering
  - Highlight movement squares
  - Render piece icons
  - Real-time sensor visualization mode (debug)

### Communication (Optional)
- UART/USB for PC interface or chess engine integration (Stockfish, etc.)
- Wi-Fi module support (ESP32) for future cloud connection

---

## How LDR Detection Works
1. LED or ambient light shines through each square  
2. LDR voltage changes based on light intensity  
3. STM32 ADC reads the voltage  
4. Thresholding determines presence of piece  
5. Comparison with previous frame → **movement detected**

Benefits over reed switches:
- No magnets required in pieces  
- Higher resolution for detecting partial movement  
- Faster scanning  
- Less hardware complexity

---

## Credits
This project is inspired by the concepts presented in the  
**Automated Chessboard** project on Instructables:  
https://www.instructables.com/Automated-Chessboard/

All firmware, electronics, and mechanical redesigns — including the STM32F767ZI migration, LDR-based sensing system, TFT LCD interface, and overall architecture — are original work by **Warachot Inkun** as part of the **MAD-CHESS Project**.

