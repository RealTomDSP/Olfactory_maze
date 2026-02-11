# WOMBAT Alternation Maze 

The **WOMBAT** system is an automated behavioral testing platform designed for animal models, integrating an alternation maze with precise scent delivery, reward dispensing, and capacitive touch sensing. It allows for complex behavioral tasks including sustained contact ("Hold") and rapid response ("Withdraw") trials.
---

## Project Overview

This repository contains the firmware and control software for the WOMBAT maze. The project is structured to guide animals through a 4-stage training progression, let's call it the wombat training protocol. The protocol leads up to the main_wombar code which combines randomized tasks and odor associations for our system. 

### Key Features

* **Mandatory Alternation:** Enforces trial-to-trial switching between left and right maze arms.

* **Multi-Modal Sensing:** Utilizes IR break-beams for location tracking and FDC1004 capacitive sensors for gavage contact detection.


* **Automated Olfactometry:** Controls four odor pumps to deliver specific scents associated with different tasks.


* **Dual-Shield Motor Control:** Uses two Adafruit Motor Shields to manage six independent DC motors for scents and rewards.



---

## Training Progression

The behavior is shaped through four incremental stages before moving to the main experimental protocol:


| **1** | `1_alternation_IR.ino` | <br>**Simple Alternation:** Focuses on lap counting and basic alternation using IR beams.

| **2** | `2_alternation_capsensor.ino` | <br>**Hold Task:** Introduces gavage contact requirements (initial 100ms hold).

| **3** | `3_holdStimulus.ino` | <br>**Scent Introduction:** Adds randomized scent delivery upon nose-poke.

| **4** | `4_withdrawStimulus.ino` | <br>**Withdraw Task:** Requires the animal to exit the nose-poke within a specific window (2s).

| **Main** | `main_wombat.ino` | <br>**Full Protocol:** Randomized Hold/Withdraw tasks with specific odor associations (Coconut vs. Orange).


---

## Hardware Configuration

### I2C Addresses

* **Left Motor Shield:** `0x60` 

* **Right Motor Shield:** `0x61` 

* **Capacitive Sensor (FDC1004):** Standard I2C 



### Motor Mapping

| Motor ID | Function | Task/Odor Association |

| **M1** | Left Scent 1 | Hold Task (Coconut) 

| **M2** | Left Scent 2 | Withdraw Task (Orange) 

| **M3** | Left Reward | Liquid Reward 

| **M4** | Right Scent 1 | Hold Task (Orange) 

| **M5** | Right Scent 2 | Withdraw Task (Coconut) 

| **M6** | Right Reward | Liquid Reward 

---

## Software & Usage

### 1. Motor Calibration & Priming

Use `Motor_controller.ino` alongside the `Wombat_Notebook.ipynb` to prime the liquid lines or troubleshoot odor delivery. The Jupyter Notebook provides a GUI with toggle switches for each motor.

### 2. Experimental Data Logging

Real-time logging is handled via the Python executable in the Jupyter Notebook.

* **Input:** Prompt for Animal ID.
* **Output:** Logs are saved to `WOMBAT_logs/training_stage_1/rat_[ID]/[Date]`.
* **Data:** Captures timestamps, trial type (Hold/Withdraw), motor IDs, and success/fail outcomes.

### 3. Sensor Calibration

The capacitive thresholds (`THRESH_LEFT` and `THRESH_RIGHT`) are currently set to `5.4` but can be tuned for sensitivity in the `.ino` files based on environment noise.

---

> **Note:** Always ensure the serial connection is closed in Python before attempting to upload new code to the Arduino to avoid port-busy errors.
