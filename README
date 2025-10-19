# STM32U5 Time-of-Flight Alarm

## Introduction

This project implements a distance-based alarm system using an STM32U5 microcontroller and a Time-of-Flight (ToF) sensor. When an object is detected within a certain range, the system will trigger an alarm. This project utilizes FreeRTOS for task management.

## Hardware

* **Microcontroller**: STM32U5 series
* **Sensor**: Time-of-Flight (ToF) distance sensor

## Software

* **IDE**: STM32CubeIDE
* **RTOS**: FreeRTOS
* **Libraries**: STM32 HAL Library

## Functionality

The main functionality of this project is to create a real-time alarm system based on distance measurement.

* The ToF sensor continuously measures the distance to the nearest object.
* A dedicated FreeRTOS task reads the distance data from the sensor.
* If the measured distance is less than a predefined threshold, another task is notified to trigger an alarm.
* The alarm could be a buzzer, an LED, or any other output device connected to the microcontroller.