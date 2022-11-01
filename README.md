# ECSE444 
Microcessors coursework at McGill University. Taken in the fall of 2018. Labs and project were done with the Discovery IoT development board and the IDE Keil μVision5. The code is written in C, but knowledge in ARM assembly is useful to be able to debug. 

## Lab 0 - Getting Started 
This lab will serve two purposes:
- Getting familiar with the IDE Keil μVision5.
- Refreshing your ARM assembly and C programming knowledge via a few programming exercises.


## Lab 1 - CMSIS-DSP functions for optimized code
In this lab you will be introduced to the CMSIS-DSP software library, which is a collection of common
signal processing functions that ARM has optimized for the various Cortex-M processor cores.
In particular, you will:
- Write two simple tasks in C code
- Re-write the tasks using the CMSIS-DSP library functions.
- Learn how to time the code to evaluate the speed-up
- Bonus: Write the tasks in assembly to to evaluate the speed-up (or lack thereof) of hand
written assembly v/s C.

## Lab 2 - GPIO's and DAC
This lab introduces the STM32L4 Discovery kit IoT node development board that will be used for the remainder of the labs. This board features an STM32L4 series micro-controller based on the ARM Cortex-M4 CPU core, and includes an impressive list of sensors and peripherals. You will also be introduced to the STM32 Hardware Abstraction Layer (HAL) drivers for the family of STM32L4 devices. The HAL drivers are a collection of embedded software that provides a simple set of APIs to the programmer, thereby abstracting the hardware specific details for each device. You will use the HAL drivers to program the General Purpose Input/Output (GPIO) pins on the board in both digital and analog mode, and to control the on-chip Digital to Analog (DAC) converter. In particular, you will:
- Learn how to use the HAL drivers to configure a GPIO as a digital output to light up an on
board LED.
- Configure the GPIO associated with the blue push-button as a digital input to control the LED.
- Learn how to use the HAL drivers to configure and initialize the on-chip DAC.
- Generate a saw-tooth wave on the DAC output GPIO via software.
- Bonus: Use the extended DAC HAL drivers to generate a triangular wave.

## Lab 3&4 - UART, ADC and DMA
In this lab, you will learn how to use more peripherals on the STM32L475VG MCU. In particular, you will:
! WEEK 1
- Learn how to use UART to enable communication with a computer.
- Use an ADC to obtain the temperature of the processor core.
- Use the Systick timer interrupt to read the temperature.
- Monitor the temperature in a desktop application.
! WEEK 2
- Enable DMA on UART transmissions.
- Use DMA to send 10 temperature values at a time.

## Lab 5 - DAC, Timers and...
In this lab, you will learn how to use more peripherals on the STM32L475VG MCU. In particular, you will:
- Learn how to use UART to enable communication with a computer.
- Use Matlab to obtain an audio data from a recorded voice.
- Use the audio data in your MCU to generate audio on a headset.
- Utilize a timer as a source of time.

## Lab 6 - Microphones, DFSDM and DAC
In this lab, you will learn how use the MEMS microphones on the board, and send the stereo audio output to earphones via the MCU’s on-chip DAC. In particular, you will:
- Learn about Pulse Density Modulation (PDM), which is the output format of the microphones.
- Learn how to use the MCU’s on-chip Digital Filter for Sigma-Delta Modulators (DFSDM) to convert the PDM signal into a standard audio signal of desired resolution and sampling frequency.
- Send the stereo audio input from the microphone to the headphones via the DAC.
- Enable interrupts and DMA for input

## Lab 7 - OS and Peripherals
In this lab, you will learn how to use some of the peripheral sensors via drivers that have been
provided by STM, and learn how to use a real time operating system (RTOS). In particular, you will:
- Study the driver documentation in order to enable and acquire sensor data.
- Learn how to use FreeRTOS.
- Create individual OS threads to sample the different peripherals and perform I/O.
- Optimize power usage by using OS thread synchronization tools to suspend unnecessary threads.

## Project - Blind Source Separation using ICA
For the final project, you will use all the material you have learned over the course of the labs to
build an audio application that employs Blind Source Separation(BSS) using the Fast Independent
Components Analysis(FastICA) algorithm. Your application should record two audio signals via the
board microphone and attempt to separate them using FastICA. There is an initial deliverable due
the week of November 19th in order to ensure you are making sufficient progress to succeed in the
project. The project can be broken down into the following steps:
1. Generation and mixing of sine wave signals
2. FastICA implementation and testing via separating the mixed sine waves.
3. Separation of microphone stereo input using FastICA.
