Hello! This is a PID demonstration device that allows a user to visualize the effects of Proportional, Integral, and Derivative control. It features 3 knobs for each respective PID component, which affects the trajectory of a servo motor.

<img width="1625" height="837" alt="Image" src="https://github.com/user-attachments/assets/ad77a3b3-4858-4768-9df0-e05648b403f0" />

The device utilizes a NUCLEO-F446RE STM32 development board and is programmed in the STM32CubeIDE environment, coded in C. Our goal was to implement FreeRTOS to allow parallel control of the PID knobs. Attached is a .mov file with a full demonstration of the device.
