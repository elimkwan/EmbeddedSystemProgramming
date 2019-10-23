# Embedded and Real Time System Programming

main_program.c gives an overview of what the program do. dcMotor.c gives an overview of how to use bitwise operations to control the motor in TI-RSLK Kit Set. To lauch the program on Code Composer Studio, please download the zip file.

# Polling Vs Interrupt
The robot is programmed to have two mode of operation, Autonomous mode and Free-motion mode using both Polling and Interrupt techniques. In Autonomous mode, the robot operates using predefined route and immediately stop when any of bump switches are touched. In Free-motion mode, the robot freely move forward but will change direction of movement according to the interrupted bump switches. 

# Real Time Operating System with Round Robin Algorithm
By implementing in Real Time Operating System, music would be played simultaneously when these tasks are performed.

# Key Concepts:
- Difference between Interrupt latency and Polling latency  
- Real-time operating system and Scheduling mechanism in RTOS
- Use of semaphore
