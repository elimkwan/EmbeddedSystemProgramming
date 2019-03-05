# EmbeddedSystemProgramming

main_program.c gives a overview of what the program do. To lauch the program on Code Composer Studio, please download the zip file.

The robot is programmed to have two mode of operation, Autonomous mode and Free-motion mode. In Autonomous mode, the robot operates using predefined route and immediately stop when any of bump switches are touched. In Free-motion mode, the robot freely move forward but will change direction of movement according to the interrupted bump switches. By implementing in Real Time Operating System, music would be played simultaneously when these tasks are performed.

Key Concepts:
Difference between Interrupt latency and Polling latency  
Real-time operating system and Scheduling mechanism in RTOS
Use of semaphore
