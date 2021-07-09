# Linux-CAN
Control MAXON EPOS4 motor driver through CAN.
This code uses native CAN socket of Linux kernel.
Socket only handles the physical CAN protocol communication, therefore, 
constructing CAN packet according to EPOS4's protocol, which is similar to CiA40 standard
is done in this code. 
