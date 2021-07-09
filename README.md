# Linux-CAN
MAXON EPOS4 motor driver can be controlled by CAN (Contrl Area Network) communication and provides a software library "libEposCmd" which provides functions for easy motion control commands (ex. MoveVelocity, MovePosition, etc.). However, these library can be only used with the computers equipped with CAN interface hardware from certain vendors (Kvaser, IXAAT, etc.) and only supports CANopen SDO protocols. 

Therefore, in order to command EPOS4 via CAN from computers with unsupported CAN interface cards, or to perform faster communication using PDO protocols, one needs a code that can code/decode CAN packets according to the EPOS4's protocol, (which is similar to CiA40 standard) and send/read it using CAN interface card's API. 

This code shows how to build CAN packets for motion control commands and PDO communications and uses native SocketCAN interface of Linux kernel to send it out. 
