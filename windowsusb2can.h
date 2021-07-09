//// ------------------------------------------------------------------------------
//// CKim - 2019. Sep.16 : C++ class encapsulating CAN interface for Linux
//// that uses SocketCAN driver. Select this CAN interface class in EposCAN
//// class if you are using pi2can board + raspberry pi for your motor control.
//// SocketCAN is open source library for CAN communication that is part of linux kernel.
//// https://en.wikipedia.org/wiki/SocketCAN
//// https://www.kernel.org/doc/Documentation/networking/can.txt
////
//// This code is based on the example code provided by pi2can
//// http://skpang.co.uk/dl/cantest.tar
//// ------------------------------------------------------------------------------


//#ifndef LINUXSOCKETCAN_H
//#define LINUXSOCKETCAN_H

//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <string>
//#include <net/if.h>
//#include <sys/ioctl.h>

//// CKim - SocketCAN is a open source CAN library contributed by Volkswagen Research to Linux kernel.
//// Below headers are for SockeCAN library, which is included in the kernel of raspberry pi
//#include <linux/can.h>
//#include <linux/can/raw.h>

//// CKim - In linux, instead of Event, I'm using semaphore
//#include <pthread.h>
//#include <semaphore.h>
//#include <time.h>

//// CKim - Common header file for CAN class
//#include "candefines.h"

//class EposCAN;  // CKim - Forward declaration

//class LinuxSocketCAN
//{

//public:
////	bool m_CANportOpen;	// Other status variables of the Node.
////	bool m_PDOready;
//////	bool
//    std::string m_errmsg;
//    static std::string m_portName;

//private:
//    //static CAN_HANDLE	m_hd;       // CKim - USB2CAN
//    static int m_hd;                // CKim - Socket Handle. For pi2can

//    //static HANDLE		m_hSDO_Event;
//    //static HANDLE		m_hPDO_Event;
//    static sem_t        m_hSDO_Sema;
//    //static sem_t        m_hPDO_Sema;

//    //static HANDLE		m_hReadThrd;	// CKim - CAN reading loop
//    static pthread_t    m_hReadThrd;

//    static SDO_data		m_RcvSDOdata;


//    static EposCAN*     m_pDev[10];     // CKim - Stores pointers to device registered to this CAN

//public:
//    LinuxSocketCAN();
//    ~LinuxSocketCAN();


//public:

//    // CKim - Open/Close CAN port
//    static int OpenCANport(const char* portName);
//    static int CloseCANport();
//    void RegisterDevice(int n, EposCAN* ptr)    {    m_pDev[n] = ptr;   }

//    // CKim - Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
//    int SendSDO(SDO_data* d, int rw);

//    // CKim - Send SYNC object which is used for PDO transport
//    static int SendSYNC();

//    // CKim - Enable / Disable PDO. This is done by using Network Management (NMT) services.
//    // PC is the NMT Master and EPOS is NMT Slave. Slave needs to switch from 'Pre-operational'
//    // to 'Operational' state to send/receive PDO. Switch between the states is done by
//    // sending NMT messages, which has Communication Object ID 0 + 1 byte state command
//    // Set nodeId to 0 to put enanle/disable all nodes
//    int SendNMT(int state, int nodeId);

//    // CKim - Send RxPDO Object
//    int SendRxPDO(int COBID, int sz, char* buff);


//private:
//    static void* CAN_ReadThread(void* pData);
//};

//#endif // LINUXSOCKETCAN_H


#ifndef WINDOWSUSB2CAN_H
#define WINDOWSUSB2CAN_H

//#include <Windows.h>
//#include <tchar.h>
//#include <CAN_Access.h> // CKim - This is header for library using NTRexLab's USB2CAN module
//#include "CAN_Access.h"       // CKim - USB2CAN library header

class WindowsUSB2CAN
{
public:
    WindowsUSB2CAN();
};

#endif // WINDOWSUSB2CAN_H



//		if (CAN_Recv(m_hd, &id, &len, buf, &ext, &rtr))
//		{
           // printf("Received Object ID 0x%04X\n", id);
