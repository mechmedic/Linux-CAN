// ------------------------------------------------------------------------------
// CKim - 2019. Sep.16 : C++ class encapsulating CAN interface for Linux
// that uses SocketCAN driver. Select this CAN interface class in EposCAN
// class if you are using pi2can board + raspberry pi for your motor control.
// SocketCAN is open source library for CAN communication that is part of linux kernel.
// https://en.wikipedia.org/wiki/SocketCAN
// https://www.kernel.org/doc/Documentation/networking/can.txt
//
// This code is based on the example code provided by pi2can
// http://skpang.co.uk/dl/cantest.tar
// ------------------------------------------------------------------------------

#ifndef LINUXSOCKETCAN_H
#define LINUXSOCKETCAN_H

// CKim - SocketCAN is a open source CAN library contributed by Volkswagen Research to Linux kernel.
// Below headers are for SockeCAN library, which is included in the kernel of raspberry pi
#include <linux/can.h>
#include <linux/can/raw.h>

// CKim - Other Linux network headers
#include <net/if.h>
#include <sys/ioctl.h>

// CKim - In linux, instead of Event, I'm using semaphore. So include following headers.
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

// CKim - C++ string
#include <string>

// CKim - Common header file for CAN class
#include "candefines.h"


// CKim - Forward declaration of the EPOS motor controller class. CAN interface needs to
// have pointer of the slave devices (EPOS) registred in the network to pass received PDOs.
class EposCAN;


class LinuxSocketCAN
{

public:
    std::string         m_errMsg;
    static std::string  m_portName;

private:
    static int          m_hd;           // CKim - Socket Handle. -1 if not initialized. For pi2can
    static sem_t        m_hSDO_Sema;    // CKim - Semaphore for signaling SDO receive
    static pthread_t    m_hReadThrd;    // CKim - Thread receiving incoming CAN data

    static SDO_data		m_RcvSDOdata;   // CKim - Received SDO data
    static EposCAN*     m_pDev[10];     // CKim - Stores pointers to device registered to this CAN

public:
    LinuxSocketCAN();
    ~LinuxSocketCAN();

    // CKim - Open/Close CAN port. return -1 on error
    static int OpenCANport(const char* portName);
    static int CloseCANport();

    // CKim - Send SYNC object to all the devices in network. This is used for
    // synchronizing PDO transport of all the connected devices.
    static int SendSYNC();

    // CKim - Register EPOS slave to this CAN network
    void RegisterDevice(int n, EposCAN* ptr)    {    m_pDev[n] = ptr;   }

    // CKim - Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
    int SendSDO(SDO_data* d, int rw);

    // CKim - Enable / Disable PDO. This is done by using Network Management (NMT) services.
    // PC is the NMT Master and EPOS is NMT Slave. Slave needs to switch from 'Pre-operational'
    // to 'Operational' state to send/receive PDO. Switch between the states is done by
    // sending NMT messages, which has Communication Object ID 0 + 1 byte state command
    // Set nodeId to 0 to put enanle/disable all nodes
    int SendNMT(int state, int nodeId);

    // CKim - Send RxPDO Object
    int SendRxPDO(int COBID, int sz, char* buff);


private:
    // CKim - Function that runs in separate thread and continuously read incoming CAN packets
    static void* CAN_ReadThread(void* pData);
};

#endif // LINUXSOCKETCAN_H
