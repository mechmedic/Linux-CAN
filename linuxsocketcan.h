/**
 ***************************************************************************
 * \file  linuxsocketcan.h
 * \brief Header file of C++ class encapsulating CAN interface
 * for Linux that uses SocketCAN driver. SocketCAN is open source library for
 * CAN communication that is part of linux kernel.
 * Select this CAN interface class in EposCAN class if you are using
 * pi2can board + raspberry pi for your motor control.
 *
 * This code is based on the example code from
 * http://skpang.co.uk/dl/cantest.tar
 * https://github.com/linux-can/can-utils
 *
 * For more information on SocketCAN, see
 * https://en.wikipedia.org/wiki/SocketCAN
 * https://www.kernel.org/doc/Documentation/networking/can.txt

 * Last Updated : 2023.01.26 Chunwoo Kim (CKim)
 * Contact Info : cwkim@kist.re.kr
 ***************************************************************************
**/




#ifndef LINUXSOCKETCAN_H
#define LINUXSOCKETCAN_H

// CKim - SocketCAN is a open source CAN library contributed by Volkswagen Research to Linux kernel.
// Below headers are for SockeCAN library, which is included in the kernel of raspberry pi
#include <linux/can.h>
#include <linux/can/raw.h>

// CKim - Other Linux network headers
#include <net/if.h>
#include <sys/ioctl.h>

// CKim - C++ string
#include <string>

// CKim - Common header file for CANopen
#include "CANopenDefs.h"

class LinuxSocketCAN
{

private:

    /// std::string for error messages. Accessed by GetError() function
    std::string         m_errMsg;

    /// Name of CAN port
    std::string  m_portName;

    /// Socket Handle. -1 if not initialized. For pi2can
    static int m_hd;

public:
    LinuxSocketCAN();
    ~LinuxSocketCAN();

    /**
     * @brief Opens CAN port and configures communication.
     * @param portName Port name ex) "CAN0"
     * @return 0 on sucess, otherwise -1
     */
    int OpenCANport(const char* portName);

    /**
     * @brief Closes CAN port
     * @return -1 on error, otherwise 0.
     */
    int CloseCANport();

    /**
     * @brief Send SYNC object to all the devices in network.
     * This is used for synchronizing PDO transport of all the connected devices.C
     */
    int SendSYNC();

    /**
     * @brief Send Network Management (NMT) message.
     * PC is the NMT Master and EPOS is NMT Slave. Slave needs to switch from 'Pre-operational'
     * to 'Operational' state to send/receive PDO. Switch between the states is done by
     * sending NMT messages, which has Communication Object ID 0 + 1 byte state command
     * This function is called to Enable / Disable PDO.
     * @param nodeId 0 to change state of all nodes
     */
    int SendNMT(int state, int nodeId);

    /**
     * @brief  Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
     * @param data Data to send, also stores received SDO data
     *
     * Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
     * CAN Master PC is Client and the connected slave devices (EPOS Controller) are Server.
     * Client (master) sends SDO to server (slave) to
     * 1. rw == SDO_WRITE to write data to the Object Dictionary of the EPOS (slave)
     * 2. rw == SDO_READ to read from the Object Dictionary of the EPOS (slave)
     * @return 0 on sucess, otherwise -1
     */
    int SendSDO(SDO_data* data, int rw);

    /**
     * @brief  Send RxPDO Object
     */
    int SendRxPDO(int COBID, int sz, char* buff);

    /**
     * @brief  Read TxPDO Object
     */
    int ReadTxPDO(int& nodeID, int& PdoId, char* buff);

    /**
     * @brief  Get Error message
     */
    void GetError(std::string& msg) {   msg = m_errMsg; }

private:
    /**
     * @brief  Parse CAN Frame into SDO_data
     */
    int FrameToSdo(const struct can_frame& frame, SDO_data* sdo);

};

#endif // LINUXSOCKETCAN_H
