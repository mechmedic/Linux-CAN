/**
 ***************************************************************************
 * \file  EposCAN.h
 * \brief Header file of C++ class for communicating with Maxon EPOS2/4
 * controller over CAN network. Builds SDO/PDO objects for motion control
 * commands and sends it over CAN using underlying CAN interface.
 * Change the defines to switch from one CAN interface to another.
 * Currently available CAN interfaces (CAN device + driver) are
 * 1. pi2can on raspberry pi (or other linux) + SocketCAN driver
 *     pi2can :  http://skpang.co.uk/dl/cantest.tar
 * 2. NTRexLab's USB2CAN and its driver (only available in Windows)
 *    USB2CAN : http://www.devicemart.co.kr/goods/view?no=1323536#goods_review)
 *
 * Last Updated : 2023.01.26 Chunwoo Kim (CKim)
 * Contact Info : cwkim@kist.re.kr
 ***************************************************************************
**/

#ifndef EPOSCAN_H
#define EPOSCAN_H

// CKim - Change definition to 1 for the CAN interface to be used.
#define USE_SOCKET_CAN  1
#define USE_USB_CAN     0

#if USE_SOCKET_CAN
    #include "linuxsocketcan.h"
    typedef LinuxSocketCAN canInterface;
#elif USE_USB_CAN
    #include "windowsusb2can.h"
    typedef WindowsUSBCAN canInterface;
#endif

#define NUM_NODE 1

#include <canslave.h>

// CKim - In linux, instead of Event, I'm using semaphore. So include following headers.
//#include <pthread.h>
//#include <semaphore.h>
#include <time.h>
#include <thread>
#include <memory>
#include <unistd.h>


// CKim - Class Encapsulating CAN interface of the Maxon EPOS2
class EposCAN
{

public:
	bool m_CANportOpen;	// Other status variables of the Node. 

    // CKim - Slaves
    CANSlave    m_Slave[NUM_NODE];

    // CKim - Motion limit parameters may be needed....

private:
    // CKim - Class encapsulating CAN interface. Switch between different CAN hardware setups
    static canInterface m_CANport;

    // CKim - ID of the EPOS controller in CAN
    int			m_nodeId;
    int			m_numNodes;

    // CKim - Mode of the EPOS controller (profile position, homing, velocity ...)
	int			m_mode;
    int         m_totalTxPdoNum;


    // CKim - PDO Specific.....
    char        m_TxPdoData[32];    // CKim - Store data from TxPDO1-4
    uint16_t	m_statusWord;
    uint16_t	m_prevStatusWord;
    uint32_t	m_Position;

    //sem_t       m_MotionSema;

public:
    EposCAN();
    ~EposCAN();
	
    int GetId()                 {	return m_nodeId;            }
    int GetMode()               {	return m_mode;              }
    std::string GetErrorMsg()   { return "";}//{   return m_CANport.m_errMsg;  }

    // -------------------------------------------
	// CKim - Open CAN port
    static int ConnectCANport(const char* portName)    {   return(m_CANport.OpenCANport(portName));  }

    // CKim - Close CAN port
    static int DisconnectCANport()  {   return(m_CANport.CloseCANport());     }

    // CKim - Send SYNC over entire network
    static int SendSYNC()           {   return(m_CANport.SendSYNC());         }

    /**
     * @brief Enable / Disable the device with nodeId
     * @param idx : Index of the Node. Different from the Node ID
     * @return 0 on sucess, otherwise -1
     */
    int EnableDevice(int idx);
    int DisableDevice(int idx);

    /**
     * @brief Enable / Disable all the device in network
     * @return 0 on sucess, otherwise -1
     */
    int EnableDeviceAll();
    int DisableDeviceAll();

    /**
     * @brief SetOperationMode of the device with nodeId.
     * Operation modes are defined as enum OP_MODE in the header
     * @param idx : Index of the Node. Different from the Node ID
     * @return 0 on sucess, otherwise -1
     */
    int SetOperationMode(int idx, int mode);
    int SetOperationModeAll(int mode);

    // -------------------------------------------

    /**
     * @brief Various read function using SDO read
     * @param idx : Index of the Node. Different from the Node ID
     * @return 0 on sucess, otherwise -1
     */
    uint16_t ReadControlWord(int idx);
    uint16_t ReadStatusWord(int idx);
    int ReadPosition(int idx);

    /**
     * @brief Halt motion by setting status word
     * @param idx : Index of the Node. Different from the Node ID
     * @return 0 on sucess, otherwise -1
     */
    int HaltAxis(int idx);
    int HaltAxisAll();

    // -------------------------------------------
	// CKim - Get/Set Position Profile Motion Parameter
    int GetPositionProfileParam(int idx, ProfilePosParam& P);
    int SetPositionProfileParam(int idx, const ProfilePosParam& P);

    // CKim - Move to target position
    int MovePosProfile(int idx, int32_t pos, bool rel);

    // CKim - Wait for Profile Motion Completion
    int WaitForMotionCompletion(int idx, long timeoutmsec);
    int WaitForMotionCompletionAll(long timeoutmsec);

    // -------------------------------------------
    // CKim - Get/Set Velocity Profile Motion Parameter
    int GetVelocityProfileParam(int idx, ProfileVelParam& P);
    int SetVelocityProfileParam(int idx, const ProfileVelParam& P);

    // CKim - Move in target velocity. Unit is in rpm
    int MoveVelProfile(int idx, int32_t vel);



    // CKim - Start PDO communication
    void StartPdoExchange();
    void StopPdoExchange();

    // -------------------------------------------
    // CKim - Get/Set Homing parameters
    int GetHomingParam(HomingParam& H);
    int SetHomingParam(const HomingParam& H);

	// CKim - Start Homing
	int StartHoming();
	bool IsHomingAttained();
    // -------------------------------------------


    // CKim - Get/Set PID gain of the position control loop
    int GetPositionControlGain(uint16_t& P, uint16_t& I, uint16_t& D);
    int SetPositionControlGain(const uint16_t& P, const uint16_t& I, const uint16_t& D);

        uint16_t GetStatus()        {	return m_statusWord;            }

    // -------------------------------------------
    // CKim - Move in Position Profile Mode. Motion starts immediately

    //int MovePosProfileUsingPDO(int32_t pos, bool rel);

    // CKim - Get current position
    //int GetPosition(int32_t& pos);
    //int GetPositionUsingPDO(int32_t& pos);
    // -------------------------------------------

private:

	// CKim - Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
    int SDO_write(SDO_data* d)  {   return m_CANport.SendSDO(d,SDO_WRITE);    };
    int SDO_read(SDO_data* d)   {   return m_CANport.SendSDO(d,SDO_READ);    };

    // -------------------------------------------
    // CKim - Enable/Disable PDO by sending NMT message
    int EnablePDO(int idx);
    int DisablePDO(int idx);

    // CKim - Write configured PDOs to slaves.
    int WriteTxPdoSettings(int idx);
    int WriteRxPdoSettings(int idx);

    // CKim - Hack these two functions to change parameters and mappings of PDO 1-4 of the device.
    // This configures PDO parameters and data mapping of the EPOS
    // TxPDO is from EPOS to PC. RxPDO is from PC to EPOS. Both 8 bytes
    int ConfigureTxPDOs(int idx);
    int ConfigureRxPDOs(int idx);

    // CKim - TxPDO Reading thread. Update this function when TxPDO map changes
    static void* TxPDOReadThread(void* pData);

    // CKim - RxPDO Sending thread. Update this function when TxPDO map changes
    static void* RxPDOSendThread(void* pData);

    // CKim - Cyclic PDO Exchange Thread. All TxPDOshould be configured to SYNC
    static void* CyclicPDOThread(void* pData);


    std::shared_ptr<std::thread> pthrSend;
    std::shared_ptr<std::thread> pthrRead;

    char* m_RxPDOsendBuff[NUM_NODE];

    bool sendFlag;
    bool readFlag;
};

#endif
