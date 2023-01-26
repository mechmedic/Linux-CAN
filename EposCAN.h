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

// CKim - In linux, instead of Event, I'm using semaphore. So include following headers.
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <thread>

// CKim - This is EPOS2 Operation mode
enum OP_MODE {
    HOMING = 6,         // EPOS2,4 common
    PROFILE_VEL = 3,    // EPOS2,4 common
    PROFILE_POS = 1,    // EPOS2,4 common

    POSITION = -1,      // EPOS2 only
    VELOCITY = -2,      // EPOS2 only
    CURRENT = -3,       // EPOS2 only

    SYNC_POS = 8,       // Cyclic Synchronous Position Mode (EPOS4, corresponds to POSITION of EPOS2)
    SYNC_VEL = 9,       // Cyclic Synchronous Velocity Mode (EPOS4, corresponds to VELOCITY)
    SYNC_TRQ = 10       // Cyclic Synchronous Torque Mode (EPOS4, corresponds to CURRENT)
};


// CKim - Homing parameters
typedef struct
{
	uint32_t	MaxFollowingError;		// 6065 0
	uint32_t	MaxProfileVelocity;		// 607F 0
	uint32_t	QuickStopDecel;			// 6085 0
	uint32_t	SpeedForSwitchSearch;	// 6099 01
	uint32_t	SpeedForZeroSearch;		// 6099 02
	uint32_t	HomingAccel;			// 609A 0
	uint16_t	CurrentThresholdHoming;	// 2080		// Used when homing by touching mechanical limit and sensing current
	int32_t		HomeOffset;				// 607c		// Amount to move away from the sensed limit
	int8_t		HomingMethod;			// 6098
} HomingParam;

// CKim - Profile Position parameters
typedef struct
{
	uint32_t	MaxFollowingError;		// 6065 0
	uint32_t	MaxProfileVelocity;		// 607F 0
	uint32_t	QuickStopDecel;			// 6085 0
	uint32_t	ProfileVelocity;		// 6081 0
	uint32_t	ProfileAccel;			// 6083 0
	uint32_t	ProfileDecel;			// 6084 0
    uint16_t    MotionProfileType;      // 6086 0       // Always set to 0 (linear ramp) in EPOS4
} ProfilePosParam;

// CKim - Profile Velocity parameters
typedef struct
{
    uint32_t	MaxProfileVelocity;		// 607F 0
    uint32_t	QuickStopDecel;			// 6085 0
    uint32_t	ProfileAccel;			// 6083 0
    uint32_t	ProfileDecel;			// 6084 0
    uint16_t    MotionProfileType;      // 6086 0
} ProfileVelParam;


// CKim - Class Encapsulating CAN interface of the Maxon EPOS2
class EposCAN
{

public:
	bool m_CANportOpen;	// Other status variables of the Node. 
	bool m_PDOready;

    // CKim - Motion limit parameters may be needed....

private:
    // CKim - Class encapsulating CAN interface. Switch between different CAN hardware setups
    static canInterface m_CANport;

    // CKim - ID of the EPOS controller in CAN
    int			m_nodeId;

    // CKim - Mode of the EPOS controller (profile position, homing, velocity ...)
	int			m_mode;

    // CKim - PDO Specific.....
    char        m_TxPdoData[32];    // CKim - Store data from TxPDO1-4
	uint16_t	m_statusWord;
    uint16_t	m_prevStatusWord;
	uint32_t	m_Position;

    sem_t       m_MotionSema;

public:
    EposCAN(int n);
    ~EposCAN();
	
    int GetId()                 {	return m_nodeId;            }
    int GetMode()               {	return m_mode;              }
    std::string GetErrorMsg()   { return "";}//{   return m_CANport.m_errMsg;  }

    // -------------------------------------------
	// CKim - Open CAN port
    static int ConnectCANport(const char* portName)    {   return(m_CANport.OpenCANport(portName));  }

    // CKim - Close CAN port
    static int DisconnectCANport()   {   return(m_CANport.CloseCANport());     }

    // CKim - Send SYNC over entire network
    static int SendSYNC()       {   return(m_CANport.SendSYNC());         }

    // CKim - Enable the device
    int EnableDevice();
    int DisableDevice();

    //* CKim - Set Operation Mode
	int SetOperationMode(int mode);
    // -------------------------------------------

    // -------------------------------------------
    uint16_t GetStatus()        {	return m_statusWord;            }
    uint16_t ReadControlWord();
    uint16_t ReadStatusWord();
    // -------------------------------------------

    // -------------------------------------------
	// CKim - Get/Set PID gain of the position control loop
	int GetPositionControlGain(uint16_t& P, uint16_t& I, uint16_t& D);
	int SetPositionControlGain(const uint16_t& P, const uint16_t& I, const uint16_t& D);

	// CKim - Get/Set Position Profile Motion Parameter
	int GetPositionProfileParam(ProfilePosParam& P);
	int SetPositionProfileParam(const ProfilePosParam& P);

    // CKim - Move to target position
    int MovePosProfile(int32_t pos, bool rel);

    // CKim - Halt motion by setting status word
    int HaltAxis();

    // CKim - Wait for Profile Motion Completion
    int WaitForMotionCompletion(long timeoutmsec);

    // CKim - Get Position
    int ReadPosition();
    // -------------------------------------------

    // -------------------------------------------
    // CKim - Get/Set Velocity Profile Motion Parameter
    int GetVelocityProfileParam(ProfileVelParam& P);
    int SetVelocityProfileParam(const ProfileVelParam& P);

    // CKim - Move in target velocity. Unit is in rpm
    int MoveVelProfile(int32_t vel);
    // -------------------------------------------

    // -------------------------------------------
    // CKim - Get/Set Homing parameters
    int GetHomingParam(HomingParam& H);
    int SetHomingParam(const HomingParam& H);

	// CKim - Start Homing
	int StartHoming();
	bool IsHomingAttained();
    // -------------------------------------------

    // -------------------------------------------
    // CKim - Enable/Disable PDO by sending NMT message
    int EnablePDO();
    int DisablePDO();

    // CKim - Hack this function to change Transmit PDO 1-4 of the device.
    // This configures TxPDO and sets the mapping of the 8 bytes of data that is transmitted from EPOS to PC
	// Also hack CAN_ReadThread to decode the TxPDO accordingly
	int ConfigureTxPDO();
	
	// CKim - Hack this function to change Receive PDO 1-4 of the device.
	// This configures RxPDO and sets the mapping of the 8 bytes of data that EPOS receives from PC
	int ConfigureRxPDO();

//    // CKim - Set Transmit PDO Mapping - This configures n th TxPDO of nodeID,
//    int SetTxPDOMapping(int n);

//    // CKim - Set Transmit PDO Mapping - This configures n th TxPDO of nodeID,
//    int SetRxPDOMapping(int n);

    int NotifyPDO(int n, const char* buf);
    // -------------------------------------------

    // -------------------------------------------
    // CKim - Move in Position Profile Mode. Motion starts immediately

    int MovePosProfileUsingPDO(int32_t pos, bool rel);

    // CKim - Get current position
    int GetPosition(int32_t& pos);
    int GetPositionUsingPDO(int32_t& pos);
    // -------------------------------------------

private:

	// CKim - Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
    int SDO_write(SDO_data* d)  {   return m_CANport.SendSDO(d,SDO_WRITE);    };
    int SDO_read(SDO_data* d)   {   return m_CANport.SendSDO(d,SDO_READ);    };



    // CKim - PDO Cycling functions will be needed.    // CKim - Function that runs in separate thread and continuously read incoming CAN packets
    //    static void* CAN_ReadThread(void* pData);


};

#endif
