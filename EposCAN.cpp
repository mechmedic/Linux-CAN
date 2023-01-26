#include <stdio.h>
#include <string.h>
#include "EposCAN.h"



// CKim - Static Variable initialization
//EposCAN* EposCAN::m_pDev[10] = { 0 };
canInterface EposCAN::m_CANport;

EposCAN::EposCAN(int n)
{
    m_nodeId = n;


    //    // ---------------------------------------------------------------
    //    // CKim - Launch CAN Read Thread
    //    pthread_create(&m_hReadThrd, NULL, LinuxSocketCAN::CAN_ReadThread, 0);
    //    // ---------------------------------------------------------------
}

EposCAN::~EposCAN()
{
    //close(m_hd);
}

// CKim - PC (client) sends SDO packets to EPOS (server) to write the dictionary of the EPOS
int EposCAN::SDO_write(SDO_data* d)
{
    int ret = m_CANport.SendSDO(d,SDO_WRITE);

    if(ret==-1) {   return 1;   }
    else        {   return 0;   }

}

// CKim - PC (client) sends SDO packets to EPOS (server) to read the dictionary of the EPOS
int EposCAN::SDO_read(SDO_data* d)
{
    int ret = m_CANport.SendSDO(d,SDO_READ);
    if(ret==-1) {   return 1;   }
    else        {   return 0;   }
}

// CKim - Enable EPOS.
int EposCAN::EnableDevice()
{
    // CKim - When the device is turned on it is in "Switch On Disabled" state. Status word (0x6041) being x0xx xxx1 x100 0000 = 0x0540
    // The state must be transited to  "Switch On Disabled" -> "Ready to Switch On" -> "Switched On" -> "Enable"
    // by writing appropriate Control word (0x6040)
    SDO_data pack;

    // CKim - "Switch On Disabled" -> "Ready to Switch On" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x110. For example 0000 0110 = 0x06
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0006;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    // CKim -  "Ready to Switch On" -> "Switched On" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x111. For example 0000 0111 = 0x07
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0007;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    // CKim -  "Switched On" -> "Enable" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x111. For example 0000 0111 = 0x0F
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x000F;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    return 0;
}

// CKim - Disable EPOS.
int EposCAN::DisableDevice()
{
    // CKim - To turn off the device, state must be transited to
    // "Enable" -> "Switched On" -> "Ready to Switch On" -> "Switch On Disabled"
    // by writing appropriate Control word (0x6040)
    SDO_data pack;

    // CKim - "Enable" -> "Switched On" transition (torque is disabled) happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx 0111. For example 0000 0111 = 0x07
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0007;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    // CKim -  "Switched On" -> "Ready to Switch On" transition (power is disabled) happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x110. For example 0000 0110 = 0x06
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0006;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    // CKim - "Ready to Switch On" -> "Switch On Disabled" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx xx0x. For example 0000 0000 = 0x00
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0000;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    return 0;
}

// CKim - Set Operating modes needs to be updated for EPOS4
int EposCAN::SetOperationMode(int mode)
{
    SDO_data pack;
    pack.nodeid = m_nodeId;	 pack.index = 0x6060;	pack.subindex = 0x00;	pack.sz = 1;

    if ((mode == HOMING) || (mode == PROFILE_POS) || (mode == PROFILE_VEL) || (mode == POSITION)
        || (mode == VELOCITY) || (mode == CURRENT) || (mode == SYNC_POS) || (mode == SYNC_VEL) || (mode == SYNC_TRQ))
    {
        pack.data = mode;
        if (!SDO_write(&pack))
        {
            m_mode = mode;
            return 0;
        }
        else { return -1; }
    }
    else
    {
        return -1;
    }
}

uint16_t EposCAN::ReadControlWord()
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    pack.index = 0x6040;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ return pack.data; }
    else	{ return -1; }
}

uint16_t EposCAN::ReadStatusWord()
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    pack.index = 0x6041;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ return pack.data; }
    else	{ return -1; }
}

// -----------------------------------------
int EposCAN::GetPositionControlGain(uint16_t& P, uint16_t& I, uint16_t& D)
{
	SDO_data pack;
	pack.nodeid = m_nodeId;	 pack.index = 0x60FB;	pack.subindex = 0x01;

	// returns 0 when successful
	if (!SDO_read(&pack))	{ P = pack.data; }
	else	{ return -1; }

	pack.subindex = 0x02;
	if (!SDO_read(&pack))	{ I = pack.data; }
	else	{ return -1; }

	pack.subindex = 0x03;
	if (!SDO_read(&pack))	{ D = pack.data; }
	else	{ return -1; }
	return 0;
}

int EposCAN::SetPositionControlGain(const uint16_t& P, const uint16_t& I, const uint16_t& D)
{
	SDO_data pack;
	pack.nodeid = m_nodeId;	 pack.index = 0x60FB;	pack.sz = 2;
	// returns 0 when successful

	pack.subindex = 0x01;		pack.data = P;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.subindex = 0x02;		pack.data = I;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.subindex = 0x03;		pack.data = D;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }
	return 0;
}

int EposCAN::GetPositionProfileParam(ProfilePosParam& P)
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    // returns 0 when successful

    pack.index = 0x6065;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.MaxFollowingError = pack.data; }
    else	{ return -1; }

    pack.index = 0x607F;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.MaxProfileVelocity = pack.data; }
    else	{ return -1; }

    pack.index = 0x6085;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.QuickStopDecel = pack.data; }
    else	{ return -1; }

    pack.index = 0x6081;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.ProfileVelocity = pack.data; }
    else	{ return -1; }

    pack.index = 0x6083;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.ProfileAccel = pack.data; }
    else	{ return -1; }

    pack.index = 0x6084;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.ProfileDecel = pack.data; }
    else	{ return -1; }

    pack.index = 0x6086;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.MotionProfileType = pack.data; }
    else	{ return -1; }

    return 0;
}

int EposCAN::SetPositionProfileParam(const ProfilePosParam& P)
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    // returns 0 when successful

    pack.index = 0x6065;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.MaxFollowingError;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x607F;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.MaxProfileVelocity;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6085;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.QuickStopDecel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6081;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.ProfileVelocity;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6083;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.ProfileAccel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6084;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.ProfileDecel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    return 0;
}

int EposCAN::MovePosProfile(int32_t pos, bool rel)
{
    SDO_data pack;

    // CKim - EPOS4 only : Set control word to 0x000F to receive new setpoint
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;     pack.data = 0x000F;
    if (!SDO_write(&pack))	{ }
    else					{ return -1; }

    // CKim - Write new target position value
    pack.nodeid = m_nodeId;     pack.index = 0x607A;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = pos;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    // CKim - Set control word so that positioning starts immedeately
    // Control word for Position Profile mode
    // bit 4-7 : 4. 1 New Set Point  5. (0 finish positioning before change setpoint. 1. immedeately change setpoint)
    // 6. 0 absolute, 1 relative, 7. Fault Reset (1 indicates fault),
    // bit 8-11 : 8. Halt
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;
    if (rel) 	{ pack.data = 0x007F; }
    else		{ pack.data = 0x003F; }

    if (!SDO_write(&pack))	{ return 0; }
    else					{ return -1; }
}

int EposCAN::HaltAxis()
{
    SDO_data pack;

    // CKim - Set bit 8 of the control word to stop motion
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;
    pack.data = 0x010F;

    if (!SDO_write(&pack))	{ return 0; }
    else					{ return -1; }
}

int EposCAN::WaitForMotionCompletion(long timeoutmsec)
{
    long sec = timeoutmsec/1000;
    long nsec = (timeoutmsec%1000)*1e6;
    struct timespec ts;     int err = 0;
    clock_gettime(CLOCK_REALTIME,&ts);
    ts.tv_sec += sec;
    ts.tv_nsec += nsec;
    err = sem_timedwait(&m_MotionSema,&ts);
    if(err == -1)   {
        //perror("Semaphore");
        printf("WaitForMotion time out\n");
        return -1;
    }
    return 1;
}

int EposCAN::ReadPosition()
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    pack.index = 0x6064;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ return pack.data; }
    else	{ return -1; }
}
// -----------------------------------------

// -----------------------------------------
int EposCAN::GetVelocityProfileParam(ProfileVelParam& P)
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    // returns 0 when successful

    pack.index = 0x607F;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.MaxProfileVelocity = pack.data; }
    else	{ return -1; }

    pack.index = 0x6085;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.QuickStopDecel = pack.data; }
    else	{ return -1; }

    pack.index = 0x6083;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.ProfileAccel = pack.data; }
    else	{ return -1; }

    pack.index = 0x6084;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.ProfileDecel = pack.data; }
    else	{ return -1; }

    pack.index = 0x6086;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ P.MotionProfileType = pack.data; }
    else	{ return -1; }

    return 0;

}

int EposCAN::SetVelocityProfileParam(const ProfileVelParam& P)
{
    SDO_data pack;
    pack.nodeid = m_nodeId;

    // returns 0 when successful

    pack.index = 0x607F;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.MaxProfileVelocity;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6085;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.QuickStopDecel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6083;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.ProfileAccel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6084;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = P.ProfileDecel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    pack.index = 0x6086;	pack.subindex = 0x00;   pack.sz = 2;	pack.data = P.MotionProfileType;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    return 0;
}

// CKim - Move in target velocity
int EposCAN::MoveVelProfile(int32_t vel)
{
    SDO_data pack;

    // CKim - Write new target velocity value. Object Index is 0x60FF
    pack.nodeid = m_nodeId;     pack.index = 0x60FF;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = vel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    // CKim - Set control word to 0x000F to start moving
    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;
    pack.data = 0x000F;

    if (!SDO_write(&pack))	{ return 0; }
    else					{ return -1; }

}
// -----------------------------------------

// -----------------------------------------
int EposCAN::GetHomingParam(HomingParam& H)
{
	SDO_data pack;
	pack.nodeid = m_nodeId;

	// returns 0 when successful

	pack.index = 0x6065;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.MaxFollowingError = pack.data; }
	else	{ return -1; }

	pack.index = 0x607F;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.MaxProfileVelocity = pack.data; }
	else	{ return -1; }

	pack.index = 0x6085;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.QuickStopDecel = pack.data; }
	else	{ return -1; }

	pack.index = 0x6099;	pack.subindex = 0x01;
	if (!SDO_read(&pack))	{ H.SpeedForSwitchSearch = pack.data; }
	else	{ return -1; }

	pack.index = 0x6099;	pack.subindex = 0x02;
	if (!SDO_read(&pack))	{ H.SpeedForZeroSearch = pack.data; }
	else	{ return -1; }

	pack.index = 0x609A;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.HomingAccel = pack.data; }
	else	{ return -1; }

	pack.index = 0x2080;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.CurrentThresholdHoming = pack.data; }
	else	{ return -1; }

	pack.index = 0x607c;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.HomeOffset = pack.data; }
	else	{ return -1; }

	pack.index = 0x6098;	pack.subindex = 0x00;
	if (!SDO_read(&pack))	{ H.HomingMethod = pack.data; }
	else	{ return -1; }

	return 0;
}

int EposCAN::SetHomingParam(const HomingParam& H)
{
	SDO_data pack;
	pack.nodeid = m_nodeId;

	// returns 0 when successful

	pack.index = 0x6065;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = H.MaxFollowingError;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x607F;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = H.MaxProfileVelocity;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x6085;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = H.QuickStopDecel;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x6099;	pack.subindex = 0x01;	pack.sz = 4;	pack.data = H.SpeedForSwitchSearch;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x6099;	pack.subindex = 0x02;	pack.sz = 4;	pack.data = H.SpeedForZeroSearch;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x609A;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = H.HomingAccel;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x2080;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = H.CurrentThresholdHoming;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x607c;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = H.HomeOffset;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	pack.index = 0x6098;	pack.subindex = 0x00;	pack.sz = 1;	pack.data = H.HomingMethod;
	if (!SDO_write(&pack))	{}
	else	{ return -1; }

	return 0;
}

int EposCAN::StartHoming()
{
    // CKim - Need some status check code.......

    // CKim - Control word for Homing mode
    // bit 4-7 : 4. Homing Start, 7. Fault Reset (1 indicates fault),
    // bit 8-11 : 8. Halt
    // To start homing bit 4 transition from 0 -> 1
    SDO_data pack;

    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x000F;
    if (!SDO_write(&pack))	{}
    else						{ return -1; }

    pack.nodeid = m_nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x001F;
    if (!SDO_write(&pack))	{ return 0; }
    else						{ return -1; }
}

bool EposCAN::IsHomingAttained()
{
    if (m_mode != HOMING)		{	return false;		}
    if (GetStatus() & 0x1000)	// Bit 12 of the status word is 1 when homed
    {
        return true;
    }
    else
    {
        return false;
    }
}
// -----------------------------------------

// -----------------------------------------
// CKim - Enable PDO by setting slave to operational state using NMT message
int EposCAN::EnablePDO()
{
    m_CANport.SendNMT(NMT_OPERATIONAL,m_nodeId);
}

// CKim - Disable PDO by setting slave to pre-operational state using NMT message
int EposCAN::DisablePDO()
{
    m_CANport.SendNMT(NMT_PREOPERATIONAL,m_nodeId);
}

// CKim - TxPDO is from EPOS to PC
int EposCAN::ConfigureTxPDO()
{
	SDO_data pack;		pack.nodeid = m_nodeId;

	// CKim - Changes to PDO mapping is only possible in Preoperational mode

	// CKim - My Configuration is
	// TxPDO1 : 16 bit 'Status word' (0x6041 Sub 00), Transmit on change
    // TxPDO2 : 32 bit 'Position Actual Value' (0x6064 Sub 00), Transmit on RTR
	// TxPDO3 : None
	// TxPDO4 : None

	for (int n = 1; n < 4; n++)
	{
		int cobId;

		// CKim - TxPOD1
		if (n == 1)
		{
			// CKim - 1. Configure TxPDO settings. Item 0x1800~03
			pack.index = 0x1800 + (n - 1);

			// SubIndex 0x01 (uint_32) : Communication Object ID (COB-ID)
			pack.subindex = 0x01;		pack.sz = 4;		pack.data = TX_PDO1 + m_nodeId;
			if (!SDO_write(&pack))	{ cobId = pack.data; }
			else					{ return -1; }

			// SubIndex 0x02 (uint_8) : Transmission type. Transmit on  1. Sync, 253. asynch transmission by RTR, 255. asynch transmission by Change
			pack.subindex = 0x02;	pack.sz = 1;	pack.data = 255;
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// SubIndex 03 (uint_16) : Inhibit time. Minimum interval between event trriggered PDO transmission. 
			// Use only when SubIndex 02 is 255 (transmit on change)
            pack.subindex = 0x03;	pack.sz = 2;	pack.data = 10;//1000;	// multiples of 100us = 100 ms.
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// CKim - 2. Configure TxPDO Map. Item 0x1A00~03
			pack.index = 0x1A00 + (n - 1);

			// Disable PDO by setting number of objects mapped Index 0x1A00~03 Sub 0x00 uint_8, to zero
			pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// Write 4 byte info of the nth object (Sub 0x0n) that will be transmitted by PDO.
			// 4 byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
			// Size in bits (0x08, 0x10, 0x20)
			pack.subindex = 0x01;	pack.sz = 4;
			pack.data = 0x60410010; // (0x6041 sub 00 : Status word, size is 0x10 16 bit . )
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// Enable PDO by setting number of objects mapped to nonzero
            pack.subindex = 0x00;	pack.sz = 1;	pack.data = 1;  // CKim changed to 0 from 1
			if (!SDO_write(&pack))	{}
			else					{ return -1; }
		}
		if (n == 2)
		{
			// CKim - 1. Configure TxPDO settings. Item 0x1800~03
			pack.index = 0x1800 + (n - 1);

			// SubIndex 0x01 (uint_32) : Communication Object ID (COB-ID)
			pack.subindex = 0x01;		pack.sz = 4;		pack.data = TX_PDO2 + m_nodeId;
			if (!SDO_write(&pack))	{ cobId = pack.data; }
			else					{ return -1; }

			// SubIndex 0x02 (uint_8) : Transmission type. Transmit on  1. Sync, 253. asynch transmission by RTR, 255. asynch transmission by Change
            pack.subindex = 0x02;	pack.sz = 1;	pack.data = 253;//1;
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// SubIndex 03 (uint_16) : Inhibit time. Minimum interval between event trriggered PDO transmission. 
			// Use only when SubIndex 02 is 255 (transmit on change)
			pack.subindex = 0x03;	pack.sz = 2;	pack.data = 1000;	// multiples of 100us = 100 ms. 
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// CKim - 2. Configure TxPDO Map. Item 0x1A00~03
			pack.index = 0x1A00 + (n - 1);

			// Disable PDO by setting number of objects mapped Index 0x1A00~03 Sub 0x00 uint_8, to zero
			pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// Write 4 byte info of the nth object (Sub 0x0n) that will be transmitted by PDO.
			// 4 byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
			// Size in bits (0x08, 0x10, 0x20)
			pack.subindex = 0x01;	pack.sz = 4;
			pack.data = 0x60640020; // (0x6064 sub 00 : Position actual value, size is 0x20 32 bit . )
			if (!SDO_write(&pack))	{}
			else					{ return -1; }

			// Enable PDO by setting number of objects mapped to nonzero
            pack.subindex = 0x00;	pack.sz = 1;	pack.data = 1;      // CKim changed to 0 from 1
			if (!SDO_write(&pack))	{}
			else					{ return -1; }
		}
		//if (n == 3) { pack.data = TX_PDO3 + m_nodeId; }
		//if (n == 4) { pack.data = TX_PDO4 + m_nodeId; }
	}

	return 0;//	return cobId;

}

// CKim - RxPDO is from PC to EPOS
int EposCAN::ConfigureRxPDO()
{
    SDO_data pack;		pack.nodeid = m_nodeId;

    // CKim - Changes to PDO mapping is only possible in Preoperational mode

    // CKim - My Configuration is
    // RxPDO1 : 16 bit 'ControlWord' (0x6040 Sub 00), Apply Immediately
    // RxPDO2 : 32 bit 'Target Position' (0x607A Sub 00), Apply Immediately
    // RxPDO3 : None
    // RxPDO4 : None

    for (int n = 1; n < 4; n++)
    {
        int cobId;

        // CKim - RxPOD1
        if (n == 1)
        {
            // CKim - 1. Configure RxPDO settings. Item 0x1400~03
            pack.index = 0x1400 + (n - 1);

            // SubIndex 0x01 (uint_32) : Communication Object ID (COB-ID)
            pack.subindex = 0x01;		pack.sz = 4;		pack.data = RX_PDO1 + m_nodeId;
            if (!SDO_write(&pack))	{ cobId = pack.data; }
            else					{ return -1; }

            // SubIndex 0x02 (uint_8) : Transmission type. Received data applied to node 1. on Sync, 255. Immediatey
            pack.subindex = 0x02;	pack.sz = 1;	pack.data = 255;
            if (!SDO_write(&pack))	{}
            else					{ return -1; }

            // CKim - 2. Configure RxPDO Map. Item 0x1600~03
            pack.index = 0x1600 + (n - 1);

            // Disable PDO by setting number of objects mapped Index 0x1600~03 Sub 0x00 uint_8, to zero
            pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
            if (!SDO_write(&pack))	{}
            else					{ return -1; }

            // Write 4 byte info of the nth object (Sub 0x0n) that will be transmitted by PDO.
            // 4 byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
            // Size in bits (0x08, 0x10, 0x20)
            pack.subindex = 0x01;	pack.sz = 4;
            pack.data = 0x60400010; // (0x6040 sub 00 : Control word, size is 0x10 16 bit . )
            if (!SDO_write(&pack))	{}
            else					{ return -1; }

            // Enable PDO by setting number of objects mapped to nonzero
            pack.subindex = 0x00;	pack.sz = 1;	pack.data = 1;
            if (!SDO_write(&pack))	{}
            else					{ return -1; }
        }
        if (n == 2)
        {
            // CKim - 1. Configure RxPDO settings. Item 0x1400~03
            pack.index = 0x1400 + (n - 1);

            // SubIndex 0x01 (uint_32) : Communication Object ID (COB-ID)
            pack.subindex = 0x01;		pack.sz = 4;		pack.data = RX_PDO2 + m_nodeId;
            if (!SDO_write(&pack))	{ cobId = pack.data; }
            else					{ return -1; }

            // SubIndex 0x02 (uint_8) : Transmission type. Received data applied to node 1. on Sync, 255. Immediatey
            pack.subindex = 0x02;	pack.sz = 1;	pack.data = 255;
            if (!SDO_write(&pack))	{}
            else					{ return -1; }

            // CKim - 2. Configure RxPDO Map. Item 0x1600~03
            pack.index = 0x1600 + (n - 1);

            // Disable PDO by setting number of objects mapped Index 0x1600~03 Sub 0x00 uint_8, to zero
            pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
            if (!SDO_write(&pack))	{}
            else					{ return -1; }

            // Write 4 byte info of the nth object (Sub 0x0n) that will be transmitted by PDO.
            // 4 byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
            // Size in bits (0x08, 0x10, 0x20)
            pack.subindex = 0x01;	pack.sz = 4;
            pack.data = 0x607A0020; // (0x607A sub 00 : Targetposition, size is 0x20 32 bit
            if (!SDO_write(&pack))	{}
            else					{ return -1; }

            // Enable PDO by setting number of objects mapped to nonzero
            pack.subindex = 0x00;	pack.sz = 1;	pack.data = 1;
            if (!SDO_write(&pack))	{}
            else					{ return -1; }
        }
        //if (n == 3) { pack.data = TX_PDO3 + m_nodeId; }
        //if (n == 4) { pack.data = TX_PDO4 + m_nodeId; }
    }

    return 0;//	return cobId;
}

int EposCAN::NotifyPDO(int n, const char* buff)
{
    memcpy(m_TxPdoData+(8*n),buff,8);

    // ------------------------------------------------------
    // CKim - TxPDO1 is configured to be 16 bit status word
    if(n==0)
    {
        uint16_t status;	char* tmp = (char*)&status;
        for (int i = 0; i < 2; i++)	{ *(tmp + i) = buff[i]; }
        m_statusWord = status;

        // CKim - Notify motion complete. In profile position mode, bit [0-15] of the statusword
        // bit 12 and bit 10 is both 1;
        if((status & 0x1400) == 0x1400)
        {
            sem_post(&m_MotionSema);
        }
    }
    // ------------------------------------------------------

//    // ------------------------------------------------------
//    // CKim - TxPDO2 is configured to be 32 bit Position actual value
//    uint32_t pos;       char* tmp = (char*)&pos;
//    for (int i = 0; i < 4; i++)	{ *(tmp + i) = buf[i]; }

//    // CKim - Copy to member variable - may need mutex here
//    if (m_pDev[nodeId - 1])
//    {
//        m_pDev[nodeId - 1]->m_Position = pos;
//        SetEvent(m_hPDO_Event);
//    }
//    // ------------------------------------------------------
}
// -----------------------------------------

// -----------------------------------------

int EposCAN::MovePosProfileUsingPDO(int32_t pos, bool rel)
{
    uint16_t cobId, cword;  char* tmp;  char sendBuff[4];   int err;

//    // CKim - Write new Control Word : RxPDO1 0x6040  Sub 00 16 bit. Control Word
//    cobId = RX_PDO1 + m_nodeId;
//    tmp = (char*)&cword;
//    cword = 0x000F;
//    for (unsigned int i = 0; i<2; i++) {
//        sendBuff[i] = *(tmp + i);    }

//    err = m_CANport.SendRxPDO(cobId, 2, sendBuff);
//    if (err == -1)	{ return -1; }

    // CKim - Write new target position value to RxPDO2 0x607A  Sub 00 32 bit. Target Position
    cobId = RX_PDO2 + m_nodeId;
    tmp = (char*)&pos;
    for (unsigned int i = 0; i<4; i++)    {
        sendBuff[i] = *(tmp + i);    }

    err = m_CANport.SendRxPDO(cobId, 4, sendBuff);
    if (err == -1)	{ return -1; }

    // CKim - Write new Control Word : RxPDO1 0x6040  Sub 00 16 bit. Control Word
    cobId = RX_PDO1 + m_nodeId;
    tmp = (char*)&cword;
    if (rel) 	{ cword = 0x007F; }
    else		{ cword = 0x003F; }
    for (unsigned int i = 0; i<2; i++) {
        sendBuff[i] = *(tmp + i);    }

    err = m_CANport.SendRxPDO(cobId, 2, sendBuff);
    if (err == -1)	{ return -1; }

    return 0;
 }

int EposCAN::GetPositionUsingPDO(int32_t& pos)
{
    // CKim - Send RTR. Request device to send its position, configured in TxPDO2
    // RTR is made by sending COB-ID of TxPDO2 with empty data with RTR bit set to 1
    // The bit is set by CAN_Send() with last argument rtr set to 1.
    uint16_t cobID = TX_PDO2 + m_nodeId;    char myBuff[8];
//    int err = CAN_Send(m_hd, cobID, 0, myBuff, 0, 1);

//    if (err == 0)	{ m_errmsg = "Send error";  return -1;  }

    // CKim - Wait for PDO receive event
//    DWORD res = WaitForSingleObject(m_hPDO_Event, 1000);
//    if (res != WAIT_OBJECT_0)	{ m_errmsg = "Event Error\n";	return -1; }

    pos = m_Position;
    return 0;
}
// -----------------------------------------


//DWORD WINAPI EposCAN::CAN_WriteThread(LPVOID pHandle)
//{
//	// CKim - Handle to CAN socket is passed as a parameter to thread function.
//	//int hd = *((int*)pHandle);

//	// CKim - Write RTR or SYNC message to regularly poll the state of the devices.

//	// CKim - Write RxPDO
//	return 1;
//}

//int SDO_acknowledge(int fd, const my_can_frame* f) {
//	Socketcan_t ack[4];
//	ack[0].size = 1;
//	ack[0].data = SDO_RESPONSE_WRITE_OK;
//
//	ack[1].size = 1;
//	ack[1].data = f->data[1]; // index lsb
//
//	ack[2].size = 1;
//	ack[2].data = f->data[2]; // index msb
//
//	ack[3].size = 1;
//	ack[3].data = f->data[3]; // subindex
//
//	int nodeid = f->id-SDO_RX;
//	return socketcan_write(fd, SDO_TX+nodeid, 4, ack);
//}


//int EposCAN::SetTxPDOMapping(int n)
//{
//    SDO_data pack;		pack.nodeid = m_nodeId;

//    // CKim - Changes to PDO mapping is only possible in Preoperational mode

//    // CKim - 1. Configure the Communication Object ID (COB-ID) of each TxPDO (1-4). Item 0x1800~3 in dictionary
//    // SubIndex 0x01 (uint_32) gives the CobId
//    int cobId;
//    pack.index = 0x1800 + (n - 1);		pack.subindex = 0x01;		pack.sz = 4;

//    if (n == 1) { pack.data = TX_PDO1 + m_nodeId; }
//    if (n == 2) { pack.data = TX_PDO2 + m_nodeId; }
//    if (n == 3) { pack.data = TX_PDO3 + m_nodeId; }
//    if (n == 4) { pack.data = TX_PDO4 + m_nodeId; }

//    if (!SDO_write(&pack))	{ cobId = pack.data; }
//    else						{ return -1; }

//    // CKim - 2. Set Transmission type.
//    // SubIndex 0x02 (uint_8)  : Transmit on  1. Sync, 253. asynch transmission by RTR, 255. asynch transmission by Change
//    // Sync : Write COB-ID 0x00000080 with no data. COB-ID for SYNC is in the index 0x1005 of dictionary.
//    // RTR : Remote Transmission Request. Request is made by setting the last bit of COB-ID in the signal to zero.
//    // Set the last argument of the CAN_Send to 1 to make RTR.
//    // For example, you can make a request to a device to transmit TPDO that contains data you need by sending
//    // an empty TPDO with the RTR flag. (if the device is configured to accept TPDO requests)
//    pack.subindex = 0x02;	pack.sz = 1;	pack.data = 1;	// 253;
//    if (!SDO_write(&pack))	{}
//    else						{ return -1; }

//    // SubIndex 03 (uint_16) : Inhibit time. Use only when SubIndex 02 is 255 (transmit on change)

//    // CKim - 3. Disable PDO by setting number of objects mapped (uint_8, Index 0x1A00~03 Subindex 0x00) to zero
//    pack.index = 0x1A00 + (n - 1);	 pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
//    if (!SDO_write(&pack))	{}
//    else						{ return -1; }

//    // CKim - 4. Info of the oject to be that will be transmitted by PDO.
//    // 4 Byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
//    //  nth object = 0x1A00~3 subindex 0x0n size (0x08, 0x10, 0x20)
//    pack.subindex = 0x01;	pack.sz = 4;
//    //pack.data = 0x60FB0210; // (0x60FB sub 02 : Position I gain, size is 0x10 16 bit . )
//    pack.data = 0x60410010; // (0x6041 sub 00 : Position I gain, size is 0x10 16 bit . )
//    if (!SDO_write(&pack))	{}
//    else						{ return -1; }

//    // CKim - 5. Enable PDO by setting number of objects mapped to nonzero
//    pack.subindex = 0x00;	pack.sz = 1;	pack.data = 1;
//    if (!SDO_write(&pack))	{}
//    else						{ return -1; }

//    return cobId;
//}

//int EposCAN::SetRxPDOMapping(int n)
//{
//	// With RPDOs you can, for example, start two devices simultaneously. You only need to map the same RPDO into
//	// two or more different devices and make sure those RPDOs are mapped with the same COB-ID.
//	SDO_data pack;		pack.nodeid = m_nodeId;

//	// CKim - Changes to PDO mapping is only possible in Preoperational mode

//	// CKim - 1. Configure the Communication Object ID (COB-ID) of each TxPDO (1-4). Item 0x1800~3 in dictionary
//	// SubIndex 0x01 (uint_32) gives the CobId
//	int cobId;
//	pack.index = 0x1400 + (n - 1);		pack.subindex = 0x01;		pack.sz = 4;

//	if (n == 1) { pack.data = RX_PDO1 + m_nodeId; }
//	if (n == 2) { pack.data = RX_PDO2 + m_nodeId; }
//	if (n == 3) { pack.data = RX_PDO3 + m_nodeId; }
//	if (n == 4) { pack.data = RX_PDO4 + m_nodeId; }

//	if (!SDO_write(&pack))	{ cobId = pack.data; }
//	else						{ return -1; }

//	// CKim - 2. Set Transmission type.
//	// SubIndex 0x02 (uint_8)  : Received data is applied to node 1. on Sync, 255. Immediately
//	// Sync : Write COB-ID 0x00000080 with no data. COB-ID for SYNC is in the index 0x1005 of dictionary.
//	pack.subindex = 0x02;	pack.sz = 1;	pack.data = 255;	// 253;
//	if (!SDO_write(&pack))	{}
//	else						{ return -1; }

//	// CKim - 3. Disable PDO by setting number of objects mapped (uint_8, Index 0x1600~03 Subindex 0x00) to zero
//	pack.index = 0x1600 + (n - 1);	 pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
//	if (!SDO_write(&pack))	{}
//	else						{ return -1; }

//	// CKim - 4. Info of the oject to be that will be transmitted by PDO.
//	// 4 Byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
//	//  nth object = 0x1A00~3 subindex 0x0n size (0x08, 0x10, 0x20)
//	pack.subindex = 0x01;	pack.sz = 4;
//	pack.data = 0x60400010; // (0x6040 sub 00 : ControlWord 16 bit . )
//	if (!SDO_write(&pack))	{}
//	else						{ return -1; }

//	// CKim - 5. Enable PDO by setting number of objects mapped to nonzero
//	pack.subindex = 0x00;	pack.sz = 1;	pack.data = 1;
//	if (!SDO_write(&pack))	{}
//	else						{ return -1; }

//	return cobId;
//}
