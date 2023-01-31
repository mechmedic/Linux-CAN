#include <stdio.h>
#include <string.h>
#include "EposCAN.h"

// CKim - Static Variable initialization
canInterface EposCAN::m_CANport;

EposCAN::EposCAN()
{
    for(int i=0; i<NUM_NODE; i++)
    {
        m_Slave[i].m_nodeId = i+1;

        // CKim - Allocate Buffer for RxPDO communication
        m_RxPDOsendBuff[i] = new char[8*4];     // 8 byte 4 PDOS
//        m_TxPDOreadBuff[i] = new char[8*4];     // 8 byte 4 PDOS
    }
    //m_nodeId = n;
}

EposCAN::~EposCAN()
{
    sendFlag = readFlag = false;
    pthrRead->join();
    pthrSend->join();

    for(int i=0; i<NUM_NODE; i++)  {
        delete m_RxPDOsendBuff[i];
        //delete m_TxPDOreadBuff[i];
    }
}

int EposCAN::EnableDevice(int idx)
{
    int nodeId = m_Slave[idx].m_nodeId;

    // CKim - When the device is turned on it is in "Switch On Disabled" state. Status word (0x6041) being x0xx xxx1 x100 0000 = 0x0540
    // The state must be transited to  "Switch On Disabled" -> "Ready to Switch On" -> "Switched On" -> "Enable"
    // by writing appropriate Control word (0x6040)
    SDO_data pack;

    // CKim - "Switch On Disabled" -> "Ready to Switch On" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x110. For example 0000 0110 = 0x06
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0006;
    if(!SDO_write(&pack)) {}
    else    {   return -1;   }

    // CKim -  "Ready to Switch On" -> "Switched On" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x111. For example 0000 0111 = 0x07
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0007;
    if(!SDO_write(&pack)) {}
    else    {   return -1;   }

    // CKim -  "Switched On" -> "Enable" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x111. For example 0000 0111 = 0x0F
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x000F;
    if(!SDO_write(&pack)) {}
    else    {   return -1;   }

    return 0;
}

int EposCAN::DisableDevice(int idx)
{
    int nodeId = m_Slave[idx].m_nodeId;

    // CKim - To turn off the device, state must be transited to
    // "Enable" -> "Switched On" -> "Ready to Switch On" -> "Switch On Disabled"
    // by writing appropriate Control word (0x6040)
    SDO_data pack;

    // CKim - "Enable" -> "Switched On" transition (torque is disabled) happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx 0111. For example 0000 0111 = 0x07
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0007;
    if(!SDO_write(&pack)) {}
    else    {   return -1;   }

    // CKim -  "Switched On" -> "Ready to Switch On" transition (power is disabled) happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx x110. For example 0000 0110 = 0x06
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0006;
    if(!SDO_write(&pack)) {}
    else    {   return -1;   }

    // CKim - "Ready to Switch On" -> "Switch On Disabled" transition happens when
    // Lower Bytes of Controlword (idx 0x6040) is set to 0xxx xx0x. For example 0000 0000 = 0x00
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;	pack.data = 0x0000;
    if(!SDO_write(&pack)) {}
    else    {   return -1;   }

    return 0;
}

int EposCAN::EnableDeviceAll()
{
    for(int i=0; i<NUM_NODE; i++){
        if(EnableDevice(i)) {   return -1;  }
    }
    return 0;
}

int EposCAN::DisableDeviceAll()
{
    for(int i=0; i<NUM_NODE; i++){
        if(DisableDevice(i)) {   return -1;  }
    }
    return 0;
}

int EposCAN::SetOperationMode(int idx, int mode)
{
    int nodeId = m_Slave[idx].m_nodeId;
    SDO_data pack;
    pack.nodeid = nodeId;	 pack.index = 0x6060;	pack.subindex = 0x00;	pack.sz = 1;

    if ((mode == HOMING) || (mode == PROFILE_POS) || (mode == PROFILE_VEL) || (mode == POSITION)
        || (mode == VELOCITY) || (mode == CURRENT) || (mode == SYNC_POS) || (mode == SYNC_VEL) || (mode == SYNC_TRQ))
    {
        pack.data = mode;
        if(!SDO_write(&pack)) {
            m_mode = mode;
            return 0;
        }
        else    {   return -1;   }
    }
    else {
        return -1;
    }
}

int EposCAN::SetOperationModeAll(int mode)
{
    for(int i=0; i<NUM_NODE; i++){
        if(SetOperationMode(i, mode)) {   return -1;  }
    }
    return 0;
}

// -----------------------------------------
uint16_t EposCAN::ReadControlWord(int idx)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

    pack.index = 0x6040;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ return pack.data; }
    else	{ return -1; }
}

uint16_t EposCAN::ReadStatusWord(int idx)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

    pack.index = 0x6041;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ return pack.data; }
    else	{ return -1; }
}

int EposCAN::ReadPosition(int idx)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

    pack.index = 0x6064;	pack.subindex = 0x00;
    if (!SDO_read(&pack))	{ return pack.data; }
    else	{ return -1; }
}

int EposCAN::HaltAxis(int idx)
{
    SDO_data pack;

    // CKim - Set bit 8 of the control word to stop motion
    pack.nodeid = m_Slave[idx].m_nodeId;
    pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;
    pack.data = 0x010F;

    if (!SDO_write(&pack))	{ return 0; }
    else					{ return -1; }
}

int EposCAN::HaltAxisAll()
{
    for(int i=0; i<NUM_NODE; i++){
        if(HaltAxis(i)) {   return -1;  }
    }
    return 0;
}

// -----------------------------------------
int EposCAN::GetPositionProfileParam(int idx, ProfilePosParam& P)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

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

int EposCAN::SetPositionProfileParam(int idx, const ProfilePosParam& P)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

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

int EposCAN::MovePosProfile(int idx, int32_t pos, bool rel)
{
    int nodeId = m_Slave[idx].m_nodeId;
    SDO_data pack;

    // CKim - EPOS4 only : Set control word to 0x000F to receive new setpoint
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;     pack.data = 0x000F;
    if (!SDO_write(&pack))	{ }
    else					{ return -1; }

    // CKim - Write new target position value
    pack.nodeid = nodeId;     pack.index = 0x607A;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = pos;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    // CKim - Set control word so that positioning starts immedeately
    // Control word for Position Profile mode
    // bit 4-7 :
    //   4. 1 New Set Point
    //   5. (0 finish positioning before change setpoint. 1. immedeately change setpoint)
    //   6. 0 absolute, 1 relative,
    //   7. Fault Reset (1 indicates fault),
    // bit 8-11 : 8. Halt
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;
    if (rel) 	{ pack.data = 0x007F; }
    else		{ pack.data = 0x003F; }

    if (!SDO_write(&pack))	{ return 0; }
    else					{ return -1; }
}

int EposCAN::WaitForMotionCompletion(int idx, long timeoutmsec)
{
    uint16_t motionStatus;

    long sec = timeoutmsec/1000;
    long nsec = (timeoutmsec%1000)*1e6;
    struct timespec start, now;
    double elapsedTime;

    clock_gettime(CLOCK_REALTIME,&start);
    start.tv_sec += sec;
    start.tv_nsec += nsec;

    while(1)
    {
        motionStatus= ReadStatusWord(idx);

        // CKim - In profile position mode, bit 12 and bit 10 of
        // the status word is set to 1 when motion is complete
        if((motionStatus & 0x1400) == 0x1400) {     break;      }

        clock_gettime(CLOCK_REALTIME,&now);
        elapsedTime = (now.tv_sec - start.tv_sec);
        elapsedTime = elapsedTime + (now.tv_nsec - start.tv_nsec)*1e-9;
        if(elapsedTime*1000 > timeoutmsec)
        {
            printf("Motion Waiting Timed Out %f\n",elapsedTime);
            return -1;
        }
    }
    printf("Motion Complete\n");
    return 0;

    //    err = sem_timedwait(&m_MotionSema,&ts);
    //    if(err == -1)   {
    //        //perror("Semaphore");
    //        printf("WaitForMotion time out\n");
    //        return -1;
    //    }

}

int EposCAN::WaitForMotionCompletionAll(long timeoutmsec)
{
    uint16_t motionStatus;
    int cnt = 0;

    long sec = timeoutmsec/1000;
    long nsec = (timeoutmsec%1000)*1e6;
    struct timespec start, now;
    double elapsedTime;

    clock_gettime(CLOCK_REALTIME,&start);
    start.tv_sec += sec;
    start.tv_nsec += nsec;

    while(1)
    {
        for(int i=0; i<NUM_NODE; i++)
        {
            motionStatus= ReadStatusWord(i);

            // CKim - In profile position mode, bit 12 and bit 10 of
            // the status word is set to 1 when motion is complete
            if((motionStatus & 0x1400) == 0x1400) {     cnt++;  }
            if(cnt == NUM_NODE) {   break;      }

            clock_gettime(CLOCK_REALTIME,&now);
            elapsedTime = (now.tv_sec - start.tv_sec);
            elapsedTime = elapsedTime + (now.tv_nsec - start.tv_nsec)*1e-9;
            if(elapsedTime*1000 > timeoutmsec)
            {
                printf("Motion Waiting Timed Out %f\n",elapsedTime);
                return -1;
            }
        }
    }
    printf("Motion Complete\n");
    return 0;
}

// -----------------------------------------
int EposCAN::GetVelocityProfileParam(int idx, ProfileVelParam& P)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

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

int EposCAN::SetVelocityProfileParam(int idx, const ProfileVelParam& P)
{
    SDO_data pack;
    pack.nodeid = m_Slave[idx].m_nodeId;

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

int EposCAN::MoveVelProfile(int idx, int32_t vel)
{
    int nodeId = m_Slave[idx].m_nodeId;
    SDO_data pack;

    // CKim - Write new target velocity value. Object Index is 0x60FF
    pack.nodeid = nodeId;     pack.index = 0x60FF;	pack.subindex = 0x00;	pack.sz = 4;	pack.data = vel;
    if (!SDO_write(&pack))	{}
    else	{ return -1; }

    // CKim - Set control word to 0x000F to start moving
    pack.nodeid = nodeId;	 pack.index = 0x6040;	pack.subindex = 0x00;	pack.sz = 2;
    pack.data = 0x000F;

    if (!SDO_write(&pack))	{ return 0; }
    else					{ return -1; }
}

// -----------------------------------------
int EposCAN::EnablePDO(int idx)
{
    return m_CANport.SendNMT(NMT_OPERATIONAL, m_Slave[idx].m_nodeId);
}

int EposCAN::DisablePDO(int idx)
{
    return m_CANport.SendNMT(NMT_PREOPERATIONAL, m_Slave[idx].m_nodeId);
}

int EposCAN::WriteTxPdoSettings(int idx)
{
    SDO_data pack;		pack.nodeid = m_Slave[idx].m_nodeId;

    // CKim - Write TxPDO parameters
    for (int n = 0; n < 4; n++)
	{
        // CKim - 1. Configure TxPDO settings. Item 0x1800~03
        pack.index = OD_TPDO1_PARAM + n;    // 0x1800 + n;

        // SubIndex 0x01 (uint_32) : Communication Object ID (COB-ID)
        pack.subindex = 0x01;		pack.sz = 4;		pack.data = m_Slave[idx].TpdoParam[n].COB_ID;
        if (!SDO_write(&pack))	{ }
        else					{ return -1; }

        // SubIndex 0x02 (uint_8) : Transmission type.
        pack.subindex = 0x02;       pack.sz = 1;	pack.data = m_Slave[idx].TpdoParam[n].TransmissionType;   //pack.data = 255;
        if (!SDO_write(&pack))	{}
        else					{ return -1; }

        // SubIndex 03 (uint_16) : Inhibit time.
        pack.subindex = 0x03;	pack.sz = 2;	pack.data = m_Slave[idx].TpdoParam[n].InhibitTime;//10;//1000;	// multiples of 100us = 100 ms.
        if (!SDO_write(&pack))	{}
        else					{ return -1; }
    }

    // CKim - Write TxPDO mappings
    for (int n = 0; n < 4; n++)
    {
        // CKim - 2. Configure TxPDO Map. Item 0x1A00~03
        pack.index = OD_TPDO1_MAP + n;

        // Set subindex 0x00, number of data to zero to disable PDO before writing subindex 0x01
       pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
        if (!SDO_write(&pack))	{}
        else					{ return -1; }

        // subindex 0x01 : objidx + subindex + size of 1st data
        for(int i=0; i<m_Slave[idx].TpdoMap[n].NumberOfMappedObject; i++)
        {
            // Write 4 byte info of the nth object (Sub 0x0n) that will be transmitted by PDO.
            // 4 byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
            // Size in bits (0x08, 0x10, 0x20)
            pack.subindex = 0x01 + i;       pack.sz = 4;
            int32_t objidx = m_Slave[idx].TpdoMap[n].ObjIdx[i];
            int32_t subidx = m_Slave[idx].TpdoMap[n].ObjSubIdx[i];
            int32_t sz = m_Slave[idx].TpdoMap[n].ObjSz[i];
            pack.data = (objidx << 16) + (subidx << 8) + sz;
            printf("TxPDO%d Index 0x%04X Data 0x%08X\n",n+1,pack.index,pack.data);
            //pack.data = m_Slave[nodeId].TpdoMap[n]0x60410010; // (0x6041 sub 00 : Status word, size is 0x10 16 bit . )
            if (!SDO_write(&pack))	{}
            else					{ return -1; }
        }

        // Enable PDO by setting number of objects mapped to nonzero
        pack.subindex = 0x00;	pack.sz = 1;	pack.data = m_Slave[idx].TpdoMap[n].NumberOfMappedObject;  // CKim changed to 0 from 1
        if (!SDO_write(&pack))	{}
        else					{ return -1; }
    }
	return 0;//	return cobId;
}

int EposCAN::WriteRxPdoSettings(int idx)
{
    SDO_data pack;		pack.nodeid = m_Slave[idx].m_nodeId;

    // CKim - Write RxPDO parameters
    for (int n = 0; n < 4; n++)
    {
        // CKim - 1. Configure RxPDO settings. Item 0x1400~03
        pack.index = OD_RPDO1_PARAM + n;    //0x1400 + n

        // SubIndex 0x01 (uint_32) : Communication Object ID (COB-ID)
        pack.subindex = 0x01;		pack.sz = 4;		pack.data = m_Slave[idx].RpdoParam[n].COB_ID;
        if (!SDO_write(&pack))	{ }
        else					{ return -1; }

        // SubIndex 0x02 (uint_8) : Transmission type.
        pack.subindex = 0x02;       pack.sz = 1;	pack.data = m_Slave[idx].RpdoParam[n].TransmissionType;   //pack.data = 255;
        if (!SDO_write(&pack))	{}
        else					{ return -1; }
    }

    // CKim - Write RxPDO mappings
    for (int n = 0; n < 4; n++)
    {
        // CKim - 2. Configure TxPDO Map. Item 0x1600~03
        pack.index = OD_RPDO1_MAP + n;  // 0x1600 + n

        // Set subindex 0x00, number of data to zero to disable PDO before writing subindex 0x01
        pack.subindex = 0x00;	pack.sz = 1;	pack.data = 0;
        if (!SDO_write(&pack))	{}
        else					{ return -1; }

        // subindex 0x01 : objidx + subindex + size of 1st data
        for(int i=0; i<m_Slave[idx].RpdoMap[n].NumberOfMappedObject; i++)
        {
            // Write 4 byte info of the nth object (Sub 0x0n) that will be transmitted by PDO.
            // 4 byte = Object Index (2byte) + SubIndex (1byte) + Size (1byte) = (uint_32)
            // Size in bits (0x08, 0x10, 0x20)
            pack.subindex = 0x01 + i;       pack.sz = 4;
            int32_t objidx = m_Slave[idx].RpdoMap[n].ObjIdx[i];
            int32_t subidx = m_Slave[idx].RpdoMap[n].ObjSubIdx[i];
            int32_t sz = m_Slave[idx].RpdoMap[n].ObjSz[i];
            pack.data = (objidx << 16) + (subidx << 8) + sz;
            printf("RxPDO%d Index 0x%04X Data 0x%08X\n",n+1,pack.index,pack.data);
            //pack.data = m_Slave[idx].TpdoMap[n]0x60410010; // (0x6041 sub 00 : Status word, size is 0x10 16 bit . )
            if (!SDO_write(&pack))	{}
            else					{ return -1; }
        }

        // Enable PDO by setting number of objects mapped to nonzero
        pack.subindex = 0x00;	pack.sz = 1;	pack.data = m_Slave[idx].RpdoMap[n].NumberOfMappedObject;  // CKim changed to 0 from 1
        if (!SDO_write(&pack))	{}
        else					{ return -1; }
    }
    return 0;//	return cobId;
}

int EposCAN::ConfigureTxPDOs(int idx)
{
    int nodeId = m_Slave[idx].m_nodeId;
    // CKim - Edit the code here to change TxPDO configurations and mappings.
    // Call this function and then WriteTxPdoSettings() to change PDO mappings
    // Changes to PDO mapping is only possible in Preoperational mode

    // CKim - TxPDO1. 16 bit 'Status word' (0x6041 Sub 00), Transmit on change (255) within 1 ms.
    m_Slave[idx].TpdoParam[0].COB_ID = COBID_TXPDO1 + nodeId;
    m_Slave[idx].TpdoParam[0].TransmissionType = TXPDO_TRANSMIT_ON_SYNC;;//TXPDO_TRANSMIT_ON_CHANGE;
    m_Slave[idx].TpdoParam[0].InhibitTime = 10;  // multiples of 100us = 1 ms..TransmissionType = 255;

    m_Slave[idx].TpdoMap[0].NumberOfMappedObject = 1;
    m_Slave[idx].TpdoMap[0].ObjIdx[0] = OD_STATUS_WORD;
    m_Slave[idx].TpdoMap[0].ObjSubIdx[0] = 0;
    m_Slave[idx].TpdoMap[0].ObjSz[0] = 0x10;    // 16 bits

    // CKim - TxPDO2. 32 bit 'Position Actual Value' (0x6064 Sub 00) and
    // 32 bit 'Velocity Actual Value' (0x606C Sub 00) Transmit on SYNC
    m_Slave[idx].TpdoParam[1].COB_ID = COBID_TXPDO2 + nodeId;
    m_Slave[idx].TpdoParam[1].TransmissionType = TXPDO_TRANSMIT_ON_SYNC;
    m_Slave[idx].TpdoParam[1].InhibitTime = 10;  // multiples of 100us = 1 ms..TransmissionType = 255;

    m_Slave[idx].TpdoMap[1].NumberOfMappedObject = 2;
    m_Slave[idx].TpdoMap[1].ObjIdx[0] = OD_POSITION_ACTUAL_VAL;
    m_Slave[idx].TpdoMap[1].ObjSubIdx[0] = 0;
    m_Slave[idx].TpdoMap[1].ObjSz[0] = 0x20;    // 32 bits

    m_Slave[idx].TpdoMap[1].ObjIdx[1] = OD_VELOCITY_ACTUAL_VAL;
    m_Slave[idx].TpdoMap[1].ObjSubIdx[1] = 0;
    m_Slave[idx].TpdoMap[1].ObjSz[1] = 0x20;    // 32 bits

    // CKim - Nothing for TxPDO3 and 4. Set # of mapped object to 0
    m_Slave[idx].TpdoParam[2] = m_Slave[idx].TpdoParam[0];
    m_Slave[idx].TpdoMap[2] = m_Slave[idx].TpdoMap[0];
    m_Slave[idx].TpdoParam[2].COB_ID = COBID_TXPDO3 + nodeId;
    m_Slave[idx].TpdoMap[2].NumberOfMappedObject = 0;

    m_Slave[idx].TpdoParam[3] = m_Slave[idx].TpdoParam[0];
    m_Slave[idx].TpdoMap[3] = m_Slave[idx].TpdoMap[0];
    m_Slave[idx].TpdoParam[3].COB_ID = COBID_TXPDO4 + nodeId;
    m_Slave[idx].TpdoMap[3].NumberOfMappedObject = 0;

    // CKim - Update Number of Total TxPDO
    m_totalTxPdoNum = 2;

    return 0;
}

int EposCAN::ConfigureRxPDOs(int idx)
{
    int nodeId = m_Slave[idx].m_nodeId;
    // CKim - Edit the code here to change RxPDO configurations and mappings.
    // Call this function and then WriteRxPdoSettings() to change PDO mappings
    // Changes to PDO mapping is only possible in Preoperational mode

    // CKim - RxPDO1 : 16 bit 'ControlWord' (0x6040 Sub 00), Apply Immediately
    m_Slave[idx].RpdoParam[0].COB_ID = COBID_RXPDO1 + nodeId;
    m_Slave[idx].RpdoParam[0].TransmissionType = RXPDO_UPDATE_IMMED;//RXPDO_UPDATE_ON_SYNC;

    m_Slave[idx].RpdoMap[0].NumberOfMappedObject = 1;
    m_Slave[idx].RpdoMap[0].ObjIdx[0] = OD_CONTROL_WORD;
    m_Slave[idx].RpdoMap[0].ObjSubIdx[0] = 0;
    m_Slave[idx].RpdoMap[0].ObjSz[0] = 0x10;    // 16 bits


    // CKim - RxPDO2 : 32 bit 'Target Position' (0x607A Sub 00), Update on SYNC
    m_Slave[idx].RpdoParam[1].COB_ID = COBID_RXPDO2 + nodeId;
    m_Slave[idx].RpdoParam[1].TransmissionType = RXPDO_UPDATE_ON_SYNC;

    m_Slave[idx].RpdoMap[1].NumberOfMappedObject = 1;
    m_Slave[idx].RpdoMap[1].ObjIdx[0] = OD_TARGET_POSITION;
    m_Slave[idx].RpdoMap[1].ObjSubIdx[0] = 0;
    m_Slave[idx].RpdoMap[1].ObjSz[0] = 0x20;    // 32 bits

    // CKim - Nothing for RxPDO3 and 4. Set # of mapped object to 0
    m_Slave[idx].RpdoParam[2] = m_Slave[idx].RpdoParam[0];
    m_Slave[idx].RpdoMap[2] = m_Slave[idx].RpdoMap[0];
    m_Slave[idx].RpdoParam[2].COB_ID = COBID_RXPDO3 + nodeId;
    m_Slave[idx].RpdoMap[2].NumberOfMappedObject = 0;

    m_Slave[idx].RpdoParam[3] = m_Slave[idx].RpdoParam[0];
    m_Slave[idx].RpdoMap[3] = m_Slave[idx].RpdoMap[0];
    m_Slave[idx].RpdoParam[3].COB_ID = COBID_RXPDO4 + nodeId;
    m_Slave[idx].RpdoMap[3].NumberOfMappedObject = 0;

    return 0;
}

void* EposCAN::RxPDOSendThread(void* pData)
{
    EposCAN* self = (EposCAN*) pData;
    //int cnt = 0;
    uint16_t cobId;  char* buff;     int nbytes;    int err;   int nodeId;

    while (self->sendFlag)
    {
        //printf("Send Thread count %d\n",cnt++);
        for(int idx=0; idx<NUM_NODE; idx++)
        {
            nodeId = self->m_Slave[idx].m_nodeId;

            // ------------------------------------------ //
            // CKim - Update this part when PDO map changes
            // Copy data to m_RxPDOsendBuff
            // RxPDO1 : control word (2 byte) to buffer 8*0
            memcpy(self->m_RxPDOsendBuff[idx],&(self->m_Slave[idx]).data_.control_word,2);
            // RxPDO2 : target position (4 byte) to 8*1 + 0 bytes;
            memcpy(self->m_RxPDOsendBuff[idx]+8,&(self->m_Slave[idx]).data_.target_pos,4);
            // ------------------------------------------ //

            // CKim - Send RxPDO1 to 4
            for(int n=0; n<4; n++)
            {
                cobId = COBID_RXPDO1 + (0x00000100*n) + nodeId;
                buff = self->m_RxPDOsendBuff[idx]+(8*n);
                for(int i=0; i<self->m_Slave[idx].RpdoMap[n].NumberOfMappedObject; i++)
                {
                    nbytes = (self->m_Slave[idx].RpdoMap[n].ObjSz[i])/8;
                    //printf("Writing item %d of RxPDO%d of Node %d\n",i+1,n+1,nodeId);
                    err = m_CANport.SendRxPDO(cobId, nbytes, buff);
                    if(err != 0)
                    {
                        printf("Error Writing item %d of RxPDO%d of Node %d in RxPDOSendThread\n",i+1,n+1,nodeId);
                        break;
                    }
                    buff = self->m_RxPDOsendBuff[idx] + self->m_Slave[idx].RpdoMap[n].ObjSz[i];
                }
            }
        }

        // CKim - Call SYNC to tell slaves to update themselves using received RxPDO value
        // Also tells slaves to report current status through TxPDO.
//        self->m_CANport.SendSYNC();
//        usleep(2000);
    }
    printf("Leaving RxPDOSend Thread\n");
    return 0;
}

void* EposCAN::TxPDOReadThread(void* pData)
{
    EposCAN* self = (EposCAN*) pData;
    int nodeId, pdoid;      int err;    char buff[8];
    int cnt = 0;  int cnt1 = 0;   int cnt2 = 0;
    while (self->readFlag)
    {
//        printf("Read Thread count %d\n",cnt++);

        // CKim - Call SYNC to tell slaves to report current status through TxPDO.
        if(cnt1 == cnt2)    {
//            printf("Synchronizing\n",cnt++);
            self->m_CANport.SendSYNC();
        }

        // CKim - Read TxPDO.
        err = m_CANport.ReadTxPDO(nodeId,pdoid,buff);
        if(err != 0)    {   printf("TxPDOReadThread Error\n");  break;   }

        // ------------------------------------------ //
        // CKim - Map memory. Update this part when PDO map changes
        // 1. Find index corresponding to the nodeId
        int idx = nodeId-1;
        // TxPDO1 : status word to buffer 8*0
        if(pdoid == COBID_TXPDO1)   {
            memcpy(&(self->m_Slave[idx]).data_.status_word,buff,2);
            cnt1++;
        }
        // TxPDO2 : actual pos/vel to 8*1 + 0/4 bytes;
        if(pdoid == COBID_TXPDO2)   {
            memcpy(&(self->m_Slave[idx]).data_.actual_pos, buff,4);
            memcpy(&(self->m_Slave[idx]).data_.actual_vel, buff+4,4);
            cnt2++;
        }

        // ------------------------------------------ //
        //usleep(5000);
    }
    printf("Leaving TxPDOReadThread\n");
    printf("TxPDO1 %d TxPDO2 %d\n",cnt1,cnt2);
    return 0;
}

void* EposCAN::CyclicPDOThread(void* pData)
{
    EposCAN* self = (EposCAN*) pData;
    uint16_t cobId;  char* sendBuff;     char readBuff[8];
    int nbytes;     int err;   int nodeId;   int pdoid;     int pdocnt;

    while (self->sendFlag)
    {
        // ------------------------------------------ //
        // CKim - Update Data to send over RxPDO
        // Edit this part when PDO map changes
        // Copy data to m_RxPDOsendBuff
        for(int idx=0; idx<NUM_NODE; idx++)
        {
            // RxPDO1 : control word (2 byte) to buffer 8*0
            memcpy(self->m_RxPDOsendBuff[idx],&(self->m_Slave[idx]).data_.control_word,2);
            // RxPDO2 : target position (4 byte) to 8*1 + 0 bytes;
            memcpy(self->m_RxPDOsendBuff[idx]+8,&(self->m_Slave[idx]).data_.target_pos,4);
        }
        // ------------------------------------------ //

        // CKim - For each device, send RxPDO1 to 4
        for(int idx=0; idx<NUM_NODE; idx++)
        {
            nodeId = self->m_Slave[idx].m_nodeId;
            for(int n=0; n<4; n++)
            {
                cobId = COBID_RXPDO1 + (0x00000100*n) + nodeId;
                sendBuff = self->m_RxPDOsendBuff[idx]+(8*n);
                for(int i=0; i<self->m_Slave[idx].RpdoMap[n].NumberOfMappedObject; i++)
                {
                    nbytes = (self->m_Slave[idx].RpdoMap[n].ObjSz[i])/8;
                    //printf("Writing item %d of RxPDO%d of Node %d\n",i+1,n+1,nodeId);
                    err = m_CANport.SendRxPDO(cobId, nbytes, sendBuff);
                    if(err != 0)
                    {
                        printf("Error Writing item %d of RxPDO%d of Node %d in RxPDOSendThread\n",i+1,n+1,nodeId);
                        break;
                    }
                    sendBuff = self->m_RxPDOsendBuff[idx] + self->m_Slave[idx].RpdoMap[n].ObjSz[i];
                }
            }
        }

        // CKim - Call SYNC to tell slaves to update themselves using received RxPDO value
        // Also tell slaves to report current status through TxPDO.
        self->m_CANport.SendSYNC();

        // CKim - All slaves will send TxPDO once per SYNC signale.
        // Make sure all PDOs are read before returning to RxPDO send
        pdocnt = 0;
        while(pdocnt != self->m_totalTxPdoNum)
        {
            // CKim - Read TxPDO.
            err = m_CANport.ReadTxPDO(nodeId,pdoid,readBuff);
            if(err != 0)    {   printf("TxPDOReadThread Error\n");  break;   }

            // ------------------------------------------ //
            // CKim - Map memory. Update this part when PDO map changes
            // 1. Find index corresponding to the nodeId
            int idx = nodeId-1;
            // TxPDO1 : status word to buffer 8*0
            if(pdoid == COBID_TXPDO1)   {
                memcpy(&(self->m_Slave[idx]).data_.status_word,readBuff,2);
                pdocnt++;
            }
            // TxPDO2 : actual pos/vel to 8*1 + 0/4 bytes;
            if(pdoid == COBID_TXPDO2)   {
                memcpy(&(self->m_Slave[idx]).data_.actual_pos, readBuff,4);
                memcpy(&(self->m_Slave[idx]).data_.actual_vel, readBuff+4,4);
                pdocnt++;
            }
            // ------------------------------------------ //
        }
        usleep(1000);
    }
    printf("Leaving CyclicPDO Thread\n");
    return 0;
}

void EposCAN::StartPdoExchange()
{
    printf("Disabling PDO\n");
    for(int i=0; i<NUM_NODE; i++) {
        DisablePDO(i);
    }

    printf("Preparing PDO Exchange\n");
    for(int i=0; i<NUM_NODE; i++) {
        printf("Configuring Tx PDO\n");
        ConfigureTxPDOs(i);
        printf("Writing Tx PDO\n");
        WriteTxPdoSettings(i);
        printf("Configuring Rx PDO\n");
        ConfigureRxPDOs(i);
        printf("Writing Rx PDO\n");
        WriteRxPdoSettings(i);
    }

    printf("Enabling PDO\n");
    for(int i=0; i<NUM_NODE; i++) {
        EnablePDO(i);
    }

    printf("Starting thread\n");
    sendFlag = true;
    readFlag = true;
    //pthrRead = std::make_shared<std::thread>(EposCAN::TxPDOReadThread,this);
    //pthrSend = std::make_shared<std::thread>(EposCAN::RxPDOSendThread,this);
    pthrRead = std::make_shared<std::thread>(EposCAN::CyclicPDOThread,this);
}

void EposCAN::StopPdoExchange()
{
    printf("Stopping thread\n");
    sendFlag = false;
    readFlag = false;
    pthrRead->join();
    //pthrSend->join();

    printf("Disabling PDO\n");
    for(int i=0; i<NUM_NODE; i++) {
        DisablePDO(i);
    }


    // CKim - Clear buffer

    //pthrSend->join();
    // 0x0740 = 0000 0111 0100 0000
    // 0x1737 = 0001 0111 0011 0111
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


// -----------------------------------------

//int EposCAN::NotifyPDO(int n, const char* buff)
//{
//    memcpy(m_TxPdoData+(8*n),buff,8);

//    // ------------------------------------------------------
//    // CKim - TxPDO1 is configured to be 16 bit status word
//    if(n==0)
//    {
//        uint16_t status;	char* tmp = (char*)&status;
//        for (int i = 0; i < 2; i++)	{ *(tmp + i) = buff[i]; }
//        m_statusWord = status;

//        // CKim - Notify motion complete. In profile position mode, bit [0-15] of the statusword
//        // bit 12 and bit 10 is both 1;
//        if((status & 0x1400) == 0x1400)
//        {
//            sem_post(&m_MotionSema);
//        }
//    }
//    // ------------------------------------------------------

////    // ------------------------------------------------------
////    // CKim - TxPDO2 is configured to be 32 bit Position actual value
////    uint32_t pos;       char* tmp = (char*)&pos;
////    for (int i = 0; i < 4; i++)	{ *(tmp + i) = buf[i]; }

////    // CKim - Copy to member variable - may need mutex here
////    if (m_pDev[nodeId - 1])
////    {
////        m_pDev[nodeId - 1]->m_Position = pos;
////        SetEvent(m_hPDO_Event);
////    }
////    // ------------------------------------------------------
//}


//int EposCAN::MovePosProfileUsingPDO(int32_t pos, bool rel)
//{
//    uint16_t cobId, cword;  char* tmp;  char sendBuff[4];   int err;

////    // CKim - Write new Control Word : RxPDO1 0x6040  Sub 00 16 bit. Control Word
////    cobId = RX_PDO1 + m_nodeId;
////    tmp = (char*)&cword;
////    cword = 0x000F;
////    for (unsigned int i = 0; i<2; i++) {
////        sendBuff[i] = *(tmp + i);    }

////    err = m_CANport.SendRxPDO(cobId, 2, sendBuff);
////    if (err == -1)	{ return -1; }

//    // CKim - Write new target position value to RxPDO2 0x607A  Sub 00 32 bit. Target Position
//    cobId = RX_PDO2 + m_nodeId;
//    tmp = (char*)&pos;
//    for (unsigned int i = 0; i<4; i++)    {
//        sendBuff[i] = *(tmp + i);    }

//    err = m_CANport.SendRxPDO(cobId, 4, sendBuff);
//    if (err == -1)	{ return -1; }

//    // CKim - Write new Control Word : RxPDO1 0x6040  Sub 00 16 bit. Control Word
//    cobId = RX_PDO1 + m_nodeId;
//    tmp = (char*)&cword;
//    if (rel) 	{ cword = 0x007F; }
//    else		{ cword = 0x003F; }
//    for (unsigned int i = 0; i<2; i++) {
//        sendBuff[i] = *(tmp + i);    }

//    err = m_CANport.SendRxPDO(cobId, 2, sendBuff);
//    if (err == -1)	{ return -1; }

//    return 0;
// }

//int EposCAN::GetPositionUsingPDO(int32_t& pos)
//{
//    // CKim - Send RTR. Request device to send its position, configured in TxPDO2
//    // RTR is made by sending COB-ID of TxPDO2 with empty data with RTR bit set to 1
//    // The bit is set by CAN_Send() with last argument rtr set to 1.
//    uint16_t cobID = TX_PDO2 + m_nodeId;    char myBuff[8];
////    int err = CAN_Send(m_hd, cobID, 0, myBuff, 0, 1);

////    if (err == 0)	{ m_errmsg = "Send error";  return -1;  }

//    // CKim - Wait for PDO receive event
////    DWORD res = WaitForSingleObject(m_hPDO_Event, 1000);
////    if (res != WAIT_OBJECT_0)	{ m_errmsg = "Event Error\n";	return -1; }

//    pos = m_Position;
//    return 0;
//}
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




//uint16_t cword;     int32_t tgtPos;

//// CKim - In my current configuration. Only RxPDO 1 and 2 needs to be sent.

//// CKim - Send RxPDO1. 16 bit 'ControlWord' (0x6040 Sub 00)
//cobId = COBID_RXPDO1 + nodeId;
//buff = m_RxPDOsendBuff[nodeId];
//memcpy(buff,&cword,m_Slave[nodeId].RpdoMap[0].2);
//err = m_CANport.SendRxPDO(cobId, 2, buff);

//// CKim - Send RxPDO1. 32 bit 'Target Position' (0x607A Sub 00)
//cobId = COBID_RXPDO2 + nodeId;
//buff = m_RxPDOsendBuff[nodeId]+8;
//memcpy(buff,&tgtPos,4);
//err = m_CANport.SendRxPDO(cobId, 4, buff);
////    tmp = (char*)&pos;
////    for (unsigned int i = 0; i<4; i++)    {
////        sendBuff[i] = *(tmp + i);    }

////    err = m_CANport.SendRxPDO(cobId, 4, sendBuff);
////    if (err == -1)	{ return -1; }

////    // CKim - Write new Control Word : RxPDO1 0x6040  Sub 00 16 bit. Control Word
////    cobId = RX_PDO1 + m_nodeId;
////    tmp = (char*)&cword;
////    if (rel) 	{ cword = 0x007F; }
////    else		{ cword = 0x003F; }
////    for (unsigned int i = 0; i<2; i++) {
////        sendBuff[i] = *(tmp + i);    }

////    err = m_CANport.SendRxPDO(cobId, 2, sendBuff);
////    if (err == -1)	{ return -1; }

////    return 0;


//}

