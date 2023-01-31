#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     // Posix Header
#include <string.h>
#include <iostream>
#include <sstream>
#include <errno.h>

#include "linuxsocketcan.h"

// CKim - Initialize static member variables
int LinuxSocketCAN::m_hd = -1;                // CKim - Socket Handle. For pi2can

LinuxSocketCAN::LinuxSocketCAN()
{

    // Memo :sudo ifconfig can0 txqueuelen 1000
}

LinuxSocketCAN::~LinuxSocketCAN()
{

}

int LinuxSocketCAN::OpenCANport(const char* portName)
{
    std::ostringstream ostr;

    // CKim - Return if already open
    if(m_hd != -1) {
        ostr << "[SocketCAN] There is already an open CAN Port " << m_portName << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }

    // CKim - Open Socket for CAN communication
    m_hd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_hd < 0) {
        ostr << "[SocketCAN] error during 'socket()'" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }

    // CKim - Configure sockaddr (Socket Address) structure for CAN sockets
    // 'ifreq' is a Interface Request struct defined in if.h of linux kernel
    // ioctl manipulates io handles...
    struct sockaddr_can addr;       addr.can_family = AF_CAN;
    struct ifreq ifr;               strcpy(ifr.ifr_name, portName);
    if (ioctl(m_hd, SIOCGIFINDEX, &ifr) < 0) {
        ostr << "[SocketCAN] error during 'SIOCGIFINDEX'" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }
    addr.can_ifindex = ifr.ifr_ifindex;

    // CKim - Bind address to the socket.
    if (bind(m_hd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ostr << "[SocketCAN] error during 'bind()'" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }
    m_portName = portName;
    return 0;
}

int LinuxSocketCAN::CloseCANport()
{
    return(close(m_hd));
}

int LinuxSocketCAN::SendSYNC()
{
    // CKim - Communication Object ID (COB-ID) of SYNC object
    uint16_t cobIDforSYNC = COBID_SYNC;
    struct can_frame frame;
    int nbytes;

    frame.can_id = cobIDforSYNC;
    frame.can_dlc = 0;

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("[SocketCAN] error during 'write()' in SendSYNC ");
        return -1;
    }

    //std::cout << "[SocketCAN] SentSYNC" << std::endl;
    return 0;
}

int LinuxSocketCAN::SendNMT(int state, int nodeId)
{
    // CKim - Switch slave to 'Operational' state.
    // COB-ID = 0, data[0] = 0x01, data[1] = nodeId, (0 for all)
    // NMT operational state is indicated by bit 9 of the status word.
    struct can_frame frame;         int nbytes;

    frame.can_id = COBID_NMT;
    frame.can_dlc = 2;
    frame.data[0] = state;
    frame.data[1] = nodeId;

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("[SocketCAN] error during 'write()' in SendNMT ");
        return -1;
    }

    //std::cout << "[SocketCAN] Sent NMT message" << std::endl;
    return 0;
}

int LinuxSocketCAN::SendSDO(SDO_data* sdo, int rw)
{
    std::ostringstream ostr;

    if( rw!=SDO_READ && rw!=SDO_WRITE ) {   return -1;  }

    // ------------------------------------------------------------
    // CKim - Build buffer for sending data. Linux already has struct can_frame for CAN data frames
    // so I copy my SDO_data struct to can_frame struct
    struct can_frame sendFrame;         int nbytes;

    // CKim - Byte 0-3. id, command specifier and index
    if(rw == SDO_WRITE)
    {
        sendFrame.can_dlc = 4 + sdo->sz;
        sendFrame.data[0] = SDO_calculate_ccs('w', sdo->sz);      // Byte 0 : Command Specifier

        // CKim - Byte 4-7. Data
        for(int i=0; i<4; i++)
        {
            char* tmp = (char*)&(sdo->data);//memcpy()
            if (i<sdo->sz)	{ sendFrame.data[4 + i] = *(tmp + i); }
            else			{ sendFrame.data[4 + i] = 0; }
        }
    }
    else if (rw == SDO_READ)
    {
        sendFrame.can_dlc = 4;
        sendFrame.data[0] = 0x40;                       // Byte 0 : Command Specifier

        // CKim - Byte 4-7. Data
        for(int i=0; i<4; i++)  {   sendFrame.data[4 + i] = 0;  }
    }
    else {
        ostr << "[SocketCAN] Neither SDO_Write nor SDO_read" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }

    sendFrame.can_id = COBID_SDO_RX + sdo->nodeid;
    sendFrame.data[1] = (0xFF & (sdo->index));				// Byte 1 : Low byte of the index
    sendFrame.data[2] = (sdo->index) >> 8;					// Byte 2 : High byte of the index
    sendFrame.data[3] = sdo->subindex;						// Byte 3 : Subindex

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &sendFrame, sizeof(sendFrame));
    if (nbytes != sizeof(sendFrame)) {
        ostr << "[SocketCAN] CAN frame write error in SendSDO" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }

    //printf("[SocketCAN] SentSDO Packet. NodeID : 0x%04X  ObjIdx : 0x%04X\n",sendFrame.can_id, sdo->index);
    // ------------------------------------------------------------


    // ---------------------------------------------------------- //
    // CKim - Read CAN frame. SocketCAN uses 'read' functions. Blocks until read
    struct can_frame readFrame;         SDO_data rcvSdo;
    nbytes = read(m_hd,&readFrame,sizeof(struct can_frame));
    if (nbytes < 0) {
        ostr << "[SocketCAN] can raw socket read in SendSDO" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame)) {
        ostr << "[SocketCAN] read: incomplete CAN frame in SendSDO" << std::endl;
        m_errMsg = ostr.str();
        std::cerr << m_errMsg;
        return -1;
    }

    // ---------------------------------------------------------- //
    // CKim - Parse frame and check error code
    FrameToSdo(readFrame, &rcvSdo);

    char str[128];
    if (rcvSdo.errcode != 0)
    {
        sprintf(str, "[SocketCAN] Error in received frame : Device 0x%04X index 0x%04X, 0x%02X. Error code 0x%08X\n",
            rcvSdo.nodeid, rcvSdo.index, rcvSdo.subindex, rcvSdo.errcode);
        m_errMsg = str;
        std::cerr << m_errMsg;
        return -1;
    }
    else if ((sdo->nodeid != rcvSdo.nodeid) || (sdo->index != rcvSdo.index) || (sdo->subindex != rcvSdo.subindex))
    {
        m_errMsg = "[SocketCAN] Wrong SDO reply index!\n";
        std::cerr << m_errMsg;
        return -1;
    }
    else
    {
         // CKim - On successful SDO receive
        if(rw == SDO_WRITE)
        {
            //printf("[SocketCAN] Write OK : Device 0x%04X index 0x%04X, 0x%02X. Data 0x%04X\n",
                //sdo->nodeid, sdo->index, sdo->subindex, sdo->data);
        }
        if(rw == SDO_READ)
        {
            memcpy(sdo,&rcvSdo,sizeof(SDO_data));
            //printf("[SocketCAN] Read OK : Device 0x%04X index 0x%04X, 0x%02X. %d byte data is 0x%08X\n",
                //sdo->nodeid, sdo->index, sdo->subindex, sdo->sz, sdo->data);
        }
        return 0;
    }
    // ---------------------------------------------------------- //
}

int LinuxSocketCAN::FrameToSdo(const struct can_frame& frame, SDO_data* sdo)
{
    // CKim - Parse can_frame into SDO_data
    long id;	int len;	char buf[8];

    id = frame.can_id;
    for(int i=0; i<8; i++)   {   buf[i] = frame.data[i];     }

    // CKim - Classify the object based on Command Object ID (COB-ID)
    // Return if the received frame is not SDO frame
    if ((id & COBID_SDO_TX) != COBID_SDO_TX)
    {
        printf("[SocketCAN] Unknown COB-ID 0x%04X  ", id);
        for (int i = 0; i < len; i++)	{
            printf("0x%02X ", (unsigned char)buf[i]); }
        printf("\n");
        return -1;
    }

    // CKim - Parse node id
    id = id - COBID_SDO_TX;
    sdo->nodeid = id;

    // CKim - Parse object index and sub index
    uint16_t idx = 0;	uint8_t subidx = 0;
    for (int i = 0; i < 2; i++)	{
        idx = idx | ((unsigned char)buf[i + 1] << (8 * i));     }
    sdo->index = idx;
    subidx = buf[3];
    sdo->subindex = subidx;

    // CKim - Parse data
    int sz = 0;		int data = 0;	uint32_t errcode = 0;
    unsigned char key = ((unsigned char)buf[0] >> 4);
    //printf("0x%02X\n", key);

    // CKim - Reply from SDO_Read. Save the data
    if (key == 4)
    {
        sz = 4 - ((buf[0] & 0x0F) >> 2);		char* tmp = (char*)&data;
        for (unsigned int i = 0; i < sz; i++)	{ *(tmp + i) = buf[4 + i]; }
        //printf("Read OK : Node 0x%04X index 0x%04X, 0x%02X. Data is 0x%08X\n", id, idx, subidx, data);
        sdo->sz = sz;
        sdo->data = data;
        sdo->errcode = 0;
    }

    // CKim - Reply from SDO_Write
    if (key == 6)
    {
        //printf("Write OK : Node 0x%04X index 0x%04X, 0x%02X\n", id, idx, subidx);
        sdo->errcode = 0;
    }

    // CKim - Reply when SDO_Read / Write fails
    if (key == 8)
    {
        for (int i = 0; i < 4; i++)
        {
            char* tmp = (char*)&errcode;
            for (unsigned int i = 0; i < 4; i++)	{ *(tmp + i) = buf[4 + i]; }
            //printf("0x%02X\n", buf[i+4]);
        }
        printf("SDO Receive Error : Device 0x%04X index 0x%04X, 0x%02X. Error code 0x%08X\n", id, idx, subidx, errcode);
        sdo->sz = 4;
        sdo->errcode = errcode;
    }
    return 0;
}

int LinuxSocketCAN::SendRxPDO(int COBID, int sz, char* buff)
{
    struct can_frame frame;         int nbytes;

    frame.can_id = COBID;
    frame.can_dlc = sz;
    for(int i=0; i<sz; i++) {   frame.data[i] = buff[i];    }

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        m_errMsg = "[SocketCAN] RxPDO send error\n";
        std::cerr << m_errMsg;
        return -1;
    }
    //printf("[SocketCAN] Sent RxPDO Packet. COB-ID : 0x%04X\n",COBID);
    return 0;
}

int LinuxSocketCAN::ReadTxPDO(int& nodeId, int& PdoId, char* buff)
{
    std::ostringstream ostr;
    struct can_frame frame;         int nbytes;

    // CKim - Read CAN frame. SocketCAN uses 'read' functions
    nbytes = read(m_hd,&frame,sizeof(struct can_frame));

    if (nbytes < 0) {
        m_errMsg = "[SocketCAN] read error in ReadTxPDO\n";
        std::cerr << m_errMsg;
        return -1;
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame)) {
        m_errMsg = "[SocketCAN] read incomplete CAN frame in ReadTxPDO\n";
        std::cerr << m_errMsg;
        return -1;
    }

    long id;

    id = frame.can_id;
    for(int i=0; i<8; i++)   {   buff[i] = frame.data[i];     }


    PdoId = (id & COBID_TXPDO1);
    if(PdoId == COBID_TXPDO1)       {   nodeId = id - COBID_TXPDO1;     return 0;   }
    PdoId = (id & COBID_TXPDO2);
    if(PdoId == COBID_TXPDO2)       {   nodeId = id - COBID_TXPDO2;     return 0;   }
    PdoId = (id & COBID_TXPDO3);
    if(PdoId == COBID_TXPDO3)       {   nodeId = id - COBID_TXPDO3;     return 0;   }
    PdoId = (id & COBID_TXPDO4);
    if(PdoId == COBID_TXPDO4)       {   nodeId = id - COBID_TXPDO4;     return 0;   }

    // CKim - Otherwise. Non. PDO COB-ID
     printf("[SocketCAN] invalid COB-ID 0x%04X in ReadTxPDO\n", frame.can_id);
     return -1;
}


//// CKim - Function that runs in separate thread and continuously read incoming CAN packets
//void* LinuxSocketCAN::CAN_ReadThread(void* pData)
//{
//    // CKim - Handle to the class is passed as a parameter to thread function.
//    //LinuxSocketCAN* ptr = (LinuxSocketCAN*) pData;

//    // CKim - Create event Object for signaling SDO read/write
//    //m_hSDO_Event = CreateEvent(NULL, FALSE, FALSE, _T("SDOEvent"));	//	null security, auto reset, initially non-signaled, name
//    sem_init(&m_hSDO_Sema,0,0);

//    // CKim - Create event Object for signaling PDO read/write
//    //m_hPDO_Event = CreateEvent(NULL, FALSE, FALSE, _T("PDOEvent"));	//	null security, auto reset, initially non-signaled, name
//    //sem_init(&(ptr->m_hPDO_Sema),0,0);

//    // CKim - Receive response
//    long id;	int len;	char buf[8];	//int ext, rtr;

//    struct can_frame frame;     int nbytes;

//    while (1)
//    {
//        // -------------------------------------------------------------
//        // CKim - Read CAN frame. SocketCAN uses 'read' functions
//        nbytes = read(m_hd,&frame,sizeof(struct can_frame));

//        if (nbytes < 0) {
//            perror("can raw socket read");
//            break;//return 1;
//        }

//        /* paranoid check ... */
//        if (nbytes < sizeof(struct can_frame)) {
//            fprintf(stderr, "read: incomplete CAN frame\n");
//            break;//return 1;
//        }

//        id = frame.can_id;
//        for(int i=0; i<8; i++)   {   buf[i] = frame.data[i];     }
//        // -------------------------------------------------------------

//        // -------------------------------------------------------------
//        // CKim - Classify the object based on Command Object ID (COB-ID)
//        if ((id & SDO_TX) == SDO_TX)
//        {
//            id = id - SDO_TX;
//            m_RcvSDOdata.nodeid = id;

//            uint16_t idx = 0;	uint8_t subidx = 0;		int sz = 0;		int data = 0;	uint32_t errcode = 0;

//            unsigned char key = ((unsigned char)buf[0] >> 4);
//            //printf("0x%02X\n", key);

//            for (int i = 0; i < 2; i++)	{
//                idx = idx | ((unsigned char)buf[i + 1] << (8 * i));
//            }
//            m_RcvSDOdata.index = idx;

//            subidx = buf[3];
//            m_RcvSDOdata.subindex = subidx;

//            // CKim - Reply from SDO_Read
//            if (key == 4)
//            {
//                sz = 4 - ((buf[0] & 0x0F) >> 2);		char* tmp = (char*)&data;
//                for (unsigned int i = 0; i < sz; i++)	{ *(tmp + i) = buf[4 + i]; }
//                //printf("Read OK : Node 0x%04X index 0x%04X, 0x%02X. Data is 0x%08X\n", id, idx, subidx, data);
//                m_RcvSDOdata.sz = sz;
//                m_RcvSDOdata.data = data;
//                m_RcvSDOdata.errcode = 0;
//                //SetEvent(m_hSDO_Event);
//                sem_post(&m_hSDO_Sema);
//            }

//            // CKim - Reply from SDO_Write
//            if (key == 6)
//            {
//                //printf("Write OK : Node 0x%04X index 0x%04X, 0x%02X\n", id, idx, subidx);
//                m_RcvSDOdata.errcode = 0;
//                //SetEvent(m_hSDO_Event);
//                sem_post(&m_hSDO_Sema);
//            }

//            // CKim - Reply when SDO_Read / Write fails
//            if (key == 8)
//            {
//                for (int i = 0; i < 4; i++)
//                {
//                    char* tmp = (char*)&errcode;
//                    for (unsigned int i = 0; i < 4; i++)	{ *(tmp + i) = buf[4 + i]; }
//                    //printf("0x%02X\n", buf[i+4]);
//                }
//                printf("SDO Receive Error : Device 0x%04X index 0x%04X, 0x%02X. Error code 0x%08X\n", id, idx, subidx, errcode);
//                m_RcvSDOdata.sz = 4;
//                m_RcvSDOdata.errcode = errcode;
//                //SetEvent(m_hSDO_Event);
//                sem_post(&m_hSDO_Sema);
//            }
//        }
//        else if ((id & TX_PDO1) == TX_PDO1)
//        {
//            // CKim - Notify device arrival of TxPDO
//            int nodeId = id - TX_PDO1;
//            //printf("TX_PDO1 from node %d\n", nodeId);
//            m_pDev[nodeId-1]->NotifyPDO(0,buf);
//        }
//        else if ((id & TX_PDO2) == TX_PDO2)
//        {
//            // CKim - Notify device arrival of TxPDO
//            int nodeId = id - TX_PDO2;
//            //printf("TX_PDO2 from node %d\n", nodeId);
//            m_pDev[nodeId-1]->NotifyPDO(1,buf);
//        }
//        else if ((id & TX_PDO3) == TX_PDO3)
//        {
//            // CKim - Notify device arrival of TxPDO
//            int nodeId = id - TX_PDO3;
//            //printf("TX_PDO3 from node %d\n", nodeId);
//            m_pDev[nodeId-1]->NotifyPDO(2,buf);
//        }
//        else if ((id & TX_PDO4) == TX_PDO4)
//        {
//            // CKim - Notify device arrival of TxPDO
//            int nodeId = id - TX_PDO4;
//            //printf("TX_PDO4 from node %d\n", nodeId);
//            m_pDev[nodeId-1]->NotifyPDO(3,buf);
//        }
//        else
//        {
//            // CKim - Other Data
//            printf("Unknown COB-ID 0x%04X  ", id);
//            for (int i = 0; i < len; i++)	{ printf("0x%02X ", (unsigned char)buf[i]); }
//            printf("\n");
//        }
//    }	// while

//}
