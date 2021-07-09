#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     // Posix Header
#include <string.h>
#include <errno.h>

#include "linuxsocketcan.h"
#include "EposCAN.h"

// CKim - Initialize static member variables
int LinuxSocketCAN::m_hd = -1;                // CKim - Socket Handle. For pi2can
sem_t LinuxSocketCAN::m_hSDO_Sema;
pthread_t LinuxSocketCAN::m_hReadThrd;
SDO_data LinuxSocketCAN::m_RcvSDOdata;
EposCAN* LinuxSocketCAN::m_pDev[10];
std::string LinuxSocketCAN::m_portName;

LinuxSocketCAN::LinuxSocketCAN()
{

}

LinuxSocketCAN::~LinuxSocketCAN()
{

}

// CKim - Open CAN port
int LinuxSocketCAN::OpenCANport(const char* portName)
{
    if(m_hd != -1)      {   return 0;   }

    // ---------------------------------------------------------------
    // CKim - Open Socket for CAN communication, as it is done in the example for pi2can
    m_hd = socket(PF_CAN, SOCK_RAW, CAN_RAW);         // CKim - Opens handle to socket
    if (m_hd < 0) {
        perror("socket");
        return -1;
    }

    // CKim - Configure sockaddr (Socket Address) structure for CAN sockets
    // 'ifreq' is a Interface Request struct defined in if.h of linux kernel
    // ioctl manipulates io handles...
    struct sockaddr_can addr;       addr.can_family = AF_CAN;
    struct ifreq ifr;               strcpy(ifr.ifr_name, portName);
    if (ioctl(m_hd, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        return -1;
    }
    addr.can_ifindex = ifr.ifr_ifindex;

    // CKim - Bind address to the socket.
    if (bind(m_hd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return -1;
    }
    m_portName = portName;
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // CKim - Launch CAN Read Thread
    pthread_create(&m_hReadThrd, NULL, LinuxSocketCAN::CAN_ReadThread, 0);
    // ---------------------------------------------------------------

    return 0;
}

int LinuxSocketCAN::CloseCANport()
{
    return(close(m_hd));
}

// CKim - Write / Read SDO_data : Blocks until reply is received. return  0 on success, -1 on error, -2 on timeout
// CAN Master PC is Client and the connected slave devices (EPOS Controller) are Server.
// Client (master) sends SDO to server (slave) to
// 1. rw == SDO_WRITE to write data to the Object Dictionary of the EPOS (slave)
// 2. rw == SDO_READ to read from the Object Dictionary of the EPOS (slave)
int LinuxSocketCAN::SendSDO(SDO_data* d, int rw)
{
    if( rw!=SDO_READ && rw!=SDO_WRITE ) {   return -1;  }

    // ------------------------------------------------------------
    // CKim - Build buffer for sending data. Linux already has struct can_frame for CAN data frames
    // so I copy my SDO_data struct to can_frame struct
    struct can_frame frame;         int nbytes;

    // CKim - Byte 0-3. id, command specifier and index
    if(rw == SDO_WRITE)
    {
        frame.can_dlc = 4 + d->sz;
        frame.data[0] = SDO_calculate_ccs('w', d->sz);      // Byte 0 : Command Specifier

        // CKim - Byte 4-7. Data
        for(int i=0; i<4; i++)
        {
            char* tmp = (char*)&(d->data);//memcpy()
            if (i<d->sz)	{ frame.data[4 + i] = *(tmp + i); }
            else			{ frame.data[4 + i] = 0; }
        }
    }
    else if (rw == SDO_READ)
    {
        frame.can_dlc = 4;
        frame.data[0] = 0x40;                       // Byte 0 : Command Specifier

        // CKim - Byte 4-7. Data
        for(int i=0; i<4; i++)  {   frame.data[4 + i] = 0;  }
    }
    else {
        return -1;
    }

    frame.can_id = SDO_RX + d->nodeid;
    frame.data[1] = (0xFF & (d->index));				// Byte 1 : Low byte of the index
    frame.data[2] = (d->index) >> 8;					// Byte 2 : High byte of the index
    frame.data[3] = d->subindex;						// Byte 3 : Subindex

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        //perror("write");
        m_errMsg = "CAN frame write Error\n";
        return -1;
    }

    printf("SentSDO Packet. NodeID : 0x%04X  ObjIdx : 0x%04X\n",frame.can_id, d->index);
    // ------------------------------------------------------------

    // ---------------------------------------------------------- //
    // CKim - Wait for the reply to the SDO Command. Windows used Event
    // In Linux, I'm using semaphore with 1 sec timeout
    struct timespec ts;     int err = 0;
    clock_gettime(CLOCK_REALTIME,&ts);
    ts.tv_sec += 1;
    err = sem_timedwait(&m_hSDO_Sema,&ts);
    if(err == -1)   {
        if(errno == ETIMEDOUT)  {   m_errMsg = "SDO receive timed out\n";     }
        else                    {   m_errMsg = "Semaphore waiting error\n";     }
        return -1;
    }
    // ---------------------------------------------------------- //

    // ---------------------------------------------------------- //
    // CKim - Process the SDO received from Server (EPOS)
    // Check error code
    if (m_RcvSDOdata.errcode != 0)
    {
        char str[128];
        sprintf(str, "SDO Receive Error : Device 0x%04X index 0x%04X, 0x%02X. Error code 0x%08X\n",
            m_RcvSDOdata.nodeid, m_RcvSDOdata.index, m_RcvSDOdata.subindex, m_RcvSDOdata.errcode);
        m_errMsg = str;
        return -1;
    }
    else if ((m_RcvSDOdata.nodeid != d->nodeid) || (m_RcvSDOdata.index != d->index) || (m_RcvSDOdata.subindex != d->subindex))
    {
        m_errMsg = "Wrong SDO reply index!\n";
        return -1;
    }
    else
    {
        // CKim - On successful SDO receive
        if(rw == SDO_WRITE)
        {
//            printf("Write OK : Device 0x%04X index 0x%04X, 0x%02X. Data 0x%04X\n",
//                m_RcvSDOdata.nodeid, m_RcvSDOdata.index, m_RcvSDOdata.subindex, d->data);
        }
        if(rw == SDO_READ)
        {
            d->sz = m_RcvSDOdata.sz;	d->data = m_RcvSDOdata.data;
//            printf("Read OK : Device 0x%04X index 0x%04X, 0x%02X. %d byte data is 0x%08X\n",
//                m_RcvSDOdata.nodeid, m_RcvSDOdata.index, m_RcvSDOdata.subindex, d->data);
        }
        return 0;
    }
    // ---------------------------------------------------------- //
}

// CKim - Send SYNC object to all the devices in network. This is used for
// synchronizing PDO transport of all the connected devices.
int LinuxSocketCAN::SendSYNC()
{
    // CKim - Communication Object ID (COB-ID) of SYNC object
    uint16_t cobIDforSYNC = 0x00000080;

    struct can_frame frame;         int nbytes;

    frame.can_id = cobIDforSYNC;
    frame.can_dlc = 0;

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("write");
        return -1;
    }

    printf("SentSYNC\n");
    return 0;
}

// CKim - Send NMT Service message to switch the slave's (EPOS) state
// between Operational and Pre-Operational. This is used to Enable / Disable PDO
int LinuxSocketCAN::SendNMT(int state, int nodeId)
{
    // CKim - Switch slave to 'Operational' state.
    // COB-ID = 0, data[0] = 0x01, data[1] = nodeId, (0 for all)
    // NMT operational state is indicated by bit 9 of the status word.
    uint16_t cobIDforNMT = 0;       // COB-ID for NMT message

    struct can_frame frame;         int nbytes;

    frame.can_id = cobIDforNMT;
    frame.can_dlc = 2;
    frame.data[0] = state;
    frame.data[1] = nodeId;

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("write");
        return -1;
    }

    printf("Sent NMT message\n");
    return 0;
}

// CKim - Send RxPDO Object
int LinuxSocketCAN::SendRxPDO(int COBID, int sz, char* buff)
{
    struct can_frame frame;         int nbytes;

    frame.can_id = COBID;
    frame.can_dlc = sz;
    for(int i=0; i<sz; i++) {   frame.data[i] = buff[i];    }

    // CKim - Use write() on socket to send CAN data
    nbytes = write(m_hd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("write");
        return -1;
    }

    //printf("Sent RxPDO\n");

    return 0;
}

// CKim - Function that runs in separate thread and continuously read incoming CAN packets
void* LinuxSocketCAN::CAN_ReadThread(void* pData)
{
    // CKim - Handle to the class is passed as a parameter to thread function.
    //LinuxSocketCAN* ptr = (LinuxSocketCAN*) pData;

    // CKim - Create event Object for signaling SDO read/write
    //m_hSDO_Event = CreateEvent(NULL, FALSE, FALSE, _T("SDOEvent"));	//	null security, auto reset, initially non-signaled, name
    sem_init(&m_hSDO_Sema,0,0);

    // CKim - Create event Object for signaling PDO read/write
    //m_hPDO_Event = CreateEvent(NULL, FALSE, FALSE, _T("PDOEvent"));	//	null security, auto reset, initially non-signaled, name
    //sem_init(&(ptr->m_hPDO_Sema),0,0);

    // CKim - Receive response
    long id;	int len;	char buf[8];	//int ext, rtr;

    struct can_frame frame;     int nbytes;

    while (1)
    {
        // -------------------------------------------------------------
        // CKim - Read CAN frame. SocketCAN uses 'read' functions
        nbytes = read(m_hd,&frame,sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("can raw socket read");
            break;//return 1;
        }

        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "read: incomplete CAN frame\n");
            break;//return 1;
        }

        id = frame.can_id;
        for(int i=0; i<8; i++)   {   buf[i] = frame.data[i];     }
        // -------------------------------------------------------------

        // -------------------------------------------------------------
        // CKim - Classify the object based on Command Object ID (COB-ID)
        if ((id & SDO_TX) == SDO_TX)
        {
            id = id - SDO_TX;
            m_RcvSDOdata.nodeid = id;

            uint16_t idx = 0;	uint8_t subidx = 0;		int sz = 0;		int data = 0;	uint32_t errcode = 0;

            unsigned char key = ((unsigned char)buf[0] >> 4);
            //printf("0x%02X\n", key);

            for (int i = 0; i < 2; i++)	{
                idx = idx | ((unsigned char)buf[i + 1] << (8 * i));
            }
            m_RcvSDOdata.index = idx;

            subidx = buf[3];
            m_RcvSDOdata.subindex = subidx;

            // CKim - Reply from SDO_Read
            if (key == 4)
            {
                sz = 4 - ((buf[0] & 0x0F) >> 2);		char* tmp = (char*)&data;
                for (unsigned int i = 0; i < sz; i++)	{ *(tmp + i) = buf[4 + i]; }
                //printf("Read OK : Node 0x%04X index 0x%04X, 0x%02X. Data is 0x%08X\n", id, idx, subidx, data);
                m_RcvSDOdata.sz = sz;
                m_RcvSDOdata.data = data;
                m_RcvSDOdata.errcode = 0;
                //SetEvent(m_hSDO_Event);
                sem_post(&m_hSDO_Sema);
            }

            // CKim - Reply from SDO_Write
            if (key == 6)
            {
                //printf("Write OK : Node 0x%04X index 0x%04X, 0x%02X\n", id, idx, subidx);
                m_RcvSDOdata.errcode = 0;
                //SetEvent(m_hSDO_Event);
                sem_post(&m_hSDO_Sema);
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
                m_RcvSDOdata.sz = 4;
                m_RcvSDOdata.errcode = errcode;
                //SetEvent(m_hSDO_Event);
                sem_post(&m_hSDO_Sema);
            }
        }
        else if ((id & TX_PDO1) == TX_PDO1)
        {
            // CKim - Notify device arrival of TxPDO
            int nodeId = id - TX_PDO1;
            //printf("TX_PDO1 from node %d\n", nodeId);
            m_pDev[nodeId-1]->NotifyPDO(0,buf);
        }
        else if ((id & TX_PDO2) == TX_PDO2)
        {
            // CKim - Notify device arrival of TxPDO
            int nodeId = id - TX_PDO2;
            //printf("TX_PDO2 from node %d\n", nodeId);
            m_pDev[nodeId-1]->NotifyPDO(1,buf);
        }
        else if ((id & TX_PDO3) == TX_PDO3)
        {
            // CKim - Notify device arrival of TxPDO
            int nodeId = id - TX_PDO3;
            //printf("TX_PDO3 from node %d\n", nodeId);
            m_pDev[nodeId-1]->NotifyPDO(2,buf);
        }
        else if ((id & TX_PDO4) == TX_PDO4)
        {
            // CKim - Notify device arrival of TxPDO
            int nodeId = id - TX_PDO4;
            //printf("TX_PDO4 from node %d\n", nodeId);
            m_pDev[nodeId-1]->NotifyPDO(3,buf);
        }
        else
        {
            // CKim - Other Data
            printf("Unknown COB-ID 0x%04X  ", id);
            for (int i = 0; i < len; i++)	{ printf("0x%02X ", (unsigned char)buf[i]); }
            printf("\n");
        }
    }	// while

}
