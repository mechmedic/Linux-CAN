// ------------------------------------------------------------------------------
// CKim - 2019. Sep.16 : Common defines for the C++ class encapsulating CAN
// interface for various OS + CAN port devices
// 1. Linux + SocketCAN : https://en.wikipedia.org/wiki/SocketCAN
// 2. Windows + USB2CAN : http://www.devicemart.co.kr/goods/view?no=1323536
// ------------------------------------------------------------------------------

#include <stdint.h>

#ifndef CANDEFINES_H
#define CANDEFINES_H

#define SDO_READ    0
#define SDO_WRITE   1

// CKim - Data used in Network Management (NMT) objects to set the state of the slave (EPOS) devices
#define NMT_PREOPERATIONAL  0x80
#define NMT_OPERATIONAL     0x01

// CKim - Command Object ID (COB-ID) of the SDO (Service Data Object) TX (from EPOS to PC) RX (from PC to EPOS)
enum SDO_ID {
    SDO_TX = 0x580, /* +node id */
    SDO_RX = 0x600  /* +node id */
};

// CKim - Command Object ID (COB-ID) of the PDO (Processe Data Object) TX (from EPOS to PC)
enum TX_PDO_ID {
    TX_PDO1 = 0x00000180, /* +node id */
    TX_PDO2 = 0x00000280, /* +node id */
    TX_PDO3 = 0x00000380, /* +node id */
    TX_PDO4 = 0x00000480  /* +node id */
};

// CKim - Command Object ID (COB-ID) of the PDO (Processe Data Object) RX (from PC to EPOS)
enum RX_PDO_ID {
    RX_PDO1 = 0x00000200, /* +node id */
    RX_PDO2 = 0x00000300, /* +node id */
    RX_PDO3 = 0x00000400, /* +node id */
    RX_PDO4 = 0x00000500  /* +node id */
};

// CKim - SDO_data Structure holding all data needed to send an SDO object */
typedef struct {
    uint16_t nodeid;	// Node id to send data to
    uint16_t index;		// Index in Object dictionary
    uint8_t subindex;	// Subindex in Object dictionary
    uint32_t sz;		// Size
    uint32_t data;		// Actual data
    uint32_t errcode;	// Error code
} SDO_data;

// CKim - Function for calculating 'Command Specifier' (byte 0 of the SDO packet)
inline uint8_t SDO_calculate_ccs(char rw, int size)
{
    uint8_t base = 0x40;
    if (rw == 'w') {	base = 0x20;	}

    switch (size) {
        case 1: return base + 0x0F;
        case 2: return base + 0x0B;
        case 3: return base + 0x07;
        case 4: return base + 0x03;
    }
    return 0;
};

#endif // CANDEFINES_H
