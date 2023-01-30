/**
 ***************************************************************************
 * \file  CANopenDefs.h
 * \brief Common header files defining CANopen related data structures
 * and functions
 *
 * Last Updated : 2023.01.26 Chunwoo Kim (CKim)
 * Contact Info : cwkim@kist.re.kr
 ***************************************************************************
**/

#include <stdint.h>

#ifndef CANOPENDEFS_H
#define CANOPENDEFS_H

#define SDO_READ    0
#define SDO_WRITE   1

#define TXPDO_TRANSMIT_ON_SYNC      1
#define TXPDO_TRANSMIT_ON_RTR       253
#define TXPDO_TRANSMIT_ON_CHANGE    255

#define RXPDO_UPDATE_ON_SYNC      1
#define RXPDO_UPDATE_IMMED        255

// CKim - Data used in Network Management (NMT) objects to set the state of the slave (EPOS) devices
#define NMT_PREOPERATIONAL  0x80
#define NMT_OPERATIONAL     0x01

// CKim - Object Dictionary Indexes
#define OD_TPDO1_PARAM                  0X1800	 // RW: uint8_t   check EPOS4 Firmware Specification pg. 247
#define OD_TPDO2_PARAM                  0X1801
#define OD_TPDO3_PARAM                  0X1802
#define OD_TPDO4_PARAM                  0X1803

#define OD_TPDO1_MAP                    0X1A00
#define OD_TPDO2_MAP                    0X1A01
#define OD_TPDO3_MAP                    0X1A02
#define OD_TPDO4_MAP                    0X1A03

#define OD_RPDO1_PARAM                  0X1400	 // RW: uint8_t   check EPOS4 Firmware Specification pg. 247
#define OD_RPDO2_PARAM                  0X1401
#define OD_RPDO3_PARAM                  0X1402
#define OD_RPDO4_PARAM                  0X1403

#define OD_RPDO1_MAP                    0X1600	 // RW: uint8_t   check EPOS4 Firmware Specification pg. 247
#define OD_RPDO2_MAP                    0X1601
#define OD_RPDO3_MAP                    0X1602
#define OD_RPDO4_MAP                    0X1603

#define OD_STATUS_WORD                  0x6041
#define OD_CONTROL_WORD                 0x6040

#define OD_TARGET_POSITION              0x607A

#define OD_POSITION_ACTUAL_VAL          0x6064
#define OD_VELOCITY_ACTUAL_VAL          0x606C
#define OD_CURRENT_ACTUAL_VAL           0x6078


// CKim - Command Object IDs (COB-ID)
// SDO (Service Data Object) TX (from EPOS to PC) RX (from PC to EPOS)
#define COBID_SYNC      0x00000080
#define COBID_NMT       0x00000000

#define COBID_SDO_TX    0x00000580 /* +node id */
#define COBID_SDO_RX    0x00000600  /* +node id */

#define COBID_TXPDO1    0x00000180 /* +node id */
#define COBID_TXPDO2    0x00000280 /* +node id */
#define COBID_TXPDO3    0x00000380 /* +node id */
#define COBID_TXPDO4    0x00000480 /* +node id */

#define COBID_RXPDO1    0x00000200 /* +node id */
#define COBID_RXPDO2    0x00000300 /* +node id */
#define COBID_RXPDO3    0x00000400 /* +node id */
#define COBID_RXPDO4    0x00000500 /* +node id */


// CKim - SDO_data Structure holding all data needed to send an SDO object
typedef struct {
    uint16_t nodeid;	// Node id to send data to
    uint16_t index;		// Index in Object dictionary
    uint8_t subindex;	// Subindex in Object dictionary
    uint32_t sz;		// Size
    uint32_t data;		// Actual data
    uint32_t errcode;	// Error code
} SDO_data;

// CKim - Structure holding TxPDO / RxPDO parameters. This should be written
// to object dictionary index 0X1800-0x1803 to configure (TxPDO1-4)
// COB_ID : SubIndex 0x01 (uint_32)
//          Communication Object ID (COB-ID) of this TxPDO.
// TransmissionType : SubIndex 0x02 (uint_8). For RxPDO, only 1 or 255.
//          Decides when the slave will send this PDO.
//          1 : Transmit when the slave receives SYNC object (sychronous)
//          253 : Transmit when requested by RTR (remote transmission) object. (polling)
//                Not recommended.
//          255. Transmit when the value of the object changes. (asynch)
// InhibitTime : SubIndex 03 (uint_16). Only for TxPDO
//          Use only when TransmissionType is 255.
//          Value of minimum interval between the event triggering PDO transmission.
//          and actual transmission. multiples of 100 us
typedef struct {
    uint32_t    COB_ID;
    uint8_t     TransmissionType;
    uint16_t    InhibitTime;
} TxPDO_Param;

typedef struct {
    uint32_t    COB_ID;
    uint8_t     TransmissionType;
} RxPDO_Param;

// CKim - Structure holding Mapping of TxPDO1-4
// This should be written to object dictionary index 0x1A00~0x1A03
// NumberOfMappedObject : SubIndex 0x00 (uint_8)
//          Number of objects mapped. Set this to 0 to disable PDO.
//          Up to 8 objects, total size up to 64 bit can be mapped.
// ObjIdx / ObjSubIdx / ObjSz :
//          Array containing up to 8 object index, subindex and data size in bits
//          that will be mapped to PDO. These three information is
//          combined into 32 bit data stored in SubIndex 0x01 to 0x08
typedef struct {
    uint8_t     NumberOfMappedObject;
    uint16_t    ObjIdx[8];
    uint8_t     ObjSubIdx[8];
    uint8_t     ObjSz[8];
} TxPDO_Mapping;

typedef struct {
    uint8_t     NumberOfMappedObject;
    uint16_t    ObjIdx[8];
    uint8_t     ObjSubIdx[8];
    uint8_t     ObjSz[8];
} RxPDO_Mapping;

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

//offset for PDO entries to register PDOs.
typedef struct
{
    uint32_t target_pos ;
    uint32_t target_vel ;
    uint32_t target_tor ;
    uint32_t max_tor  ;
    uint32_t control_word ;
    uint32_t op_mode ;
    uint32_t profile_acc ;
    uint32_t profile_dec ;
    uint32_t quick_stop_dec ;
    uint32_t profile_vel ;

    uint32_t actual_pos ;
    uint32_t pos_fol_err ;
    uint32_t actual_vel ;
    uint32_t actual_cur ;
    uint32_t actual_tor ;
    uint32_t status_word ;
    uint32_t op_mode_display ;
    uint32_t error_code ;
    uint32_t extra_status_reg ;

    uint32_t r_limit_switch;
    uint32_t l_limit_switch;
    uint32_t emergency_switch;
} OffsetPDO ;


// Received feedback data from slaves
typedef struct
{
    int32_t   target_pos ;
    int32_t   target_vel ;
    int16_t   target_tor ;
    int16_t   max_tor ;
    uint16_t  control_word ;
    //OpMode    op_mode ;
    int32_t   vel_offset ;
    int16_t   tor_offset ;

    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    int8_t   op_mode_display ;
    uint8_t  left_limit_switch_val ;
    uint8_t  right_limit_switch_val ;
    uint8_t  s_emergency_switch_val;
}ReceivedData;



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

#endif // CANOPENDEFS_H
