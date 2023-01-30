/**
 ***************************************************************************
 * \file  canslave.h
 * \brief Class implementing Motion controller (ex. EPOS) as a CANopen slave
 *
 * Last Updated : 2023.01.27 Chunwoo Kim (CKim)
 * Contact Info : cwkim@kist.re.kr
 ***************************************************************************
**/

#ifndef CANSLAVE_H
#define CANSLAVE_H


#include "CANopenDefs.h"

class CANSlave
{
public:
    CANSlave();
    ~CANSlave();

    // CKim - ID of the EPOS controller in CAN
    int			m_nodeId;

    // CKim - Mode of the EPOS controller (profile position, homing, velocity ...)
    int			m_mode;

    /// Offset for PDO entries to assign pdo registers.
    OffsetPDO                 offset_ ;
    /// Received data from servo drivers.
    ReceivedData              data_ ;


    /// Slave velocity parameters.
    ProfileVelParam    velocity_param_ ;
    /// Slave position parameters.
    ProfilePosParam         position_param_ ;
    // Slave homing parameters.
    HomingParam             homing_param_ ;

    TxPDO_Param     TpdoParam[4];
    TxPDO_Mapping   TpdoMap[4];
    RxPDO_Param     RpdoParam[4];
    RxPDO_Mapping   RpdoMap[4];
};

#endif // EPOSSLAVE_H




//class EthercatSlave
//{
//    public:
//        EthercatSlave();
//        ~EthercatSlave();
//    /**
//     * @brief This function will check slave's application layer states.
//     *        (INIT/PREOP/SAFEOP/OP)
//     * @note This function shouldn't be called in real time context.For diagnosis
//     *       you can use CheckDomainState() encapsulation in ecat_node.
//     * @return 0 if succesful.
//     */
//    int CheckSlaveConfigState();


//    /// DC sync shift setting, zero will give best synchronization.
//    const static uint32_t   kSync0_shift_ = 0;

//    /// Slave configuration parameters, assinged to each slave.
//    ec_slave_config_t       * slave_config_ ;

//    /// Slave state handle to check if slave is online and slaves state machine status(INIT/PREOP/SAFEOP/0P)
//    ec_slave_config_state_t  slave_config_state_ ;

//    /// PDO domain for data exchange
//    uint8_t                * slave_pdo_domain_ ;

//    /// Variable for checking motor state
//    int32_t                  motor_state_ ;
//    /**
//     * @brief Slave information data structure.
//     *      This structure contains all information related to slave.
//     *      It will be used to get slave's information from master.
//     */
//    ec_slave_info_t         slave_info_;


//};// EthercatSlave class
