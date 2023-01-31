#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include "EposCAN.h"

using namespace std;

int res;
int numNodes = NUM_NODE;
EposCAN* pDev;


//sem_t sema;

int TestProfilePosition()
{
    // ---------------------------------------------------------------
    // CKim - Enable Position Control Mode
    ProfilePosParam PosPara;
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->SetOperationMode(i, PROFILE_POS);
        if (res == -1)
        {
            printf("Failed to set PosProfile Mode, device %d\n", pDev->GetId());
            return -1;
        }

        res = pDev->GetPositionProfileParam(i, PosPara);
        if (res == -1)
        {
            printf("Param Reading Error\n");
            return -1;
        }
        else
        {
            printf("Node %d ProfilePos Params\n",i);
            printf("MaxVel: %d,  QuickDecel: %d,  Acc: %d,  Decel: %d  Following Errr: %d  Vel: %d\n",
                   PosPara.MaxProfileVelocity, PosPara.QuickStopDecel, PosPara.ProfileAccel, PosPara.ProfileDecel,
                   PosPara.MaxFollowingError, PosPara.ProfileVelocity);
        }

        PosPara.ProfileAccel = PosPara.ProfileDecel = 10000;
        PosPara.MaxProfileVelocity = 5000;

        res = pDev->SetPositionProfileParam(i,PosPara);
        if (res == -1)
        {
            printf("Param Setting Error\n");
            return -1;
        }

        printf("Control : 0x%04X, Status : 0x%04X\n", pDev->ReadControlWord(i), pDev->ReadStatusWord(i));
        printf("Current Position : %d\n", pDev->ReadPosition(i));
    }

    sleep(2);
    printf("Starting Motion\n");

    // CKim - Move to one position
    struct timespec start, end;         double elapsedTime;     int pos;

    // CKim - Measure time when sending target position by SDO
    pos = 500000;
    clock_gettime(CLOCK_REALTIME,&start);
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->MovePosProfile(i, pos, true);
        //res = pDev[0]->MovePosProfile(pos, false);
        //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
        if(res == -1)
        {
            printf("MovePosProfile Error\n");
        }
    }
    //res = pDev->WaitForMotionCompletion(1,5000);
    //res = pDev->WaitForMotionCompletion(1,500);
    res = pDev->WaitForMotionCompletionAll(500);
    if(res!=0) {
        //res = pDev->HaltAxis(0);
        res = pDev->HaltAxisAll();
    }

    clock_gettime(CLOCK_REALTIME,&end);
    elapsedTime = (end.tv_sec - start.tv_sec);
    elapsedTime = elapsedTime + (end.tv_nsec - start.tv_nsec)*1e-9;
    cout<< "Time taken is : " << fixed << elapsedTime << endl;

    //sleep(5);

    for (int i=0; i<NUM_NODE; i++)
    {
        printf("Control : 0x%04X, Status : 0x%04X\n", pDev->ReadControlWord(i), pDev->ReadStatusWord(i));
        printf("Current Position : %d\n", pDev->ReadPosition(i));
    }

    return 0;
}

int TestProfileVelocity()
{
    // ------------------------------------------------
    // CKim - Setup Profile Velocity Control Mode
    ProfileVelParam VelPara;
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->SetOperationMode(i, PROFILE_VEL);
        res = pDev->GetVelocityProfileParam(i, VelPara);
        if (res == -1)
        {
            printf("Param Reading Error\n");
            return -1;
        }
        else
        {
            printf("ProfileVel Params\n");
            printf("MaxVel: %d,  QuickDecel: %d,  Acc: %d,  Decel: %d  Type: %d\n",VelPara.MaxProfileVelocity,
                   VelPara.QuickStopDecel, VelPara.ProfileAccel, VelPara.ProfileDecel, VelPara.MotionProfileType);
        }

        VelPara.ProfileAccel = VelPara.ProfileDecel = 5000;

        res = pDev->SetVelocityProfileParam(i, VelPara);
        if (res == -1)
        {
            printf("Param Setting Error\n");
            return -1;
        }
    }

    // CKim - Move one direction
    printf("Moving Forward\n");
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->MoveVelProfile(i, 5000);
        if (res == -1)
        {
            printf("MoveVelProfile error\n");
            return -1;
        }
    }
    sleep(10);

    // CKim - Move the other direction
    printf("Moving Backward\n");
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->MoveVelProfile(i, -5000);
        if (res == -1)
        {
            printf("MoveVelProfile error\n");
            return -1;
        }
    }
    sleep(10);

    // CKim - Stop
    printf("Stopping\n");
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->HaltAxis(i);
        if (res == -1)
        {
            printf("MoveVelProfile error\n");
            return -1;
        }
    }
    sleep(5);
    return 0;
}

int TestHoming()
{
    return 1;
    //	// --------------------------------------------------------
    //	// CKim - Set Homing mode
    //	for (int i = 0; i < 3; i++)	{
    //		pDev[i]->SetOperationMode(HOMING);	}

    //	// CKim - Set Homing parameter
    //	HomingParam hPara;
    //	hPara.MaxFollowingError = 20000;	hPara.MaxProfileVelocity = 8000;	hPara.QuickStopDecel = 40000;
    //	hPara.SpeedForSwitchSearch = 500;	hPara.SpeedForZeroSearch = 100;		hPara.HomingAccel = 500;
    //	hPara.CurrentThresholdHoming = 100;	hPara.HomeOffset = (int)(5 / 1.5 * 11 * 1000 * 4 * 1 / 3);	hPara.HomingMethod = 7;

    //	for (int i = 0; i < 3; i++) {
    //		pDev[i]->SetHomingParam(hPara);
    //	}

    //	// CKim - Start Homing. Start from device 3.
    //	for (int i = 2; i > -1; i--) {
    //		pDev[i]->StartHoming();
    //		Sleep(2000);
    //	}

    //	// CKim - Wait until homing ends
    //	bool homingDone = false;
    //	while (!homingDone)
    //	{
    //		int cnt = 0;
    //		for (int i = 0; i < 3; i++)
    //		{
    //			if (pDev[i]->IsHomingAttained())	{ cnt++; }
    //			if (cnt == 3)
    //			{
    //				printf("Homing Complete\n");	homingDone = true;
    //			}
    //		}
    //		Sleep(100);
    //	}
    //	Sleep(2000);
    //	// --------------------------------------------------------

}

int TestOscillatingMotion()
{
//    // ---------------------------------------------------------------
//    // CKim - Enable Position Control Mode
//    ProfilePosParam PosPara;
//    for (int i=0; i<numNodes; i++)
//    {
//        res = pDev[i]->SetOperationMode(PROFILE_POS);
//        if (res == -1)
//        {
//            printf("Failed to set PosProfile Mode, device %d\n", pDev[i]->GetId());
//            cout << pDev[i]->GetErrorMsg() << endl;
//            return 0;
//        }

//        res = pDev[i]->GetPositionProfileParam(PosPara);
//        if (res == -1)
//        {
//            printf("Param Reading Error\n");
//            cout << pDev[i]->GetErrorMsg() << endl;
//            return 0;
//        }
//        else
//        {
//            printf("ProfilePos Params\n");
//            printf("MaxVel: %d,  QuickDecel: %d,  Acc: %d,  Decel: %d  Following Errr: %d  Vel: %d\n",
//                   PosPara.MaxProfileVelocity, PosPara.QuickStopDecel, PosPara.ProfileAccel, PosPara.ProfileDecel,
//                   PosPara.MaxFollowingError, PosPara.ProfileVelocity);
//        }

//        PosPara.ProfileAccel = PosPara.ProfileDecel = 10000;
//        PosPara.MaxProfileVelocity = 50000;
//        PosPara.ProfileVelocity = 50000;

//        res = pDev[i]->SetPositionProfileParam(PosPara);
//        if (res == -1)
//        {
//            printf("Param Setting Error\n");
//            cout << pDev[i]->GetErrorMsg() << endl;
//            return 0;
//        }
//    }


//    printf("Starting Motion\n");
//    int pos;        int amp = 50000;
//    for(int i=0; i<10; i++)
//    {
//        pos = amp;
//        res = pDev[0]->MovePosProfile(pos, true);
//        //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
//        if(res == -1)
//        {
//            printf("MovePosProfile Error\n");
//            cout << pDev[0]->GetErrorMsg() << endl;
//            break;
//        }

//        res = pDev[0]->WaitForMotionCompletion(5000);
//        if(res == -1)
//        {
//            printf("WaitForMotion Error\n");
//            break;
//        }

//        pos = -amp;
//        res = pDev[0]->MovePosProfile(pos, true);
//        //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
//        if(res == -1)
//        {
//            printf("MovePosProfile Error\n");
//            cout << pDev[0]->GetErrorMsg() << endl;
//            break;
//        }

//        res = pDev[0]->WaitForMotionCompletion(5000);
//        if(res == -1)
//        {
//            printf("WaitForMotion Error\n");
//            break;
//        }

//    }
//    printf("Oscillation Done\n");
//    sleep(1);
    return 1;

}

int TestSyncPos()
{
    return 1;
//    // ---------------------------------------------------------------
//    // CKim - Enable Cyclic Synch Position Mode
//    for (int i=0; i<numNodes; i++)
//    {
//        res = pDev[i]->SetOperationMode(SYNC_POS);
//        if (res == -1)
//        {
//            printf("Failed to set PosProfile Mode, device %d\n", pDev[i]->GetId());
//            cout << pDev[i]->GetErrorMsg() << endl;
//            return 0;
//        }
//    }

//    struct timespec start, curr;
//    int pos;        int amp = 100000;    double elapsedTime = 0;
//    //printf("Current Position : %d\n", pDev[0]->ReadPosition());
//    int offset = pDev[0]->ReadPosition();

//    printf("Starting Motion\n");
//    clock_gettime(CLOCK_REALTIME,&start);
//    while(elapsedTime < 5.0)
//    {
//        clock_gettime(CLOCK_REALTIME,&curr);
//        elapsedTime = (curr.tv_sec - start.tv_sec);
//        elapsedTime = elapsedTime + (curr.tv_nsec - start.tv_nsec)*1e-9;

//        pos = offset+amp*sin(2.0*3.141592/2.0*elapsedTime);

//        //res = pDev[0]->MovePosProfile(pos, true);
//        res = pDev[0]->MovePosProfileUsingPDO(pos, false);
//        if(res == -1)
//        {
//            printf("MovePosProfile Error\n");
//            cout << pDev[0]->GetErrorMsg() << endl;
//            break;
//        }

//        usleep(750);
//    }
//    printf("Oscillation Done\n");
//    sleep(1);
}

int main()
{
    // CKim - Open CAN port
    res = EposCAN::ConnectCANport("can0");
    if (res != 0)	{
        printf("Failed to open CAN port\n");
        return -1;
    }
    printf("Opened CAN Port\n");

    usleep(1000);

    // CKim - Wake up the devices
    res = EposCAN::SendSYNC();
    if (res != 0)	{
        printf("Failed to send SYNC\n");
        return -1;
    }

    // CKim - Create EposCAN object.
    pDev = new EposCAN();

    // CKim - Set nodeId for all Slaves 0 to NUM_NODE-1


    // CKim - Switch On and Enable Device
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->EnableDevice(i);
        if (res != 0)
        {
            printf("Failed to Enable Device %d\n", i);
            cout << pDev->GetErrorMsg() << endl;
            return -1;
        }
        else {
            printf("Enabled device %d\n",i);
        }
        usleep(1000);
    }

    std::cout << "Press number to continue : ";
    int a;
    std::cin >> a;

    // CKim - Profile Position Test
    //TestProfilePosition();

    // CKim - Profile Velocity Test
//    TestProfileVelocity();

    // CKim - Test PDO
    // CKim - Enable Position Control Mode
    ProfilePosParam PosPara;
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->SetOperationMode(i, PROFILE_POS);
        if (res == -1)
        {
            printf("Failed to set PosProfile Mode, device %d\n", pDev->GetId());
            return -1;
        }

        res = pDev->GetPositionProfileParam(i, PosPara);
        if (res == -1)
        {
            printf("Param Reading Error\n");
            return -1;
        }
        else
        {
            printf("Node %d ProfilePos Params\n",i);
            printf("MaxVel: %d,  QuickDecel: %d,  Acc: %d,  Decel: %d  Following Errr: %d  Vel: %d\n",
                   PosPara.MaxProfileVelocity, PosPara.QuickStopDecel, PosPara.ProfileAccel, PosPara.ProfileDecel,
                   PosPara.MaxFollowingError, PosPara.ProfileVelocity);
        }

        PosPara.ProfileAccel = PosPara.ProfileDecel = 10000;
        PosPara.MaxProfileVelocity = 5000;

        res = pDev->SetPositionProfileParam(i,PosPara);
        if (res == -1)
        {
            printf("Param Setting Error\n");
            return -1;
        }
    }
    pDev->StartPdoExchange();
    //ReceivedData data;
    for (int i=0; i<10; i++)
    {
        //data = pDev->m_Slave[0].data_;

        // CKim - Update RxPDO data
        pDev->m_Slave[0].data_.target_pos = 5000*i;
        pDev->m_Slave[0].data_.control_word = 0x003F;   // Absolute

        // CKim -Read data updated from TxPDO
        printf("Status word 0x%04X\n", pDev->m_Slave[0].data_.status_word);
        std::cout<<"Position : "<<pDev->m_Slave[0].data_.actual_pos << std::endl;
        std::cout<<"Velocity : "<<pDev->m_Slave[0].data_.actual_vel << std::endl;

        sleep(2);
    }
    //sleep(10);
    pDev->StopPdoExchange();

    //TestHoming();
    //TestOscillatingMotion();
    //TestSyncPos();



    // CKim - Disable Device
    for (int i=0; i<NUM_NODE; i++)
    {
        res = pDev->DisableDevice(i);
        if (res != 0)
        {
            printf("Failed to Disable Device %d\n", i);
            return -1;
        }
        else {
            printf("Disabled device %d\n", i);
        }
        usleep(1000);
    }

    EposCAN::DisconnectCANport();
    return 0;

}
