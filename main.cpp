#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include "EposCAN.h"

using namespace std;

int res;
int numNodes = 1;
EposCAN* pDev[1];


sem_t sema;

int TestProfilePosition()
{
    // ---------------------------------------------------------------
    // CKim - Enable Position Control Mode
    ProfilePosParam PosPara;
    for (int i=0; i<numNodes; i++)
    {
        res = pDev[i]->SetOperationMode(PROFILE_POS);
        if (res == -1)
        {
            printf("Failed to set PosProfile Mode, device %d\n", pDev[i]->GetId());
            return -1;
        }

        res = pDev[i]->GetPositionProfileParam(PosPara);
        if (res == -1)
        {
            printf("Param Reading Error\n");
            return -1;
        }
        else
        {
            printf("ProfilePos Params\n");
            printf("MaxVel: %d,  QuickDecel: %d,  Acc: %d,  Decel: %d  Following Errr: %d  Vel: %d\n",
                   PosPara.MaxProfileVelocity, PosPara.QuickStopDecel, PosPara.ProfileAccel, PosPara.ProfileDecel,
                   PosPara.MaxFollowingError, PosPara.ProfileVelocity);
        }

        PosPara.ProfileAccel = PosPara.ProfileDecel = 10000;
        PosPara.MaxProfileVelocity = 5000;

        res = pDev[i]->SetPositionProfileParam(PosPara);
        if (res == -1)
        {
            printf("Param Setting Error\n");
            return -1;
        }

        printf("Control : 0x%04X, Status : 0x%04X\n", pDev[i]->ReadControlWord(), pDev[i]->ReadStatusWord());
        printf("Current Position : %d\n", pDev[i]->ReadPosition());
    }

    sleep(2);

    // CKim - Move to one position
//    struct timespec start, end;         double elapsedTime;     int pos;

//    // CKim - Measure time when sending target position by SDO
//    pos = 500000;
//    clock_gettime(CLOCK_REALTIME,&start);
//    res = pDev[0]->MovePosProfile(pos, true);
//    //res = pDev[0]->MovePosProfile(pos, false);
//    //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
//    if(res == -1)
//    {
//        printf("MovePosProfile Error\n");
//        cout << pDev[0]->GetErrorMsg() << endl;
//    }
//    clock_gettime(CLOCK_REALTIME,&end);
//    elapsedTime = (end.tv_sec - start.tv_sec);
//    elapsedTime = elapsedTime + (end.tv_nsec - start.tv_nsec)*1e-9;
//    cout<< "Time taken is : " << fixed << elapsedTime << endl;

//    sleep(5);

//    //printf("Control : 0x%04X, Status : 0x%04X\n", pDev[0]->ReadControlWord(), pDev[0]->ReadStatusWord());

//    //pDev[0]->HaltAxis();

//    // CKim - Measure time when sending target position by PDO
//    pos = -500000;
//    clock_gettime(CLOCK_REALTIME,&start);
//    res = pDev[0]->MovePosProfile(pos, true);
//    //res = pDev[0]->MovePosProfile(pos, false);
//    //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
//    clock_gettime(CLOCK_REALTIME,&end);
//    elapsedTime = (end.tv_sec - start.tv_sec);
//    elapsedTime = elapsedTime + (end.tv_nsec - start.tv_nsec)*1e-9;
//    cout<< "Time taken is : " << fixed << elapsedTime << endl;

//    sleep(5);

//    if (res == -1)
//    {
//        printf("MovePosProfile error\n");
//        return 0;
//    }

//    sleep(10);

}

int TestProfileVelocity()
{
    // ------------------------------------------------
    // CKim - Setup Profile Velocity Control Mode
    ProfileVelParam VelPara;
    for(int i=0; i<numNodes; i++)
    {
        res = pDev[i]->SetOperationMode(PROFILE_VEL);
        res = pDev[i]->GetVelocityProfileParam(VelPara);
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

        res = pDev[i]->SetVelocityProfileParam(VelPara);
        if (res == -1)
        {
            printf("Param Setting Error\n");
            return -1;
        }
    }

    // CKim - Move one direction
    printf("Moving Forward\n");
    for(int i=0; i<numNodes; i++)
    {
        res = pDev[i]->MoveVelProfile(5000);
        if (res == -1)
        {
            printf("MoveVelProfile error\n");
            return -1;
        }
    }
    sleep(10);

    // CKim - Move the other direction
    printf("Moving Backward\n");
    for(int i=0; i<numNodes; i++)
    {
        res = pDev[i]->MoveVelProfile(-5000);
        if (res == -1)
        {
            printf("MoveVelProfile error\n");
            return -1;
        }
    }
    sleep(10);

    // CKim - Stop
    printf("Stopping\n");
    for(int i=0; i<numNodes; i++)
    {
        res = pDev[i]->HaltAxis();
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
    // ---------------------------------------------------------------
    // CKim - Enable Position Control Mode
    ProfilePosParam PosPara;
    for (int i=0; i<numNodes; i++)
    {
        res = pDev[i]->SetOperationMode(PROFILE_POS);
        if (res == -1)
        {
            printf("Failed to set PosProfile Mode, device %d\n", pDev[i]->GetId());
            cout << pDev[i]->GetErrorMsg() << endl;
            return 0;
        }

        res = pDev[i]->GetPositionProfileParam(PosPara);
        if (res == -1)
        {
            printf("Param Reading Error\n");
            cout << pDev[i]->GetErrorMsg() << endl;
            return 0;
        }
        else
        {
            printf("ProfilePos Params\n");
            printf("MaxVel: %d,  QuickDecel: %d,  Acc: %d,  Decel: %d  Following Errr: %d  Vel: %d\n",
                   PosPara.MaxProfileVelocity, PosPara.QuickStopDecel, PosPara.ProfileAccel, PosPara.ProfileDecel,
                   PosPara.MaxFollowingError, PosPara.ProfileVelocity);
        }

        PosPara.ProfileAccel = PosPara.ProfileDecel = 10000;
        PosPara.MaxProfileVelocity = 50000;
        PosPara.ProfileVelocity = 50000;

        res = pDev[i]->SetPositionProfileParam(PosPara);
        if (res == -1)
        {
            printf("Param Setting Error\n");
            cout << pDev[i]->GetErrorMsg() << endl;
            return 0;
        }
    }


    printf("Starting Motion\n");
    int pos;        int amp = 50000;
    for(int i=0; i<10; i++)
    {
        pos = amp;
        res = pDev[0]->MovePosProfile(pos, true);
        //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
        if(res == -1)
        {
            printf("MovePosProfile Error\n");
            cout << pDev[0]->GetErrorMsg() << endl;
            break;
        }

        res = pDev[0]->WaitForMotionCompletion(5000);
        if(res == -1)
        {
            printf("WaitForMotion Error\n");
            break;
        }

        pos = -amp;
        res = pDev[0]->MovePosProfile(pos, true);
        //res = pDev[0]->MovePosProfileUsingPDO(pos, true);
        if(res == -1)
        {
            printf("MovePosProfile Error\n");
            cout << pDev[0]->GetErrorMsg() << endl;
            break;
        }

        res = pDev[0]->WaitForMotionCompletion(5000);
        if(res == -1)
        {
            printf("WaitForMotion Error\n");
            break;
        }

    }
    printf("Oscillation Done\n");
    sleep(1);


}

int TestSyncPos()
{
    // ---------------------------------------------------------------
    // CKim - Enable Cyclic Synch Position Mode
    for (int i=0; i<numNodes; i++)
    {
        res = pDev[i]->SetOperationMode(SYNC_POS);
        if (res == -1)
        {
            printf("Failed to set PosProfile Mode, device %d\n", pDev[i]->GetId());
            cout << pDev[i]->GetErrorMsg() << endl;
            return 0;
        }
    }

    struct timespec start, curr;
    int pos;        int amp = 100000;    double elapsedTime = 0;
    //printf("Current Position : %d\n", pDev[0]->ReadPosition());
    int offset = pDev[0]->ReadPosition();

    printf("Starting Motion\n");
    clock_gettime(CLOCK_REALTIME,&start);
    while(elapsedTime < 5.0)
    {
        clock_gettime(CLOCK_REALTIME,&curr);
        elapsedTime = (curr.tv_sec - start.tv_sec);
        elapsedTime = elapsedTime + (curr.tv_nsec - start.tv_nsec)*1e-9;

        pos = offset+amp*sin(2.0*3.141592/2.0*elapsedTime);

        //res = pDev[0]->MovePosProfile(pos, true);
        res = pDev[0]->MovePosProfileUsingPDO(pos, false);
        if(res == -1)
        {
            printf("MovePosProfile Error\n");
            cout << pDev[0]->GetErrorMsg() << endl;
            break;
        }

        usleep(750);
    }
    printf("Oscillation Done\n");
    sleep(1);
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

    // CKim - Create EPOS object. Index is base 1.
    for (int i=0; i<numNodes; i++)
    {
        pDev[i] = new EposCAN(i + 1);
    }

    // CKim - Switch On and Enable Device
    for (int i=0; i<numNodes; i++)
    {
        res = pDev[i]->EnableDevice();
        if (res != 0)
        {
            printf("Failed to Enable Device %d\n", pDev[i]->GetId());
            cout << pDev[i]->GetErrorMsg() << endl;
            return -1;
        }
        else {
            printf("Enabled device %d\n",pDev[i]->GetId());
        }
        usleep(1000);
    }

    std::cout << "Press number to continue : ";
    int a;
    std::cin >> a;

//    // -----------------------------------------------
//    // CKim - Configure TxPDO1 and RxPDO1
//    for (int i=0; i<numNodes; i++)
//    {
//        res = pDev[i]->DisablePDO();        if (res == -1)	{  break;    }
//        res = pDev[i]->ConfigureTxPDO();    if (res == -1)	{  break;    }
//        res = pDev[i]->ConfigureRxPDO();    if (res == -1)	{  break;    }
//        //res = pDev[i]->SetTxPDOMapping(1);		if (res == -1)	{ break; }
//        //res = pDev[i]->SetRxPDOMapping(1);		if (res == -1)	{ break; }
//        res = pDev[i]->EnablePDO();         if (res == -1)	{  break;    }
//    }
//    if (res == -1)
//    {
//        printf("Error while configuring PDO\n");
//        cout << pDev[0]->GetErrorMsg() << endl;
//        return 0;
//    }
//    // -----------------------------------------------

    //sleep(2);

    TestProfilePosition();
    //TestProfileVelocity();

    //TestHoming();
    //TestOscillatingMotion();
    //TestSyncPos();



    // CKim - Disable Device
    for (int i = 0; i < numNodes; i++)
    {
        res = pDev[i]->DisableDevice();
        if (res != 0)
        {
            printf("Failed to Disable Device %d\n", pDev[i]->GetId());
            return -1;
        }
        else {
            printf("Disabled device %d\n",pDev[i]->GetId());
        }
        usleep(1000);
    }

    EposCAN::DisconnectCANport();
    return 0;

}
