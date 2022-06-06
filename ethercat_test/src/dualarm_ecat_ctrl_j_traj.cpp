
//
// Created by mservo lab, Jeong
// Modified with Wonseok Jeon on 20. 11. 11...

// This is for Dual-arm mobile platform
// homepos for ver2 changed on 21. 01. 29

// Updated 22. 05. 29 --> only motioncallback(ver1) modified

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <memory.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <xenomai/init.h>

#include "ethercat_test/pos.h"
#include "ethercat_test/vel.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "soem/ethercat.h"
#include "pdo_def.h"
#include "servo_def.h"
#include "ecat_dc.h"

#define EC_TIMEOUTMON 500
#define NUMOFMANI_DRIVE     14
#define NUMOFWHEEL_DRIVE    0
#define NUMOFEPOS4_DRIVE	NUMOFMANI_DRIVE+NUMOFWHEEL_DRIVE
#define NSEC_PER_SEC 1000000000

int step_size = 2;
unsigned int cycle_ns = step_size*1000000;
double sps = 1000.0/step_size; // step per sec


EPOS4_Drive_pt	epos4_drive_pt[NUMOFEPOS4_DRIVE];
int started[NUMOFEPOS4_DRIVE]={0}, ServoState=0, TargetState=0;
uint8 servo_ready = 0, servo_prestate = 0;

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
int wkc;
boolean inOP;
uint8 currentgroup = 0;

RT_TASK motion_task;
RT_TASK pub_task;

RTIME now, previous;

double tr[14] = {0};
long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "eno1";
int run = 1;
int sys_ready = 0;
boolean limit_flag = FALSE;

int wait = 0;
int control1 = 0, control2 = 0;
int recv_fail_cnt = 0;
double gt1 = 0, gt2=0;

//for ver1 i = 0~6, for ver2 i = 7~13

double ap[14] = {0},bp[14] = {0},cp[14] ={0},dp[14] = {0};
int32_t zeropos[14] = {0}; // initial pos
int32_t targetpos[14] = {0};
int32_t homepos[14] = {0,0,0,0,0,0,0,-63715, 38594, 37694, -20069, 85386, -12212, -72300};
// for ver1 : 0, for ver2 : each home position {-63715, 38594, 37694, -20069, 85386, -12212, -72300};
double zerovel[14] = {0};  // initial axis vel
int32_t actualvel[14] = {0}; // initial motor vel

double acc[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // axis rpm/s

int32_t wheeldes[4] = {0};

int gear_ratio[] = {160, 160, 160, 120, 100, 100, 100,
                    120, 120, 120, 120, 100, 100, 100};
//ver 1
int pulse_rev[] = {4096, 4096, 4096, 4096, 2048, 2048, 2048};
//ver 2
int encoder_bit[] = {18, 18, 18, 18, 18, 18, 18};

double rad2inc[14] = {0}, rpm2ips[14] = {0}, rpms2ipss[14] = {0};
int resol[14] = {0};

int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;


void resol_conv(){
    int i = 0;
    for (i=0;i<7;i++){
        resol[i] = 4*gear_ratio[i]*pulse_rev[i];  // axis rot to motor inc
        rad2inc[i] = resol[i]/2.0/M_PI; // axis 1 rad to motor inc
        rpm2ips[i] = resol[i]/sps/60.0; // axis rpm to motor inc per step (ms)
        rpms2ipss[i] = resol[i]/60.0/sps/sps; // rpm/sec to inc/step/step
//        printf("%i\n",resol[i]);
    }
    for (i=7;i<14;i++){
        resol[i] = pow(2,encoder_bit[i-7]);  // axis rot to motor inc
        rad2inc[i] = resol[i]/2.0/M_PI; // axis 1 rad to motor inc
        rpm2ips[i] = resol[i]/sps/60.0; // axis rpm to motor inc per step (ms)
        rpms2ipss[i] = resol[i]/60.0/sps/sps; // rpm/sec to inc/step/step
 //       printf("%i\n",resol[i]);
    }
}

boolean ecat_init(void)
{
    int i, oloop, iloop, chk, wkc_count;
    needlf = FALSE;
    inOP = FALSE;

    rt_printf("Starting DC test\n");
    if (ec_init(ecat_ifname))
    {
        rt_printf("ec_init on %s succeeded. \n", ecat_ifname);

        /* find and auto-config slaves in network */
        if (ec_config_init(FALSE) > 0)
        {
            rt_printf("%d slaves found and configured.\n", ec_slavecount);

            for (int k=1; k<15; ++k)
            {
                if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k+1].name,"EPOS4") == 0)) //change name for other drives
                {
                    rt_printf("Re mapping for EPOS4 %d...\n", k);

                    os=sizeof(ob3); ob3 = 0x00;	//RxPDO, check MAXPOS ESI
                    wkc_count=ec_SDOread(k+1, 0x2000, 0x00, FALSE, &os, &ob3, EC_TIMEOUTRXM);

                    rt_printf("NodeID %d...\n", ob3);

                    os=sizeof(ob); ob = 0x00;	//RxPDO, check MAXPOS ESI
                    //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
                    wkc_count=ec_SDOwrite(k+1, 0x1c12, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                    wkc_count=ec_SDOwrite(k+1, 0x1c13, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    // 1. StatusWord UINT16
                    os=sizeof(ob); ob = 0x60410010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 2. PositionActualValue INT32
                    os=sizeof(ob); ob = 0x60640020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 3. VelocityActualValue INT32
                    os=sizeof(ob); ob = 0x606C0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 4. DigitalInput UINT32
                    os=sizeof(ob); ob = 0x60FD0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x04, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 5. ErrorCode UINT16
                    os=sizeof(ob); ob = 0x603F0010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x05, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x05;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    if (wkc_count==0)
                    {
                        rt_printf("TxPDO assignment error\n");
                        //return FALSE;
                    }

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A01, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A02, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A03, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    // 1. ControlWorl UINT16
                    os=sizeof(ob); ob = 0x60400010;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 2. TargetPosition INT32
                    os=sizeof(ob); ob = 0x607A0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 3. TargetVelocity INT32
                    os=sizeof(ob); ob = 0x60FF0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);


                    os=sizeof(ob3); ob3 = 0x03;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    if (wkc_count==0)
                    {
                        rt_printf("RxPDO assignment error\n");
                        //return FALSE;
                    }


                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1601, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1602, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1603, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1600;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1A00;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    // Interpolation time period --> 2ms (same as control period)
                    os=sizeof(ob3); ob3 = step_size;
                    wkc_count=ec_SDOwrite(k+1, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    // Modes of operation, CSP : 0x08, CSV : 0x09
                    os=sizeof(ob3); ob3 = 0x08;
                    wkc_count=ec_SDOwrite(k+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    // Following error window for position
                    os=sizeof(ob); ob = 0x0186A0; // Following error window
                    wkc_count=ec_SDOwrite(k+1, 0x6065, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    //os=sizeof(ob); ob = -0; // Software position limit min
                    //wkc_count=ec_SDOwrite(k, 0x607D, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);

                    //os=sizeof(ob); ob = 0; // Software position limit max
                    //wkc_count=ec_SDOwrite(k, 0x607D, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM); 17.097000000]: Waiting for dualarm/manipulator1_controller/follow_joint_trajectory to come up

                    os=sizeof(ob3); ob3 = 22; // Digital input configuration
                    if (k<8){
                        wkc_count=ec_SDOwrite(k+1, 0x3142, 0x04, FALSE, os, &ob3, EC_TIMEOUTRXM); // for ver1
                    }
                    else{
                        wkc_count=ec_SDOwrite(k+1, 0x3142, 0x02, FALSE, os, &ob3, EC_TIMEOUTRXM); // for ver2
                    }
                    os=sizeof(ob3); ob3 = 20; // Digital input configuration, quick stop
                    wkc_count=ec_SDOwrite(k+1, 0x3142, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);
                }
            }
            for (int k=15; k<19; ++k)
            {
                if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k+1].name,"EPOS4") == 0)) //change name for other drives
                {
                    rt_printf("Re mapping for EPOS4 %d...\n", k);

                    os=sizeof(ob3); ob3 = 0x00;	//RxPDO, check MAXPOS ESI
                    wkc_count=ec_SDOread(k+1, 0x2000, 0x00, FALSE, &os, &ob3, EC_TIMEOUTRXM);

                    rt_printf("NodeID %d...\n", ob3);

                    os=sizeof(ob); ob = 0x00;	//RxPDO, check MAXPOS ESI
                    //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
                    wkc_count=ec_SDOwrite(k+1, 0x1c12, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                    wkc_count=ec_SDOwrite(k+1, 0x1c13, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    // 1. StatusWord UINT16
                    os=sizeof(ob); ob = 0x60410010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 2. PositionActualValue INT32
                    os=sizeof(ob); ob = 0x60640020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 3. VelocityActualValue INT32
                    os=sizeof(ob); ob = 0x606C0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 4. DigitalInput UINT32
                    os=sizeof(ob); ob = 0x60FD0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x04, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 5. ErrorCode UINT16
                    os=sizeof(ob); ob = 0x603F0010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x05, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x05;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    if (wkc_count==0)
                    {
                        rt_printf("TxPDO assignment error\n");
                        //return FALSE;
                    }

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A01, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A02, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A03, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    // 1. ControlWorl UINT16
                    os=sizeof(ob); ob = 0x60400010;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 2. TargetPosition INT32
                    os=sizeof(ob); ob = 0x607A0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 3. TargetVelocity INT32
                    os=sizeof(ob); ob = 0x60FF0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);


                    os=sizeof(ob3); ob3 = 0x03;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    if (wkc_count==0)
                    {
                        rt_printf("RxPDO assignment error\n");
                        //return FALSE;
                    }


                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1601, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1602, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1603, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1600;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1A00;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    // Interpolation time period --> 2ms (same as control period)
                    os=sizeof(ob3); ob3 = step_size;
                    wkc_count=ec_SDOwrite(k+1, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    // Modes of operation, CSP : 0x08, CSV : 0x09
                    os=sizeof(ob3); ob3 = 0x09;
                    wkc_count=ec_SDOwrite(k+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                }
            }

            ec_config_map(&IOmap);
            /* Configurate distributed clock */
            ec_configdc();
            rt_printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /* configure DC options for every DC capable slave found in the list */
            rt_printf("DC capable : %d\n", ec_configdc());

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

            rt_printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            rt_printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            rt_printf("Caculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* To enter state OP we need to send valid date to outpus */
            /* send one valid process data to make outputs in slaves happy */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            chk = 200;
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                rt_printf("Operational state reached for all slaves.\n");
                for (int k=0; k<NUMOFEPOS4_DRIVE; ++k)
                {
                    epos4_drive_pt[k].ptOutParam=(EPOS4_DRIVE_RxPDO_t*)  ec_slave[k+2].outputs;
                    epos4_drive_pt[k].ptInParam= (EPOS4_DRIVE_TxPDO_t*)  ec_slave[k+2].inputs;
                }
                inOP = TRUE;
            }
            else
            {
                rt_printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i=0; i<(ec_slavecount); i++)
                {
                    if (ec_slave[i+2].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State 0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i+2].state, ec_slave[i+2].ALstatuscode, ec_ALstatuscode2string(ec_slave[i+2].ALstatuscode));
                    }
                }
                for (i=0; i<NUMOFEPOS4_DRIVE; i++)
                    ec_dcsync0(i+2, FALSE, 0, 0);
            }
        }
        else
        {
            rt_printf("No slaves found!\n");
            inOP = FALSE;
        }
    }
    else
    {
        rt_printf("No socket connection on %s\nExecute as root\n", ecat_ifname);
        return FALSE;
    }
    return inOP;
}

void EPOS_OP(void *arg)
{
    unsigned long ready_cnt = 0;
    uint16_t controlword=0;
    int i, f;

    if (ecat_init()==FALSE)
    {
        run = 0;
        printf("EPOS INIT FAIL");
        return;
    }
    rt_task_sleep(1e6);

    /* Distributed clock set up */
    long long toff;
    long long cur_DCtime = 0, max_DCtime = 0;
    unsigned long long cur_dc32 = 0, pre_dc32 = 0;
    int32_t shift_time = 380000;
    long long diff_dc32;

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+2, TRUE, cycle_ns, 0);

    RTIME cycletime = cycle_ns;
    RTIME cur_time = 0; // current master time
    RTIME cur_cycle_cnt = 0; // number of cycles has been passed
    RTIME cycle_time;  // cur_cycle_cnt*cycle_ns
    RTIME remain_time; // cur_time%cycle_ns
    RTIME dc_remain_time; // remain time to next cycle of REF clock, cur_dc32 % cycletime
    RTIME rt_ts; // updated master time to REF clock
    toff = 0;
    RTIME pr_t, n_t;

    ec_send_processdata();
    cur_time = rt_timer_read();
    cur_cycle_cnt = cur_time/cycle_ns;
    cycle_time = cur_cycle_cnt*cycle_ns;
    remain_time = cur_time%cycle_ns;

    rt_printf("cycles have been passed : %lld\n", cur_cycle_cnt);
    rt_printf("remain time to next cycle : %lld\n", remain_time);

    wkc = ec_receive_processdata(EC_TIMEOUTRET); // get reference DC time
    cur_dc32 = (uint32_t)(ec_DCtime & 0xFFFFFFFF); // consider only lower 32-bit, because epos has 32-bit processor
    dc_remain_time = cur_dc32%cycletime;
    rt_ts = cycle_time + dc_remain_time; // update master time to REF clock

    rt_printf("DC remain time to next cycle : %lld\n", dc_remain_time);

    rt_task_sleep_until(rt_ts); // wait until next REF clock

    while (run)
    {

        // wait for next cycle
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);

        previous = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3*NUMOFEPOS4_DRIVE)
            recv_fail_cnt++;
        now = rt_timer_read();
        ethercat_time = (long) (now-previous);

        cur_dc32 = (uint32_t) (ec_DCtime & 0xFFFFFFFF);
        if (cur_dc32>pre_dc32)
            diff_dc32 = cur_dc32-pre_dc32;
        else
            diff_dc32 = (0xFFFFFFFF - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;

        //servo-on
        for (i=0; i<14; ++i)
        {
            if (limit_flag&&sys_ready==0)
                epos4_drive_pt[i].ptOutParam->ControlWord=2;
            else {
                controlword = 0;
                started[i] = ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);
                epos4_drive_pt[i].ptOutParam->ControlWord = controlword;
                if (started[i]) ServoState |= (1 << i);
                if (ready_cnt < 10) {
                    zeropos[i] = epos4_drive_pt[i].ptInParam->PositionActualValue;
                    targetpos[i] = zeropos[i];

                    epos4_drive_pt[i].ptOutParam->TargetPosition = zeropos[i];

                }
            }

        }
        for (i=14; i<NUMOFEPOS4_DRIVE; ++i)
        {
            controlword=0;
            started[i]=ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);
            epos4_drive_pt[i].ptOutParam->ControlWord=controlword;
            if (started[i]) ServoState |= (1<<i); //started[i] is same as enable
        }


        if (ServoState == (1<<NUMOFEPOS4_DRIVE)-1) //all servos are in ON state
        {
            if (servo_ready==0) {
                servo_ready = 1;
            }
        }

        if (servo_ready) ready_cnt++;
        if (ready_cnt>=1000) //wait for 1s after servo-on
        {
            ready_cnt=10000;
            sys_ready=1;
        }

        if (sys_ready)
        {   
            pr_t = rt_timer_read();

            for (i=0; i<7; i++)
            {
                if (epos4_drive_pt[i].ptInParam->DigitalInput != 0x00000000)
                {
                    limit_flag = TRUE;
                    zeropos[i]=epos4_drive_pt[i].ptInParam->PositionActualValue;
                    if (wait ==0)
                        rt_printf("joint #%i is limit position, %i\n", i+1,zeropos[i]);
                    f = i;

                }
                else
                {
                    
			tr[i] = ap[i]*gt1*gt1*gt1 + bp[i]*gt1*gt1 + cp[i]*gt1 + dp[i];
//			        tr[i] = tr[i] * resol[i]/2/M_PI;
			if (control1 ==1) targetpos[i] = int (tr[i])+homepos[i];
			if (i == 3){
//	                rt_printf("%i %f %f %f %f\n",i, ap[i],bp[i],cp[i],dp[i]);
			        rt_printf("target = %d\n",targetpos[i]);}
//                        epos4_drive_pt[i].ptOutParam->TargetPosition = targetpos[i];

//		    if (i==3) rt_printf("Desired: %d /Traj: %d / Actual: %d / %i\n",targetpos[i],p_des,epos4_drive_pt[i].ptInParam->PositionActualValue,w);
                }
            }

            for (i=7; i<14; i++)
            {
                if (epos4_drive_pt[i].ptInParam->DigitalInput != 0x00000000)
                {
                    limit_flag = TRUE;
                    zeropos[i]=epos4_drive_pt[i].ptInParam->PositionActualValue;
                    if (wait ==0)
                        rt_printf("joint #%i is limit position, %i\n", i+1,zeropos[i]);
                    f = i;

                }
                else
                {
                    tr[i] = ap[i]*gt2*gt2*gt2 + bp[i]*gt2*gt2 + cp[i]*gt2 + dp[i];
//			        tr[i] = tr[i] * resol[i]/2/M_PI;
		    if (control2 == 1) targetpos[i] = int (tr[i])+homepos[i];
			        if (i == 10){
//	                rt_printf("%i %f %f %f %f\n",i, ap[i],bp[i],cp[i],dp[i]);
			        rt_printf("target = %d\n",targetpos[i]);}
//                  
//                   epos4_drive_pt[i].ptOutParam->TargetPosition = targetpos[i];

                }
            }


            for (i=14;i<NUMOFEPOS4_DRIVE; ++i)
            {
                epos4_drive_pt[i].ptOutParam->TargetVelocity=wheeldes[i-NUMOFMANI_DRIVE];
                if(epos4_drive_pt[i].ptInParam->ErrorCode != 0)
                    rt_printf("Error : 0x%x %d\n",epos4_drive_pt[i].ptInParam->ErrorCode, rt_timer_read());
            }
	        n_t = rt_timer_read();

    	    if (control1 == 1){
                gt1 += double (n_t-pr_t)/NSEC_PER_SEC*500;
            }
            if (control2 == 1){
                gt2 += double (n_t-pr_t)/NSEC_PER_SEC*500;
            }

        } // sysready

        else
        {
            for (i=NUMOFMANI_DRIVE;i<NUMOFEPOS4_DRIVE; ++i)
            {
                epos4_drive_pt[i].ptOutParam->TargetVelocity=0;
            }
        }


        if (sys_ready)
            if (worst_time<ethercat_time) worst_time=ethercat_time;

        if (limit_flag)
        {
            zeropos[f]=epos4_drive_pt[f].ptInParam->PositionActualValue;

            if (acc[f] <0){
                targetpos[f] = zeropos[f] + 2;
            }
            else{
                targetpos[f] = zeropos[f] - 2;
            }
            epos4_drive_pt[f].ptOutParam->TargetPosition=targetpos[f];

            wait += 1;
            if (wait==0) {
                rt_printf("joint #%i is limit position, \n", f);
            }
            if (wait == step_size*1000)
                run = 0;
        }
    }

    rt_task_sleep(cycle_ns);

    for (i=0; i<NUMOFEPOS4_DRIVE; i++)
        ec_dcsync0(i+2, FALSE, 0, 0); // SYNC0,1 on slave 1

    //Servo OFF
    for (i=0; i<NUMOFEPOS4_DRIVE; i++)
    {
        epos4_drive_pt[i].ptOutParam->ControlWord=2; // quick stop
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    rt_printf("End EPOS Dual arm control, close socket\n");
    /* stop SOEM, close socket */
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

    ec_close();

}

void pub_run(void *arg)
{

    int i, print_stop = 1;
    unsigned long itime = 0;
    long stick = 0;
    int argc;
    char** argv;
    ethercat_test::pos p_msg;
    ethercat_test::pos p_msg2;
    ethercat_test::vel v_msg;

    ros::init(argc, argv, "dualarm_pub");

    ros::NodeHandle n;

    ros::Publisher pos_pub1 = n.advertise<ethercat_test::pos>("motor_pos_1",1);
    ros::Publisher pos_pub2 = n.advertise<ethercat_test::pos>("motor_pos_2",1);
    ros::Publisher vel_pub = n.advertise<ethercat_test::vel>("measure",1);


    rt_task_set_periodic(NULL, TM_NOW, 5e6);

    while (run)
    {
        rt_task_wait_period(NULL);
        if (inOP==TRUE)
        {
            if (!sys_ready)
            {
                if(stick==0)
                    rt_printf("waiting for system ready...\n");
                if(stick%10==0)
                    rt_printf("%i \n", stick/10);
                stick++;
            }
            else
            {
                if (limit_flag)
                {
                    if (print_stop<500) {
                        i =11;
                        rt_printf("#%i, Actual/Target = %i / %i/ %i\n", i,
                                      epos4_drive_pt[i].ptInParam->PositionActualValue,
                                      epos4_drive_pt[i].ptOutParam->TargetPosition, epos4_drive_pt[i].ptInParam->PositionActualValue-epos4_drive_pt[i].ptOutParam->TargetPosition);

                        print_stop+=1;
                    }
                }
                else {
                    itime++;
//                    rt_printf("\nTime = %06d.%01d, \e[32;1m fail=%ld\e[0m, ecat_T=%ld, maxT=%ld\n",
//                              itime / 10, itime % 10, recv_fail_cnt, ethercat_time / 1000, worst_time / 1000);
                    for (i = 0; i < 14; i++) {

//                        rt_printf("EPOS4_DRIVE #%i\n", i + 1);
                        //rt_printf("Statusword = 0x%x\n", epos4_drive_pt[i].ptInParam->StatusWord);
//                        rt_printf("vel = %i\n", epos4_drive_pt[i].ptInParam->VelocityActualValue);
//                        rt_printf("Actual/Target = %i / %i\n", epos4_drive_pt[i].ptInParam->PositionActualValue,
//                                  epos4_drive_pt[i].ptOutParam->TargetPosition);
//                        rt_printf("Following error = %i\n" , epos4_drive_pt[i].ptInParam->PositionActualValue-epos4_drive_pt[i].ptOutParam->TargetPosition);
                        if (i < 7) {
//                            p_msg.position[i] = epos4_drive_pt[i].ptInParam->PositionActualValue;
			                p_msg.position[i] = targetpos[i];
                        } else {
//			    rt_printf("%d",epos4_drive_pt[i].ptInParam->PositionActualValue);
                            // p_msg2.position[i - 7] = epos4_drive_pt[i].ptInParam->PositionActualValue;
                            p_msg2.position[i-7] = targetpos[i];
                                                
                        }
                        //rt_printf("\n");
                    }
                    for (i = 14; i < NUMOFEPOS4_DRIVE; i++) {
                        rt_printf("Wheel #%i\n", i - 13);
                        rt_printf("Statusword = 0x%x\n", epos4_drive_pt[i].ptInParam->StatusWord);
                        rt_printf("Actual/Target = %i / %i\n", epos4_drive_pt[i].ptInParam->VelocityActualValue,
                                  epos4_drive_pt[i].ptOutParam->TargetVelocity);
////                    rt_printf("Following error = %i\n" , epos4_drive_pt[i].ptInParam->PositionActualValue-epos4_drive_pt[i].ptOutParam->TargetPosition);t_printf("EPOS4_DRIVE #%i\n", i+1);
////
                        v_msg.velocity[i - 14] = epos4_drive_pt[i].ptInParam->VelocityActualValue;
                    }
//
                    pos_pub1.publish(p_msg);
                    pos_pub2.publish(p_msg2);
                    vel_pub.publish(v_msg);

                }


            }
        }
    }
}


void motion_callback1(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{

    std::vector<trajectory_msgs::JointTrajectoryPoint>::size_type traj = msg->goal.trajectory.points.size();

    double pos_desired_rad[7] = {0}, vel_desired_rad[7] = {0}, pos_desired_rad_[7] = {0}, vel_desired_rad_[7] = {0};
    int pos_desired[7] = {0}, vel_desired[7] ={0}, pos_desired_[7] = {0}, vel_desired_[7] = {0};
    double t =0, t_ = 0;
    int ave =1000;
    uint64_t sec_ = 0, nsec_ = 0, sec = 0, nsec = 0;

    control1 = 1;

    for (int j=1;j<traj;++j){
//        rt_printf("%i, \n" ,j);
	    sec = msg->goal.trajectory.points[j].time_from_start.sec; //***
	    sec = sec*NSEC_PER_SEC;
    	nsec = msg->goal.trajectory.points[j].time_from_start.nsec; //***
	
	    rt_printf("j = %i %llu %d\n",j, sec,nsec);
	    t_ = 0;
	    t = double ((sec+nsec)-(sec_+nsec_))/NSEC_PER_SEC*500;
	    rt_printf("%f\n",t);
        gt1 = 0;

        for (int i=0; i<7; i++)
        {
//            pos_desired_rad[j-1][i] = msg->goal.trajectory.points[j].positions[i];
	        pos_desired_rad_[i] = msg->goal.trajectory.points[j-1].positions[i];	    
	        pos_desired_rad[i] = msg->goal.trajectory.points[j].positions[i];

	        vel_desired_rad_[i] = msg->goal.trajectory.points[j-1].velocities[i];
	        vel_desired_rad[i] = msg->goal.trajectory.points[j].velocities[i];            
            rt_printf("%i, pos = %f,\n" ,i, pos_desired_rad[i]);

	        if(i==1||i==2||i==4||i==6){
			    pos_desired_rad_[i] *= -1;
			    pos_desired_rad[i] *= -1;
			    vel_desired_rad_[i] *= -1;
			    vel_desired_rad[i] *= -1;
	        }


            if (pos_desired_rad[i] >= pos_desired_rad_[i]){
                acc[i] = 1;
            }
            else{
                acc[i] = -1;
            }

	        pos_desired[i] = pos_desired_rad[i]*resol[i]/2/M_PI;
	        pos_desired_[i] = pos_desired_rad_[i]*resol[i]/2/M_PI;
 	        vel_desired[i] = vel_desired_rad[i]*resol[i]/2/M_PI;
 	        vel_desired_[i] = vel_desired_rad_[i]*resol[i]/2/M_PI;

       	    ap[i] = (vel_desired_[i]+vel_desired[i]-2*(pos_desired_[i]-pos_desired[i])/(t_ - t))/(t_-t)/(t_-t);
	        bp[i] = (vel_desired_[i]-vel_desired[i]-3*ap[i]*(t_*t_-t*t))/2/(t_-t);
	        cp[i] = vel_desired_[i]-3*ap[i]*t_*t_ -2*bp[i]*t_;
	        dp[i] = pos_desired_[i]-ap[i]*t_*t_*t_-bp[i]*t_*t_-cp[i]*t_;

//	        ap[i] = (vel_desired_rad_[i]+vel_desired_rad[i]-2*(pos_desired_rad_[i]-pos_desired_rad[i])/(t_ - t))/(t_-t)/(t_-t);
//	        bp[i] = (vel_desired_rad_[i]-vel_desired_rad[i]-3*ap[i]*(t_*t_-t*t))/2/(t_-t);
//	        cp[i] = vel_desired_rad_[i]-3*ap[i]*t_*t_ -2*bp[i]*t_;
//	        dp[i] = pos_desired_rad_[i]-ap[i]*t_*t_*t_-bp[i]*t_*t_-cp[i]*t_;
					
            rt_printf("%i %f %f %f %f\n",i, ap[i],bp[i],cp[i],dp[i]);
        }

        rt_task_sleep(sec+nsec-sec_-nsec_-2000);//***
	    sec_ = sec;
	    nsec_ = nsec;
    }

    while (ave > 500){
        for (int i=0; i<7; i++){
            ave += abs(tr[i]-pos_desired[i]);
        }
        ave = ave/7;
    }
        
    control1 = 0;
}

void motion_callback2(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{

    std::vector<trajectory_msgs::JointTrajectoryPoint>::size_type traj = msg->goal.trajectory.points.size();

    double pos_desired_rad[7] = {0}, vel_desired_rad[7] = {0}, pos_desired_rad_[7] = {0}, vel_desired_rad_[7] = {0};
    int pos_desired[7] = {0}, vel_desired[7] ={0}, pos_desired_[7] = {0}, vel_desired_[7] = {0};
    double t =0, t_ = 0;
    int ave = 1000;
    uint64_t sec_ = 0, nsec_ = 0, sec = 0, nsec = 0;

    control2 = 1;

    for (int j=1;j<traj;++j){
//        rt_printf("%i, \n" ,j);
	    sec = msg->goal.trajectory.points[j].time_from_start.sec; //***
	    sec = sec*NSEC_PER_SEC;
	    nsec = msg->goal.trajectory.points[j].time_from_start.nsec; //***
	
	    rt_printf("j = %i %llu %d\n",j, sec,nsec);
	    t_ = 0;
	    t = double ((sec+nsec)-(sec_+nsec_))/NSEC_PER_SEC*500;
	    rt_printf("%f\n",t);
        gt2 = 0;

        for (int i=0; i<7; i++)
        {
	        pos_desired_rad_[i] = msg->goal.trajectory.points[j-1].positions[i];	    
	        pos_desired_rad[i] = msg->goal.trajectory.points[j].positions[i];

	        vel_desired_rad_[i] = msg->goal.trajectory.points[j-1].velocities[i];
	        vel_desired_rad[i] = msg->goal.trajectory.points[j].velocities[i];            
            rt_printf("%i, pos = %f,\n" ,i, pos_desired_rad[i]);

	        if(i!=5){
			    pos_desired_rad_[i] *= -1;
			    pos_desired_rad[i] *= -1;
			    vel_desired_rad_[i] *= -1;
			    vel_desired_rad[i] *= -1;
            }


            if (pos_desired_rad[i] >= pos_desired_rad_[i]){
                acc[i+7] = 1;
            }
            else{
                acc[i+7] = -1;
            }


	        pos_desired[i] = pos_desired_rad[i]*resol[i+7]/2/M_PI;
	        pos_desired_[i] = pos_desired_rad_[i]*resol[i+7]/2/M_PI;
 	        vel_desired[i] = vel_desired_rad[i]*resol[i+7]/2/M_PI;
 	        vel_desired_[i] = vel_desired_rad_[i]*resol[i+7]/2/M_PI;

	        ap[i+7] = (vel_desired_[i]+vel_desired[i]-2*(pos_desired_[i]-pos_desired[i])/(t_ - t))/(t_-t)/(t_-t);
	        bp[i+7] = (vel_desired_[i]-vel_desired[i]-3*ap[i+7]*(t_*t_-t*t))/2/(t_-t);
	        cp[i+7] = vel_desired_[i]-3*ap[i+7]*t_*t_ -2*bp[i+7]*t_;
	        dp[i+7] = pos_desired_[i]-ap[i+7]*t_*t_*t_-bp[i+7]*t_*t_-cp[i+7]*t_;


//	        ap[i+7] = (vel_desired_rad_[i]+vel_desired_rad[i]-2*(pos_desired_rad_[i]-pos_desired_rad[i])/(t_ - t))/(t_-t)/(t_-t);
//	        bp[i+7] = (vel_desired_rad_[i]-vel_desired_rad[i]-3*ap[i+7]*(t_*t_-t*t))/2/(t_-t);
//	        cp[i+7] = vel_desired_rad_[i]-3*ap[i+7]*t_*t_ -2*bp[i+7]*t_;
//	        dp[i+7] = pos_desired_rad_[i]-ap[i+7]*t_*t_*t_-bp[i+7]*t_*t_-cp[i+7]*t_;
					
            rt_printf("%i %f %f %f %f\n",i, ap[i+7],bp[i+7],cp[i+7],dp[i+7]);
        }


        rt_task_sleep(sec+nsec-sec_-nsec_-2000);//***
	    sec_ = sec;
	    nsec_ = nsec;
    }
    while (ave > 500){
        for (int i=0; i<7; i++){
            ave += abs(tr[i+7]-pos_desired[i]);
        }
        ave = ave/7;
    }

    control2 = 0;
}


void wheel_callback(const ethercat_test::vel& msg)
{
    for(int i = 0; i < 4;++i){
        wheeldes[i] = msg.velocity[i];
    	//rt_printf("wheel %d : %d\n",i,msg.velocity[i]);
    }
}

void catch_signal(int sig)
{
    run = 0;
    usleep(5e5);
    rt_task_delete(&motion_task);
    rt_task_delete(&pub_task);
    exit(1);
}

int main(int argc, char** argv)
{
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("use default adapter %s\n", ecat_ifname);

    cpu_set_t cpu_set_ecat;
    CPU_ZERO(&cpu_set_ecat);
    CPU_SET(0, &cpu_set_ecat); //assign CPU#0 for ethercat task
    cpu_set_t cpu_set_pub;
    CPU_ZERO(&cpu_set_pub);
    CPU_SET(1, &cpu_set_pub); //assign CPU#1 (or any) for main task


    resol_conv();

    ros::init(argc, argv, "dualarm_sub");
    ros::NodeHandle n;
//    printf("1\n");
    ros::Subscriber pos_sub1 = n.subscribe("dualarm/manipulator1_controller/follow_joint_trajectory/goal", 1, motion_callback1);
//    printf("2\n");
    ros::Subscriber pos_sub2 = n.subscribe("dualarm/manipulator2_controller/follow_joint_trajectory/goal", 1, motion_callback2);
//    printf("3\n");
    ros::Subscriber vel_sub = n.subscribe("input_msg",1, wheel_callback);
//    printf("4\n");
    rt_task_create(&motion_task, "SOEM_motion_task", 0, 95, 0 );
    rt_task_set_affinity(&motion_task, &cpu_set_ecat); //CPU affinity for ethercat task
//    printf("5\n");
    rt_task_create(&pub_task, "ec_printing", 0, 50, 0 );
    rt_task_set_affinity(&pub_task, &cpu_set_pub); //CPU affinity for printing task
//    rt_printf("6\n");
    rt_task_start(&motion_task, &EPOS_OP, NULL);
    rt_task_start(&pub_task, &pub_run, NULL);
//    rt_printf("7\n");




    ros::spin();

    run = 0;
    usleep(5e5);
    rt_task_delete(&motion_task);
    rt_task_delete(&pub_task);

    printf("End program\n");
    return (0);


}
