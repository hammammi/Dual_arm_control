//
// Created by Hyam on 2020-09-17.
//

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <xenomai/init.h>

#include "soem/ethercat.h"
#include "pdo_def.h"
#include "servo_def.h"
#include "ecat_dc.h"

#define EC_TIMEOUTMON 500
#define NUMOFEPOS4_DRIVE	7
#define NSEC_PER_SEC 1000000000
unsigned int cycle_ns = 1000000;

EPOS4_Drive_pt	epos4_drive_pt[NUMOFEPOS4_DRIVE];
int started[NUMOFEPOS4_DRIVE]={0}, HomingState=0;
uint8 servo_ready = 0, servo_prestate = 0;

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

RT_TASK motion_task;
//RT_TASK pub_task;

RTIME now, previous;
long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "eno1";
int step = 0, run = 1;
int sys_ready = 0;


int recv_fail_cnt = 0;
int wait = 0;

int32_t homeoffset[] ={1183288,685834,1064152,586632,331032,248261,368469};// for ver1

int first_step[] = {0,2,4,6};//0,2,4,6}; //vertical axis
int second_step[] = {1,3,5};//1,3,5};  //horizontal axis

int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;

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

            for (int k=0; k<(NUMOFEPOS4_DRIVE+1); ++k)
            {
                //rt_printf("%i\n",k);
                //rt_printf(ec_slave[k].name);
                //rt_printf("\n");

                if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k+1].name,"EPOS4") == 0)) //change name for other drives
                {
                    rt_printf("Re mapping for EPOS4 %d...\n", k+1);

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
                    // 4. DigitalInput(LogicState) UINT16
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

                    os=sizeof(ob3); ob3 = 0x01;     // interporation time period
                    wkc_count=ec_SDOwrite(k+1, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x06;  // mode of operation 0x06 -> homing mode
                    wkc_count=ec_SDOwrite(k+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    //os=sizeof(ob); ob = 0x000186A0; // Following error window
                    //wkc_count=ec_SDOwrite(k+1, 0x6065, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os = sizeof(ob);
                    ob = homeoffset[k-1]; // Home offset move distance : No PDO mapping
                    //ob = 120000;
                    wkc_count=ec_SDOwrite(k+1, 0x30B1, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os = sizeof(ob);
                    ob = 200; // Speed for switch search -> less than max speed (0x3000, you can see the value using epos studio)
                    wkc_count=ec_SDOwrite(k+1, 0x6099, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os = sizeof(ob);
                    ob = 1000; // Homing acceleration
                    wkc_count=ec_SDOwrite(k+1, 0x609A, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);


                    os = sizeof(ob);
                    ob = 0; // Home position
                    wkc_count=ec_SDOwrite(k+1, 0x30B0, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob); ob = 0x05F5E100; // Following error window
                    wkc_count=ec_SDOwrite(k+1, 0x6065, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);


                    if (k == 6){
                        os = sizeof(ob3);
                        ob3 = 17; // Homing method : Negative Limit switch
                        wkc_count=ec_SDOwrite(k+1, 0x6098, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        //For Negative Limit switch method using negative limit switch
                        //Connect vcc to COM, DigIn to NO
                        // negative 0 positive 1 homeswitch 2
                        // each function can only be mapped once, so default value is important.

                        os = sizeof(ob3);
                        ob3 = 20;
                        wkc_count=ec_SDOwrite(k+1, 0x3142, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        os = sizeof(ob3);
                        ob3 = 0;
                        wkc_count=ec_SDOwrite(k+1, 0x3142, 0x04, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    }
                    else{
                        os = sizeof(ob3);
                        ob3 = 18; // Homing method : Positive Limit switch
                        wkc_count=ec_SDOwrite(k+1, 0x6098, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        //For Positive Limit switch method using positive limit switch
                        //Connect vcc to COM, DigIn to NO
                        // negative 0 positive 1 homeswitch 2
                        // each function can only be mapped once, so default value is important.

                        os = sizeof(ob3);
                        ob3 = 20;
                        wkc_count=ec_SDOwrite(k+1, 0x3142, 0x02, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        os = sizeof(ob3);
                        ob3 = 1;
                        wkc_count=ec_SDOwrite(k+1, 0x3142, 0x04, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        //os = sizeof(ob3);
                        //ob3 = 20;
                        //wkc_count=ec_SDOwrite(k+1, 0x3142, 0x02, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        //os = sizeof(ob3);
                        //ob3 = 2;
                        //wkc_count=ec_SDOwrite(k+1, 0x3142, 0x03, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    }
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
                for (i=0; i<ec_slavecount; i++)
                {
                    if (ec_slave[i+2].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State 0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i+1, ec_slave[i+2].state, ec_slave[i+2].ALstatuscode, ec_ALstatuscode2string(ec_slave[i+2].ALstatuscode));
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




void EPOS_HOMING(void *arg)
{
    unsigned long ready_cnt = 0;
    uint16_t controlword=0;
    int i, j, k, p;

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

    rt_printf("Mode of operation : Homing Mode\n");

    i = 0;


    while (run) {
        // wait for next cycle
        rt_ts += (RTIME)(cycle_ns + toff);
        rt_task_sleep_until(rt_ts);

        previous = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3 * NUMOFEPOS4_DRIVE)
            recv_fail_cnt++;
        now = rt_timer_read();
        ethercat_time = (long) (now - previous);

        cur_dc32 = (uint32_t)(ec_DCtime & 0xFFFFFFFF);
        if (cur_dc32 > pre_dc32)
            diff_dc32 = cur_dc32 - pre_dc32;
        else
            diff_dc32 = (0xFFFFFFFF - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;


        //servo-on & homing
//        k=0;
//        k = homingOrder[i]-1;
        //k = i;
        controlword = 0;



        for (j = 0; j < NUMOFEPOS4_DRIVE; ++j) {
            //epos4_drive_pt[j].ptOutParam->ModeOfOperation = OP_MODE_HOMING;

            //epos4_drive_pt[j].ptOutParam->HomingMethod = 18;
            controlword = 0;

            started[j] = ServoOn_GetCtrlWrd(epos4_drive_pt[j].ptInParam->StatusWord, &controlword);
            epos4_drive_pt[j].ptOutParam->ControlWord = controlword;
        }
        ready_cnt++;

        if (ready_cnt>=3000){
            controlword = 0x1F;
            if (step == 0){
                p = sizeof(first_step)/sizeof(first_step[0]);
                for (k=0;k<p;k++){
                    epos4_drive_pt[first_step[k]].ptOutParam->ControlWord = controlword;
                    rt_printf("%i Actual Position = %i / %i\n", first_step[k], epos4_drive_pt[first_step[k]].ptInParam->PositionActualValue, homeoffset[first_step[k]]);
                    if (bit_is_set(epos4_drive_pt[first_step[k]].ptInParam->StatusWord,STATUSWORD_HOMING_ATTAINED_BIT))
                    {
                        rt_printf("Epos %i's homing is completed",first_step[k]);
                        rt_printf("StatusWord = 0x%X",epos4_drive_pt[first_step[k]].ptInParam->StatusWord);
                        HomingState |= (1<<k);
                    }
                    rt_printf("\n%i\n",((1<<p)-1));
                    rt_printf("\n%i\n",HomingState);
                    if (HomingState == ((1<<p)-1))
                    {
                        step=1;
                        ready_cnt=0;
                        HomingState = 0;
                    }
                    if (step==1)
                    {HomingState=0;}
                }
            }
            else {
                p = sizeof(second_step) / sizeof(second_step[0]);
                for (k = 0; k < p; k++) {
                    epos4_drive_pt[second_step[k]].ptOutParam->ControlWord = controlword;
                    rt_printf("%i Actual Position = %i / %i\n", second_step[k],
                              epos4_drive_pt[second_step[k]].ptInParam->PositionActualValue,
                              homeoffset[second_step[k]]);
                    if (bit_is_set(epos4_drive_pt[second_step[k]].ptInParam->StatusWord,
                                   STATUSWORD_HOMING_ATTAINED_BIT)) {
                        rt_printf("Epos %i's homing is completed", second_step[k]);
                        rt_printf("StatusWord = 0x%X", epos4_drive_pt[second_step[k]].ptInParam->StatusWord);
                        HomingState |= (1 << k);
                    }
                    rt_printf("\n%i\n", ((1 << p) - 1));
                    rt_printf("\n%i\n", HomingState);
                    rt_printf("\n%i\n", step);
                    if (HomingState == ((1 << p) - 1)) {
                        step = 2;
                        ready_cnt = 0;

                    }
                }
            }
        }

        if (step == 2) //all servos are attained homing
        {
            run = 0;
        }


        if (worst_time<ethercat_time) worst_time=ethercat_time;

    }

    rt_task_sleep(cycle_ns);

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+2, FALSE, 0, 0); // SYNC0,1 on slave 1

    //Servo OFF
    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
    {
        epos4_drive_pt[i].ptOutParam->ControlWord=6; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    rt_printf("End Mani_ver1 Homing, close socket\n");
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

//void pub_run(void *arg)
//{
//    int i;
//    unsigned long itime = 0;
//    long stick = 0;
//
//    while (run)
//    {
//        rt_task_wait_period(NULL);
//        if (inOP==TRUE)
//        {
//            if (!sys_ready)
//            {
//                if(stick==0)
//                    rt_printf("waiting for system ready...\n");
//                if(stick%10==0)
//                    rt_printf("%i \n", stick/10);
//                stick++;
//            }
//
//        }
//    }
//}


void catch_signal(int sig)
{
    run = 0;
    usleep(5e5);
    rt_task_delete(&motion_task);
//    rt_task_delete(&pub_task);
    exit(1);
}

int main(int argc, char** argv)
{
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);


    cycle_ns=1000000; // nanosecond

    printf("use default adapter %s\n", ecat_ifname);

    cpu_set_t cpu_set_ecat;
    CPU_ZERO(&cpu_set_ecat);
    CPU_SET(0, &cpu_set_ecat); //assign CPU#0 for ethercat task

//    cpu_set_t cpu_set_pub;
//    CPU_ZERO(&cpu_set_pub);
//    CPU_SET(1, &cpu_set_pub); //assign CPU#2 (or any) for main task



    rt_task_create(&motion_task, "SOEM_motion_task", 0, 97, 0 );
    rt_task_set_affinity(&motion_task, &cpu_set_ecat); //CPU affinity for ethercat task

//    rt_task_create(&pub_task, "pos_pub_task", 0, 50, 0 );
//    rt_task_set_affinity(&pub_task, &cpu_set_pub); //CPU affinity for printing task


    rt_task_start(&motion_task, &EPOS_HOMING, NULL);
//    rt_task_start(&pub_task, &pub_run, NULL);


    while (run)
    {
        usleep(1000000);
    }


    printf("End program\n");
    return (0);


}
