#include <filesystem>
#include <iostream>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <sstream>
// For data storage
#include <fstream>
#include <iomanip>
#include <cmath> 
#include <math.h>
//timer
#include <chrono>
//
#include "Motor_TaiHu.hpp"
#include "Auto_Set_Id.hpp"
#include "Custom_TOP.hpp"
#include "HARDWARE_TOP.hpp"
// #include "Hw_Pressure_Sensor.hpp"
#include "Motor_TOP.hpp"
#include "Switch_Board.hpp"
#include "Switch_Board_Orin.hpp"
#include "syst.hpp"
#include "UDP.hpp"
#include "unistd.h"

using namespace std;
Robot_Hardware* Y_bot;
//
//
// 收放线方向约定（模式2设定，模式3可独立覆盖）
// true  = 逆时针为放线方向、顺时针为收线方向 — 对应逆时针绕线
// false = 顺时针为放线方向、逆时针为收线方向 — 对应顺时针绕线
bool g_ccw_is_release = true; // 默认：逆时针=放线
// 电机名稱
typedef struct Y_bot_TaiHu_Out_Data
{
    float left_limb_iliopsoas;  // left_shoulder_pitch;
    float left_limb_quadriceps; // left_shoulder_roll;
    float left_limb_hamstring;  // left_shoulder_yaw;
    float left_elbow;
    float left_wrist_yaw;
    float left_wrist_pitch;
    float left_wrist_roll;

    float right_limb_iliopsoas;  // right_shoulder_pitch;
    float right_limb_quadriceps; // right_shoulder_roll;
    float right_limb_hamstring;  // right_shoulder_yaw;
    float right_elbow;
    float right_wrist_yaw;
    float right_wrist_pitch;
    float right_wrist_roll;

    float head_pitch;
    float head_yaw;
} Y_bot_TaiHu_Out_Data;

shared_ptr<Device_class> Main_Switch_Board;
/**
 * @brief W_Bot数据结构体
 */
Y_bot_TaiHu_Out_Data W_Bot_OD_Get;
Y_bot_TaiHu_Out_Data W_Bot_OD_Set;

/**
 * @brief 下肢电机
 */
float left_limb_iliopsoas;  // left_shoulder_pitch;
float left_limb_quadriceps; // left_shoulder_roll;
float left_limb_hamstring;  // left_shoulder_yaw;
float left_elbow;

shared_ptr<Device_class> Lower_Limbs_Motor_Waist_Roll;
shared_ptr<Device_class> Lower_Limbs_Motor_Waist_Yaw;
shared_ptr<Device_class> Lower_Limbs_Motor_Knee;
shared_ptr<Device_class> Lower_Limbs_Motor_Hip;
shared_ptr<Device_class> Lower_Limbs_Motor_Ankel;
shared_ptr<Device_class> Lower_Limbs_Motor_Ankel_Right;
/**
 * @brief 左臂电机
 */
shared_ptr<Device_class> TaiHu_Device_1;
shared_ptr<Device_class> TaiHu_Device_2;
shared_ptr<Device_class> TaiHu_Device_3;
shared_ptr<Device_class> TaiHu_Device_4;
shared_ptr<Device_class> TaiHu_Device_5;
shared_ptr<Device_class> TaiHu_Device_6;
shared_ptr<Device_class> TaiHu_Device_7;
/**
 * @brief 右臂电机
 */
shared_ptr<Device_class> TaiHu_Device_8;
shared_ptr<Device_class> TaiHu_Device_9;
shared_ptr<Device_class> TaiHu_Device_10;
shared_ptr<Device_class> TaiHu_Device_11;
shared_ptr<Device_class> TaiHu_Device_12;
shared_ptr<Device_class> TaiHu_Device_13;
shared_ptr<Device_class> TaiHu_Device_14;
/**
 * @brief 头部电机
 */
shared_ptr<Device_class> TaiHu_Device_15;
shared_ptr<Device_class> TaiHu_Device_16;

Main_B *Main_Switch_Board_Control;
Main_B *Waist_Main_Switch_Board_Control;
Main_B *Chassis_Main_Switch_Board_Control;

Motor_TaiHu *TaiHu_Device_T1;
Motor_TaiHu *TaiHu_Device_T2;
Motor_TaiHu *TaiHu_Device_T3;
Motor_TaiHu *TaiHu_Device_T4;
Motor_TaiHu *TaiHu_Device_T5;
Motor_TaiHu *TaiHu_Device_T6;
Motor_TaiHu *TaiHu_Device_T7;

Motor_TaiHu *TaiHu_Device_T8;
Motor_TaiHu *TaiHu_Device_T9;
Motor_TaiHu *TaiHu_Device_T10;
Motor_TaiHu *TaiHu_Device_T11;
Motor_TaiHu *TaiHu_Device_T12;
Motor_TaiHu *TaiHu_Device_T13;
Motor_TaiHu *TaiHu_Device_T14;

Motor_TaiHu *TaiHu_Device_T15;
Motor_TaiHu *TaiHu_Device_T16;
//


shared_ptr<Device_class> Motor_1_D;
shared_ptr<Device_class> Motor_2_D;
shared_ptr<Device_class> Motor_3_D;
shared_ptr<Device_class> Motor_4_D;
shared_ptr<Device_class> Motor_5_D;
shared_ptr<Device_class> Motor_6_D;
shared_ptr<Device_class> Motor_7_D;
shared_ptr<Device_class> Motor_8_D;

Motor* Motor_1_Control;
Motor* Motor_2_Control;
Motor* Motor_3_Control;
Motor* Motor_4_Control;
Motor* Motor_5_Control;
Motor* Motor_6_Control;
Motor* Motor_7_Control;
Motor* Motor_8_Control;


typedef struct Y_bot_FB {
    float P;
    float V;
    float F;
    float temp[2];
    u16 error;
} Y_bot_FB;

typedef struct Y_bot_Send_Data {
    float P;
    float V;
    float F;
    float KP;
    float KD;
} Y_bot_Send_Data;

// int init_time_step = 1000 * 1000 / 500;
// float Pos_Offest[6] = {0, 0, 0, 0, 0, 0};
// float Motor_Mirror[6] = {-1, -1, 1, 1, 1, 1};
// float Motor_K[6] = {3900, 1, 1, 1, 1, 1};c
float Motor_K[8] = {81920 , 81920 , 81920, 81920, 81920, 81920, 81920, 81920};//, 81920
// // float Motor_K[6] = {3103 - 20, 421.953 - 5, 3779.49 - 20, 3833.81 - 20, 3739.36 - 20, 3780.85 - 20};
// float Motor_K[6] = {
//     2462.73 - 100,
//     465.147,
//     4207.49 - 200,
//     4292.93 - 200,
//     4163.54 - 200,
//     4137.34 - 200,
// };

Y_bot_Send_Data Send_Datas[8];
Y_bot_FB FB_Datas[8];

// map<u8, vector<u16>> g_sensor_data;

void Get_FB(void) {
    Motor_1_Control->Get_Motor_FB_Data(Motor_1_D, &FB_Datas[0].P, &FB_Datas[0].V, &FB_Datas[0].F, FB_Datas[0].temp, &FB_Datas[0].error);
    Motor_2_Control->Get_Motor_FB_Data(Motor_2_D, &FB_Datas[1].P, &FB_Datas[1].V, &FB_Datas[1].F, FB_Datas[1].temp, &FB_Datas[1].error);
    Motor_3_Control->Get_Motor_FB_Data(Motor_3_D, &FB_Datas[2].P, &FB_Datas[2].V, &FB_Datas[2].F, FB_Datas[2].temp, &FB_Datas[2].error);
    Motor_4_Control->Get_Motor_FB_Data(Motor_4_D, &FB_Datas[3].P, &FB_Datas[3].V, &FB_Datas[3].F, FB_Datas[3].temp, &FB_Datas[3].error);
    Motor_5_Control->Get_Motor_FB_Data(Motor_5_D, &FB_Datas[4].P, &FB_Datas[4].V, &FB_Datas[4].F, FB_Datas[4].temp, &FB_Datas[4].error);
    Motor_6_Control->Get_Motor_FB_Data(Motor_6_D, &FB_Datas[5].P, &FB_Datas[5].V, &FB_Datas[5].F, FB_Datas[5].temp, &FB_Datas[5].error);
    Motor_7_Control->Get_Motor_FB_Data(Motor_7_D, &FB_Datas[6].P, &FB_Datas[6].V, &FB_Datas[6].F, FB_Datas[6].temp, &FB_Datas[6].error);
    Motor_8_Control->Get_Motor_FB_Data(Motor_8_D, &FB_Datas[7].P, &FB_Datas[7].V, &FB_Datas[7].F, FB_Datas[7].temp, &FB_Datas[7].error);
        // FB_Datas[1].P = (FB_Datas[1].P - Pos_Offest[1]) * Motor_Mirror[1] / Motor_K[1];
    // for (int i = 0; i < 6; i++) {
    //     FB_Datas[i].P = (FB_Datas[i].P - Pos_Offest[i]) * Motor_Mirror[i] / Motor_K[i];
    //     FB_Datas[i].F = FB_Datas[i].F * Motor_Mirror[i];
    //     FB_Datas[i].V = FB_Datas[i].V * Motor_Mirror[i] / Motor_K[i];
    // }
}

void Send(void) {

    Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, Send_Datas[0].P , Send_Datas[0].V , Send_Datas[0].F , Send_Datas[0].KP, Send_Datas[0].KD);
    Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, Send_Datas[1].P , Send_Datas[1].V , Send_Datas[1].F , Send_Datas[1].KP, Send_Datas[1].KD);
    Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, Send_Datas[2].P , Send_Datas[2].V , Send_Datas[2].F , Send_Datas[2].KP, Send_Datas[2].KD);
    Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, Send_Datas[3].P , Send_Datas[3].V , Send_Datas[3].F , Send_Datas[3].KP, Send_Datas[3].KD);
    Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, Send_Datas[4].P , Send_Datas[4].V , Send_Datas[4].F , Send_Datas[4].KP, Send_Datas[4].KD);
    Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, Send_Datas[5].P , Send_Datas[5].V , Send_Datas[5].F , Send_Datas[5].KP, Send_Datas[5].KD);
    Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, Send_Datas[6].P , Send_Datas[6].V , Send_Datas[6].F , Send_Datas[6].KP, Send_Datas[6].KD);
    Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, Send_Datas[7].P , Send_Datas[7].V , Send_Datas[7].F , Send_Datas[7].KP, Send_Datas[7].KD);
     Y_bot->Send_Buff_Data();
}

#ifndef HAVE_ROS
int main(int argc, char* argv[])
#else
int hardware_init(const string& ADDR, const string& Config)
#endif
{
    UDP_Def UDP_Def_one;
    Init_One_UDP_Connect(&UDP_Def_one, inet_addr("127.0.0.1"), htons(16201));
    Y_bot = new Robot_Hardware();
    Y_bot->Add_Device_Type(Switch_Board_Type, Switch_Board_Device_Init, Switch_Board_Device_CallBack_F, Switch_Board_Device_Delete_F);
    Y_bot->Add_Device_Type(Switch_Board_Orin_Type, Switch_Board_Orin_Device_Init, Switch_Board_Orin_Device_CallBack_F, Switch_Board_Device_Orin_Delete_F);
    Y_bot->Add_Device_Type(Motor_Device_Type, Motor_Device_Init, Motor_Device_CallBack_F, Motor_Device_Delete_F);
    Y_bot->Add_Device_Type(Auto_Set_Id_Type, Auto_Set_Id_Init, Auto_Set_Id_CallBack_F, Auto_Set_Id_Delete_F);
    // Y_bot->Add_Device_Type("Tactile_Sensor_Custom", Hw_Pressure_Sensor_Init, Hw_Pressure_Sensor_CallBack_F, Hw_Pressure_Sensor_Delete_F);
    Y_bot->Add_Device_Type("TaiHu_Custom_Motor", Motor_Device_Init_TaiHu, Motor_Device_CallBack_TaiHu, Motor_Device_Delete_TaiHu);
#ifndef HAVE_ROS
    filesystem::path exe_path = filesystem::canonical("/proc/self/exe");
    filesystem::path dir_path = exe_path.parent_path();
    string ADDR = dir_path.string() + "/../config/YAML/Y_Bot/out/mz22/TOP.yaml";
    string Config = "None";
//     string Config = R"(
// PC_IP: 192.168.3.245
// SN: 1X1T2603005MG
// Boards:
//   - Id: 101
//     IP: 192.168.3.105
//     # Port: 19001
// )";
#endif
    if (Y_bot->Init_TOP(ADDR, Config) != 0) {
        cout << "Init_ERR" << endl;
        return -1;
    }
//
Main_Switch_Board = Y_bot->Get_Device_For_Name("Main_Switch_Board");
    Main_Switch_Board_Control =
        static_cast<Main_B *>(Y_bot->Get_Control_Class(Main_Switch_Board));

    TaiHu_Device_1 = Y_bot->Get_Device_For_Name(
        "left_limb_iliopsoas"); // left_shoulder_pitch
    TaiHu_Device_T1 = static_cast<Motor_TaiHu *>(
        Y_bot->Get_Control_Class(TaiHu_Device_1));

    TaiHu_Device_2 = Y_bot->Get_Device_For_Name(
        "left_limb_quadriceps"); // left_shoulder_roll
    TaiHu_Device_T2 = static_cast<Motor_TaiHu *>(
        Y_bot->Get_Control_Class(TaiHu_Device_2));

    TaiHu_Device_3 = Y_bot->Get_Device_For_Name(
        "left_limb_hamstring"); // left_shoulder_yaw
    TaiHu_Device_T3 = static_cast<Motor_TaiHu *>(
        Y_bot->Get_Control_Class(TaiHu_Device_3));

    TaiHu_Device_8 =
        Y_bot->Get_Device_For_Name(
            "right_limb_iliopsoas"); // right_shoulder_pitch
    TaiHu_Device_T8 = static_cast<Motor_TaiHu *>(
        Y_bot->Get_Control_Class(TaiHu_Device_8));

    TaiHu_Device_9 =
        Y_bot->Get_Device_For_Name(
            "right_limb_quadriceps"); // right_shoulder_roll
    TaiHu_Device_T9 = static_cast<Motor_TaiHu *>(
        Y_bot->Get_Control_Class(TaiHu_Device_9));

    TaiHu_Device_10 =
        Y_bot->Get_Device_For_Name(
            "right_limb_hamstring"); // right_shoulder_yaw
    TaiHu_Device_T10 = static_cast<Motor_TaiHu *>(
        Y_bot->Get_Control_Class(TaiHu_Device_10));

    vector<Motor_TaiHu *> Motor_vector;
    Motor_vector.push_back(TaiHu_Device_T1);
    Motor_vector.push_back(TaiHu_Device_T2);
    Motor_vector.push_back(TaiHu_Device_T3);
    Motor_vector.push_back(TaiHu_Device_T8);
    Motor_vector.push_back(TaiHu_Device_T9);
    Motor_vector.push_back(TaiHu_Device_T10);

    vector<shared_ptr<Device_class>> Device_vector;
    Device_vector.push_back(TaiHu_Device_1);
    Device_vector.push_back(TaiHu_Device_2);
    Device_vector.push_back(TaiHu_Device_3);
    Device_vector.push_back(TaiHu_Device_8);
    Device_vector.push_back(TaiHu_Device_9);
    Device_vector.push_back(TaiHu_Device_10);

//
    Motor_1_D = Y_bot->Get_Device_For_Name("Motor_1");
    Motor_2_D = Y_bot->Get_Device_For_Name("Motor_2");
    Motor_3_D = Y_bot->Get_Device_For_Name("Motor_3");
    Motor_4_D = Y_bot->Get_Device_For_Name("Motor_4");
    Motor_5_D = Y_bot->Get_Device_For_Name("Motor_5");
    Motor_6_D = Y_bot->Get_Device_For_Name("Motor_6");
    Motor_7_D = Y_bot->Get_Device_For_Name("Motor_7");
    Motor_8_D = Y_bot->Get_Device_For_Name("Motor_8");
    // Tactile_Sensor_D = Y_bot->Get_Device_For_Name("Tactile_Sensor");

    Motor_1_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_1_D));
    Motor_2_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_2_D));
    Motor_3_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_3_D));
    Motor_4_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_4_D));
    Motor_5_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_5_D));
    Motor_6_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_6_D));
    Motor_7_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_7_D));
    Motor_8_Control = static_cast<Motor*>(Y_bot->Get_Control_Class(Motor_8_D));

    // Y_bot_Init();

// auto check_ptr = [](const char* name, auto ptr){
//     if (ptr == nullptr ){
//         cout << "error: " << name << " is nullptr" << endl;
//         return false;

//     }
//     cout << " ok: " << name << endl;
//     return true;
// };
// check_ptr("TaiHu_Device_1", TaiHu_Device_1);
// check_ptr("TaiHu_Device_T1", TaiHu_Device_T1);
// check_ptr("TaiHu_Device_2", TaiHu_Device_2);
// check_ptr("TaiHu_Device_T2", TaiHu_Device_T2);
// check_ptr("TaiHu_Device_3", TaiHu_Device_3);
// check_ptr("TaiHu_Device_T3", TaiHu_Device_T3);
// check_ptr("TaiHu_Device_8", TaiHu_Device_8);
// check_ptr("TaiHu_Device_T8", TaiHu_Device_T8);
// check_ptr("TaiHu_Device_9", TaiHu_Device_9);
// check_ptr("TaiHu_Device_T9", TaiHu_Device_T9);
// check_ptr("TaiHu_Device_10", TaiHu_Device_10);
// check_ptr("TaiHu_Device_T10", TaiHu_Device_T10);



#ifndef HAVE_ROS
    Get_FB();
    constexpr int loop_time_step = 1000 * 10000 / 1000;
    using clock = chrono::steady_clock;

    // ============================================================
    // Pretension 1: run once, for 5 seconds
    // ============================================================
#if 1
    {
        cout << "[PRETENSION 1 START]" << endl;
        auto start_time = clock::now();

        while (true) {
            Get_FB();
            auto now = clock::now();
            double elapsed_time = chrono::duration<double>(now - start_time).count();
            if (elapsed_time >= 5.0) {
                break;
            }

            TaiHu_Device_T1->Send_MIT_PD_Control_Data(TaiHu_Device_1, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            TaiHu_Device_T2->Send_MIT_PD_Control_Data(TaiHu_Device_2, -25.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, 43.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            TaiHu_Device_T8->Send_MIT_PD_Control_Data(TaiHu_Device_8, -10.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            TaiHu_Device_T9->Send_MIT_PD_Control_Data(TaiHu_Device_9, -40.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, 40.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Y_bot->Send_Buff_Data();

            usleep(500000);

            Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, 0.0, 0.0, 0.035f, 0.0, 0.0);
            Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, 0.0, 0.0, 0.03f, 0.0, 0.0);
            Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, 0.0, 0.0, 0.03f, 0.0, 0.0);
            Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, 0.0, 0.0, 0.035f, 0.0, 0.0);
            Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, 0.0, 0.0, 0.02f, 0.0, 0.0);
            Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, 0.0, 0.0, 0.06f, 0.0, 0.0);
            Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, 0.0, 0.0, 0.03f, 0.0, 0.0);
            Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, 0.0, 0.0, 0.035f, 0.0, 0.0);
            Y_bot->Send_Buff_Data();

            usleep(loop_time_step);

            for (int i = 0; i < 8; i++) {
                cout << "Motor:" << i + 1
                     << " Current:" << FB_Datas[i].F
                     << "\tPosition:" << FB_Datas[i].P
                     << "\t" << elapsed_time << endl;
            }
        }
        cout << "[PRETENSION 1 FINISHED]" << endl;
    }
#endif

    Get_FB();
    const float p[8] = {FB_Datas[0].P, FB_Datas[1].P, FB_Datas[2].P, FB_Datas[3].P,
                        FB_Datas[4].P, FB_Datas[5].P, FB_Datas[6].P, FB_Datas[7].P};

    // ============================================================
    // Pretension 2: read TH feedback and hold all MZ motors once
    // ============================================================
#if 1
    {
        cout << "[PRETENSION 2 START]" << endl;
        float thp[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        float thv[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        float thf[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        TaiHu_Device_T1->Get_Motor_FB_Data(TaiHu_Device_1, &thp[0], &thv[0], &thf[0]);
        TaiHu_Device_T2->Get_Motor_FB_Data(TaiHu_Device_2, &thp[1], &thv[1], &thf[1]);
        TaiHu_Device_T3->Get_Motor_FB_Data(TaiHu_Device_3, &thp[2], &thv[2], &thf[2]);
        TaiHu_Device_T8->Get_Motor_FB_Data(TaiHu_Device_8, &thp[3], &thv[3], &thf[3]);
        TaiHu_Device_T9->Get_Motor_FB_Data(TaiHu_Device_9, &thp[4], &thv[4], &thf[4]);
        TaiHu_Device_T10->Get_Motor_FB_Data(TaiHu_Device_10, &thp[5], &thv[5], &thf[5]);

        for (int i = 0; i < 6; i++) {
            cout << "TH" << i + 1 << "\t" << thp[i] << endl;
        }

        Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, p[0], 0.0, 0.0, 500.0, 100.0);
        Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, p[1], 0.0, 0.0, 500.0, 100.0);
        Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, p[2], 0.0, 0.0, 500.0, 100.0);
        Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, p[3], 0.0, 0.0, 500.0, 100.0);
        Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, p[4], 0.0, 0.0, 500.0, 100.0);
        Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, p[5], 0.0, 0.0, 500.0, 100.0);
        Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, p[6], 0.0, 0.0, 500.0, 100.0);
        Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, p[7], 0.0, 0.0, 500.0, 100.0);
        Y_bot->Send_Buff_Data();
        usleep(loop_time_step);

        cout << "[PRETENSION 2 FINISHED]" << endl;
    }
#endif

    // ============================================================
    // Main repeated action sequence
    // Order: Hip abduction -> Knee Flexion -> Swing Leg -> Ankle motion
    // ============================================================
    int sequence_id = 0;
    while (true) {
        cout << "==============================" << endl;
        cout << "[MAIN SEQUENCE] cycle = " << sequence_id << endl;
        cout << "==============================" << endl;

        // ========================================================
        // Action 1: Hip abduction
        // ========================================================
#if 1
        {
            cout << "[ACTION] Hip abduction" << endl;

            const int dt_us = 10000;
            const float T_phase = 2.0f;
            const float T_home  = 1.0f;

            const int N_phase = static_cast<int>(T_phase * 1000000.0f / dt_us);
            const int N_home  = static_cast<int>(T_home  * 1000000.0f / dt_us);
            const int move_times = 3;

            const float mz1_home = p[0];
            const float mz5_home = p[4];
            const float th3_home = 43.0f;
            const float th10_home = 40.0f;

            const float mz1_A = p[0] + 400.0f;
            const float mz5_A = p[4];
            const float th3_A = 13.0f;
            const float th10_A = 40.0f;

            const float mz1_B = p[0];
            const float mz5_B = p[4] + 400.0f;
            const float th3_B = 43.0f;
            const float th10_B = 10.0f;

            for (int k = 0; k <= N_phase; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_phase);
                float mz1_cmd = mz1_home + r * (mz1_A - mz1_home);
                float mz5_cmd = mz5_home + r * (mz5_A - mz5_home);
                float th3_cmd = th3_home + r * (th3_A - th3_home);
                float th10_cmd = th10_home + r * (th10_A - th10_home);

                Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, mz1_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, mz5_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }

            usleep(500000);

            for (int cycle = 0; cycle < move_times; ++cycle) {
                cout << "[Hip cycle " << cycle + 1 << "] A -> B" << endl;
                for (int k = 0; k <= N_phase; ++k) {
                    float r = static_cast<float>(k) / static_cast<float>(N_phase);
                    float mz1_cmd = mz1_A + r * (mz1_B - mz1_A);
                    float mz5_cmd = mz5_A + r * (mz5_B - mz5_A);
                    float th3_cmd = th3_A + r * (th3_B - th3_A);
                    float th10_cmd = th10_A + r * (th10_B - th10_A);

                    Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, mz1_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, mz5_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    Y_bot->Send_Buff_Data();
                    usleep(dt_us);
                }

                usleep(500000);

                cout << "[Hip cycle " << cycle + 1 << "] B -> A" << endl;
                for (int k = 0; k <= N_phase; ++k) {
                    float r = static_cast<float>(k) / static_cast<float>(N_phase);
                    float mz1_cmd = mz1_B + r * (mz1_A - mz1_B);
                    float mz5_cmd = mz5_B + r * (mz5_A - mz5_B);
                    float th3_cmd = th3_B + r * (th3_A - th3_B);
                    float th10_cmd = th10_B + r * (th10_A - th10_B);

                    Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, mz1_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, mz5_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    Y_bot->Send_Buff_Data();
                    usleep(dt_us);
                }

                usleep(500000);
            }

            cout << "[Hip return home]" << endl;
            for (int k = 0; k <= N_home; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_home);
                float mz1_cmd = mz1_A + r * (mz1_home - mz1_A);
                float mz5_cmd = mz5_A + r * (mz5_home - mz5_A);
                float th3_cmd = th3_A + r * (th3_home - th3_A);
                float th10_cmd = th10_A + r * (th10_home - th10_A);

                Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, mz1_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, mz5_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }
        }
#endif

        usleep(1000000);

        // ========================================================
        // Action 2: Swing Leg
        // ========================================================
#if 1
        {
            cout << "[ACTION] Swing Leg" << endl;

            const int dt_us = 10000;
            const float T_phase = 1.5f;
            const float T_home = 1.5f;

            const int N_phase = static_cast<int>(T_phase * 1000000.0f / dt_us);
            const int N_home = static_cast<int>(T_home * 1000000.0f / dt_us);
            const int swing_times = 5;

            const float r_th8_home = -10.0f;
            const float r_th9_home = -50.0f;
            const float r_th10_home = 40.0f;
            const float r_mz6_home = p[5];

            const float r_th8_lift = 80.0f;
            const float r_th9_lift = 50.0f;
            const float r_th10_lift = -30.0f;
            const float r_mz6_lift = p[5] - 1000.0f;

            const float r_th8_swing = -50.0f;
            const float r_th9_swing = -90.0f;
            const float r_th10_swing = 80.0f;
            const float r_mz6_swing = p[5] + 180.0f;

            const float l_th1_home = 1.0f;
            const float l_th2_home = -30.0f;
            const float l_th3_home = 43.0f;
            const float l_mz2_home = p[1];

            const float l_th1_lift = 101.0f;
            const float l_th2_lift = 70.0f;
            const float l_th3_lift = -27.0f;
            const float l_mz2_lift = p[1] - 1000.0f;

            const float l_th1_swing = -39.0f;
            const float l_th2_swing = -70.0f;
            const float l_th3_swing = 83.0f;
            const float l_mz2_swing = p[1] + 180.0f;

            float r_th8_now = r_th8_home;
            float r_th9_now = r_th9_home;
            float r_th10_now = r_th10_home;
            float r_mz6_now = r_mz6_home;

            float l_th1_now = l_th1_home;
            float l_th2_now = l_th2_home;
            float l_th3_now = l_th3_home;
            float l_mz2_now = l_mz2_home;

            for (int cycle = 0; cycle < swing_times; ++cycle) {
                for (int k = 0; k <= N_phase; ++k) {
                    float r = static_cast<float>(k) / static_cast<float>(N_phase);

                    float r_th8_cmd = r_th8_now + r * (r_th8_lift - r_th8_now);
                    float r_th9_cmd = r_th9_now + r * (r_th9_lift - r_th9_now);
                    float r_th10_cmd = r_th10_now + r * (r_th10_lift - r_th10_now);
                    float r_mz6_cmd = r_mz6_now + r * (r_mz6_lift - r_mz6_now);

                    float l_th1_cmd = l_th1_now + r * (l_th1_swing - l_th1_now);
                    float l_th2_cmd = l_th2_now + r * (l_th2_swing - l_th2_now);
                    float l_th3_cmd = l_th3_now + r * (l_th3_swing - l_th3_now);
                    float l_mz2_cmd = l_mz2_now + r * (l_mz2_swing - l_mz2_now);

                    TaiHu_Device_T8->Send_MIT_PD_Control_Data(TaiHu_Device_8, r_th8_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T9->Send_MIT_PD_Control_Data(TaiHu_Device_9, r_th9_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, r_th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T1->Send_MIT_PD_Control_Data(TaiHu_Device_1, l_th1_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T2->Send_MIT_PD_Control_Data(TaiHu_Device_2, l_th2_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, l_th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, r_mz6_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, l_mz2_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Y_bot->Send_Buff_Data();
                    usleep(dt_us);
                }

                r_th8_now = r_th8_lift;
                r_th9_now = r_th9_lift;
                r_th10_now = r_th10_lift;
                r_mz6_now = r_mz6_lift;
                l_th1_now = l_th1_swing;
                l_th2_now = l_th2_swing;
                l_th3_now = l_th3_swing;
                l_mz2_now = l_mz2_swing;

                usleep(200000);

                for (int k = 0; k <= N_phase; ++k) {
                    float r = static_cast<float>(k) / static_cast<float>(N_phase);

                    float r_th8_cmd = r_th8_now + r * (r_th8_swing - r_th8_now);
                    float r_th9_cmd = r_th9_now + r * (r_th9_swing - r_th9_now);
                    float r_th10_cmd = r_th10_now + r * (r_th10_swing - r_th10_now);
                    float r_mz6_cmd = r_mz6_now + r * (r_mz6_swing - r_mz6_now);

                    float l_th1_cmd = l_th1_now + r * (l_th1_lift - l_th1_now);
                    float l_th2_cmd = l_th2_now + r * (l_th2_lift - l_th2_now);
                    float l_th3_cmd = l_th3_now + r * (l_th3_lift - l_th3_now);
                    float l_mz2_cmd = l_mz2_now + r * (l_mz2_lift - l_mz2_now);

                    TaiHu_Device_T8->Send_MIT_PD_Control_Data(TaiHu_Device_8, r_th8_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T9->Send_MIT_PD_Control_Data(TaiHu_Device_9, r_th9_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, r_th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T1->Send_MIT_PD_Control_Data(TaiHu_Device_1, l_th1_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T2->Send_MIT_PD_Control_Data(TaiHu_Device_2, l_th2_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, l_th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                    Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, r_mz6_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, l_mz2_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Y_bot->Send_Buff_Data();
                    usleep(dt_us);
                }

                r_th8_now = r_th8_swing;
                r_th9_now = r_th9_swing;
                r_th10_now = r_th10_swing;
                r_mz6_now = r_mz6_swing;
                l_th1_now = l_th1_lift;
                l_th2_now = l_th2_lift;
                l_th3_now = l_th3_lift;
                l_mz2_now = l_mz2_lift;

                usleep(200000);
            }

            for (int k = 0; k <= N_home; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_home);
                float r_th8_cmd = r_th8_now + r * (r_th8_home - r_th8_now);
                float r_th9_cmd = r_th9_now + r * (r_th9_home - r_th9_now);
                float r_th10_cmd = r_th10_now + r * (r_th10_home - r_th10_now);
                float r_mz6_cmd = r_mz6_now + r * (r_mz6_home - r_mz6_now);
                float l_th1_cmd = l_th1_now + r * (l_th1_home - l_th1_now);
                float l_th2_cmd = l_th2_now + r * (l_th2_home - l_th2_now);
                float l_th3_cmd = l_th3_now + r * (l_th3_home - l_th3_now);
                float l_mz2_cmd = l_mz2_now + r * (l_mz2_home - l_mz2_now);

                TaiHu_Device_T8->Send_MIT_PD_Control_Data(TaiHu_Device_8, r_th8_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T9->Send_MIT_PD_Control_Data(TaiHu_Device_9, r_th9_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, r_th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T1->Send_MIT_PD_Control_Data(TaiHu_Device_1, l_th1_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T2->Send_MIT_PD_Control_Data(TaiHu_Device_2, l_th2_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, l_th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, r_mz6_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, l_mz2_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }
        }
#endif

        usleep(1000000);

        // ========================================================
        // Action 3: Knee Flexion
        // ========================================================
#if 1
        {
            cout << "[ACTION] Knee Flexion" << endl;

            const int dt_us = 10000;
            const float T_forward = 3.0f;
            const float T_return  = 3.0f;

            const int N_forward = static_cast<int>(T_forward * 1000000.0f / dt_us);
            const int N_return  = static_cast<int>(T_return  * 1000000.0f / dt_us);

            const float mz3_home = p[2];
            const float mz7_home = p[6];
            const float mz3_target = p[2] + 400.0f;
            const float mz7_target = p[6] + 400.0f;

            const float th2_home = -25.0f;
            const float th9_home = -40.0f;
            const float th10_home = 40.0f;
            const float th3_home = 43.0f;

            const float th2_target = -90.0f;
            const float th9_target = -125.0f;
            const float th3_target = 60.0f;
            const float th10_target = 60.0f;

            for (int k = 0; k <= N_forward; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_forward);
                float s = r * r * (3.0f - 2.0f * r);

                float mz3_cmd = mz3_home + s * (mz3_target - mz3_home);
                float mz7_cmd = mz7_home + s * (mz7_target - mz7_home);
                float th2_cmd = th2_home + s * (th2_target - th2_home);
                float th9_cmd = th9_home + s * (th9_target - th9_home);
                float th3_cmd = th3_home + s * (th3_target - th3_home);
                float th10_cmd = th10_home + s * (th10_target - th10_home);

                TaiHu_Device_T2->Send_MIT_PD_Control_Data(TaiHu_Device_2, th2_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T9->Send_MIT_PD_Control_Data(TaiHu_Device_9, th9_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, mz3_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, mz7_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }

            usleep(50000);

            for (int k = 0; k <= N_return; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_return);
                float s = r * r * (3.0f - 2.0f * r);

                float mz3_cmd = mz3_target + s * (mz3_home - mz3_target);
                float mz7_cmd = mz7_target + s * (mz7_home - mz7_target);
                float th2_cmd = th2_target + s * (th2_home - th2_target);
                float th9_cmd = th9_target + s * (th9_home - th9_target);
                float th3_cmd = th3_target + s * (th3_home - th3_target);
                float th10_cmd = th10_target + s * (th10_home - th10_target);

                TaiHu_Device_T2->Send_MIT_PD_Control_Data(TaiHu_Device_2, th2_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T9->Send_MIT_PD_Control_Data(TaiHu_Device_9, th9_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T3->Send_MIT_PD_Control_Data(TaiHu_Device_3, th3_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                TaiHu_Device_T10->Send_MIT_PD_Control_Data(TaiHu_Device_10, th10_cmd, 0.0f, 0.0f, 0.0f, 0.0f);
                Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, mz3_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, mz7_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }
        }
#endif

        usleep(1000000);

        // ========================================================
        // Action 4: Ankle motion
        // ========================================================
#if 1
        {
            cout << "[ACTION] Ankle motion" << endl;

            const int dt_us = 10000;
            const float T_phase = 0.5f;
            const float T_home = 1.0f;

            const int N_phase = static_cast<int>(T_phase * 1000000.0f / dt_us);
            const int N_home = static_cast<int>(T_home * 1000000.0f / dt_us);
            const int move_times = 5;

            const float mz3_home = p[2];
            const float mz4_home = p[3];
            const float mz7_home = p[6];
            const float mz8_home = p[7];

            const float mz3_A = p[2] + 420.0f;
            const float mz4_A = p[3] - 350.0f;
            const float mz7_A = p[6] - 260.0f;
            const float mz8_A = p[7] + 200.0f;

            const float mz3_B = p[2] - 260.0f;
            const float mz4_B = p[3] + 200.0f;
            const float mz7_B = p[6] + 420.0f;
            const float mz8_B = p[7] - 350.0f;

            for (int k = 0; k <= N_phase; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_phase);
                float mz3_cmd = mz3_home + r * (mz3_A - mz3_home);
                float mz4_cmd = mz4_home + r * (mz4_A - mz4_home);
                float mz7_cmd = mz7_home + r * (mz7_A - mz7_home);
                float mz8_cmd = mz8_home + r * (mz8_A - mz8_home);

                Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, mz3_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, mz4_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, mz7_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, mz8_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }

            usleep(200000);

            for (int cycle = 0; cycle < move_times; ++cycle) {
                cout << "[Ankle cycle " << cycle + 1 << "] A -> B" << endl;
                for (int k = 0; k <= N_phase; ++k) {
                    float r = static_cast<float>(k) / static_cast<float>(N_phase);
                    float mz3_cmd = mz3_A + r * (mz3_B - mz3_A);
                    float mz4_cmd = mz4_A + r * (mz4_B - mz4_A);
                    float mz7_cmd = mz7_A + r * (mz7_B - mz7_A);
                    float mz8_cmd = mz8_A + r * (mz8_B - mz8_A);

                    Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, mz3_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, mz4_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, mz7_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, mz8_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Y_bot->Send_Buff_Data();
                    usleep(dt_us);
                }

                usleep(200000);

                cout << "[Ankle cycle " << cycle + 1 << "] B -> A" << endl;
                for (int k = 0; k <= N_phase; ++k) {
                    float r = static_cast<float>(k) / static_cast<float>(N_phase);
                    float mz3_cmd = mz3_B + r * (mz3_A - mz3_B);
                    float mz4_cmd = mz4_B + r * (mz4_A - mz4_B);
                    float mz7_cmd = mz7_B + r * (mz7_A - mz7_B);
                    float mz8_cmd = mz8_B + r * (mz8_A - mz8_B);

                    Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, mz3_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, mz4_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, mz7_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, mz8_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                    Y_bot->Send_Buff_Data();
                    usleep(dt_us);
                }

                usleep(200000);
            }

            cout << "[Ankle return home]" << endl;
            for (int k = 0; k <= N_home; ++k) {
                float r = static_cast<float>(k) / static_cast<float>(N_home);
                float mz3_cmd = mz3_A + r * (mz3_home - mz3_A);
                float mz4_cmd = mz4_A + r * (mz4_home - mz4_A);
                float mz7_cmd = mz7_A + r * (mz7_home - mz7_A);
                float mz8_cmd = mz8_A + r * (mz8_home - mz8_A);

                Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, mz3_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, mz4_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, mz7_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, mz8_cmd, 0.0f, 0.0f, 100.0f, 50.0f);
                Y_bot->Send_Buff_Data();
                usleep(dt_us);
            }
        }
#endif

        usleep(1000000);
        sequence_id++;
        if (sequence_id>= 2){
            break;
        }
    }
#endif

    return 0;
}