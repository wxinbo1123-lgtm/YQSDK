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




    //

#ifndef HAVE_ROS
    Get_FB();
    constexpr int loop_time_step = 1000 * 10000 / 1000; // 1000Hz
    // int times = 500;
    // float test = 0;
    // float PP_old = 0;
    // float PP = 0;
    // float V = 0;
    // float PP_old2 = 0;
    // float PP2 = 0;
    // float V2 = 0;
    // float theta = 1.0f;
    // Paramters for pre-tensioning
    const float Ftar[8] ={ 0.02f, 0.02f, 0.03f, 0.03f,0.02f, 0.02f, 0.03f, 0.03f}; //gear35:0.2=7kg; gear21 0.08
    const float Kcur[8] = {1000.0f, 1000.0f, 500.0f, 500.0f, 1000.0f, 1000.0f, 500.0f, 500.0f}; //gear35:200; gear21
    const float Vmax[8] = {300.0f, 300.0f, 80.0f, 80.0f,300.0f, 300.0f, 80.0f, 80.0f}; //gear35:100; gear21
    const float CurDead[8] = {0.0001,0.0001f,0.0001f,0.0001f,0.0001f,0.0001f,0.0001f,0.0001f}; 

    const float Pmax[8] = {1000.0f,700.0f,3000.0f,3000.0f,3000.0f,3000.0f,3000.0f,3000.0f};
    const float Pmin[8] = {-90.0f,-90.0f,-90.0f,-90.0f,-90.0f,-90.0f,-90.0f,-90.0f};
    float kp[8] = {100.0f,100.0f,100.0f,100.0f,100.0f,100.0f,100.0f,100.0f};
    float kd[8] = {50.0f,50.0f,50.0f,50.0f,50.0f,50.0f,50.0f,50.0f};
    float PP[8] = {FB_Datas[0].P,FB_Datas[1].P,FB_Datas[2].P,FB_Datas[3].P,FB_Datas[4].P,FB_Datas[5].P,FB_Datas[6].P,FB_Datas[7].P};
    float V[8]= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float Fmeas[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float Ferror[8] ={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float Vcmd[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    using clock = chrono::steady_clock;
    auto start_time = clock::now();
    /* ------------------- 反复握拳 ------------------- */
#if 1
    // File export
    // ofstream force_file("pretensioning21_1.txt");
    // if (!force_file.is_open()) {
    //     cout << "Failed to open file for writing." << endl;
    //     return -1;
    // }
    // force_file << "Tarposition(deg)\tposition(deg)\tCurrent(A)" << endl;
    // ReSharper disable once CppDFAEndlessLoop/
    // bool force_reached = false;
    while (true) {
        // Get_FB();
        // test += 0.0125f;
       
        // const float P = sin(test) * 2* 270.0f;
        
        // // // if(fabs(FB_Datas[1].F ) < 0.1f) {
        // // //     test += 0.0125f;
        // // // }
        // // // else{
        // // //     test = test;
        // // // }
        // // const float PP = test;
        // // // test += 1.0f;
        // const float V = (P - PP_old) * (static_cast<float>(1000000 / loop_time_step));
        //  PP_old = P;
        // if(fabs(FB_Datas[0].F ) > 0.25f) {
        //     force_reached = true;
        //     V = 0;
        //     test = test;
        //  }
        // if (!force_reached){
        //      test += 0.5f;
        //      V = VV;
        // }
        // quote pp above
        // const float PP = test;
        //  PP_old = PP;
        // test += 1.0f;
        // const float VV = (PP - PP_old) * (static_cast<float>(1000000 / loop_time_step));
        // Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, P , V, 0.0, 100.0, 50.0);
        //
        // if (test  > 300.0f) {
        //     test = 300.0f;
        // }
        // // code for pretensioning
        // Get_FB();
        // for (int i = 0; i < 1; i++){
        //     Fmeas[i] = fabs(FB_Datas[i].F);
        //     Ferror[i] = Ftar[i] - Fmeas[i];
        //     if (fabs(Ferror[i]) < CurDead[i]){
        //         Ferror[i] = 0.0f;
        //     }
        //     Vcmd[i] = Kcur[i] * Ferror[i]; 
        //     if (Vcmd[i] > Vmax[i]){
        //       Vcmd[i]  = Vmax[i];
        //     }
        //     else if (Vcmd[i]  < -Vmax[i]){
        //     Vcmd[i]  = -Vmax[i];
        //     }
        //     PP[i] += Vcmd[i] * (static_cast<float>(loop_time_step)/ 1000000.0f );
        //     if (PP[i] > Pmax[i]){
        //         PP[i] = Pmax[i];
        //     }
        //     else if (PP[i] < Pmin[i]){
        //      PP[i] = Pmin[i];
        //     }
        //     V[i] = Vcmd[i] ;
        //     // Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, p , v, 0.0, kp[1], kd[1]);
        //     // p = p+1.0f;
        //     // Motor_Control[i]->Send_MIT_PD_Control_Data(Motor_D[i],PP[i],V[i],0.0f,kp[i],kd[i]);
        // }

        // //end of pre tensioning code
         Get_FB();
        auto now = clock::now();
        double elapsed_time = chrono::duration<double>(now-start_time).count();
        if (elapsed_time >=3.0){
            break;
        }

        TaiHu_Device_T1 -> Send_MIT_PD_Control_Data(TaiHu_Device_1,1.0f,0.0f,0.0f,0.0f,0.0f);
        TaiHu_Device_T2 -> Send_MIT_PD_Control_Data(TaiHu_Device_2,-30.0f,0.0f,0.0f,0.0f,0.0f);
        TaiHu_Device_T3 -> Send_MIT_PD_Control_Data(TaiHu_Device_3,110.0f,0.0f,0.0f,0.0f,0.0f);
        TaiHu_Device_T8 -> Send_MIT_PD_Control_Data(TaiHu_Device_8,-20.0f,0.0f,0.0f,0.0f,0.0f);
        TaiHu_Device_T9 -> Send_MIT_PD_Control_Data(TaiHu_Device_9,20.0f,0.0f,0.0f,0.0f,0.0f);
        TaiHu_Device_T10 -> Send_MIT_PD_Control_Data(TaiHu_Device_10,110.0f,0.0f,0.0f,0.0f,0.0f);
        // TaiHu_Device_T1 -> Send_MIT_PD_Control_Data(TaiHu_Device_1, 0.0f,0.0f,300.0f,0.0f,0.0f);
        // TaiHu_Device_T2 -> Send_MIT_PD_Control_Data(TaiHu_Device_2, 0.0f,0.0f,300.0f,0.0f,0.0f);
        // TaiHu_Device_T3 -> Send_MIT_PD_Control_Data(TaiHu_Device_3, 0.0f,0.0f,300.0f,0.0f,0.0f);
        // TaiHu_Device_T8 -> Send_MIT_PD_Control_Data(TaiHu_Device_8, 0.0f,0.0f,300.0f,0.0f,0.0f);
        // TaiHu_Device_T9 -> Send_MIT_PD_Control_Data(TaiHu_Device_9, 0.0f,0.0f,300.0f,0.0f,0.0f);
        // TaiHu_Device_T10 -> Send_MIT_PD_Control_Data(TaiHu_Device_10, 0.0f,0.0f,300.0f,0.0f,0.0f);
        Y_bot->Send_Buff_Data();
        usleep(500000);
        Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, 0.0 , 0.0, 0.03f, 0.0, 0.0);
        Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, 0.0 , 0.0, 0.03f, 0.0, 0.0);
        Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, 0.0 , 0.0, 0.03f, 0.0, 0.0);
        Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, 0.0 , 0.0, 0.035f, 0.0, 0.0);
        Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, 0.0 , 0.0, 0.03f, 0.0, 0.0);
        Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, 0.0 , 0.0, 0.04f, 0.0, 0.0);
        Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, 0.0 , 0.0, 0.03f, 0.0, 0.0);
        Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, 0.0 , 0.0, 0.035f, 0.0, 0.0);

        // Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, 0.0 , 0.0, 0.0, 500.0, 100.0);
        // Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, 0.0 , 0.0, 0.0, 500.0, 100.0);

        // Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, 300.0f, 5.0f, 0.0, kp[0], kd[0]);
        // Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, 450.0f, 5.0f, 0.0, kp[0], kd[0]);
        // Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D,  0.0, 0.0, 0.0, kp[0], kd[0]);
        // Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, 0.0, 0.0, 0.0, kp[0], kd[0]);
        // Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D,  0.0, 0.0, 0.0, kp[0], kd[0]);
        // Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, 0.0, 0.0, 0.0, kp[0], kd[0]);
        // Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D,  0.0, 0.0, 0.0, kp[0], kd[0]);
        // Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D,  0.0, 0.0, 0.0, kp[0], kd[0]);


        // for (int i = 0; i < 1; i++) {
        //     Send_Datas[i].P =  test;
        //     Send_Datas[i].V = 0;
        //     Send_Datas[i].F = 0;
        //     Send_Datas[i].KP = 500;  // 800
        //     Send_Datas[i].KD = 10;   // 20
        // }
        // Send_Datas[0].P = 0.5f / 4 + test / 4;
        // Send_Datas[1].P = 0.5f / 2 + test / 2;
        // test++;
            // Send();
        Y_bot->Send_Buff_Data();

        usleep(loop_time_step);

        for(int i =0; i < 8 ; i++)
        {
            cout<< "Motor:" << i+1 << "Current:"<< FB_Datas[i].F <<"\t"<< "Position:" << FB_Datas[i].P <<"\t"<< elapsed_time << endl;
        }
        // Get_FB();

        // For export
        

        // for (int i = 0; i < 8; i++) {
        //     cout << "num: "<< i << PP[i] << " " << FB_Datas[i].P  <<" " << fabs(FB_Datas[i].F) << endl;
            // force_file << PP << " " << fabs(FB_Datas[i].P) << " " << fabs(FB_Datas[i].F) << endl;
        // }
        // float force_abs = fabs(FB_Datas[0].F);
        // float pos_abs = fabs(FB_Datas[0].P);
        // float force_abs2 = fabs(FB_Datas[1].F);
        // float pos_abs2 = fabs(FB_Datas[1].P);
        // force_file << PP << " " << pos_abs << " " << force_abs << " " << PP2 << " " << pos_abs2 << " " << force_abs2<< endl;
        // force_file << fixed << setprecision(8);
        // test = test + 10;

    }

#endif
#if 1
const float p[8] = {FB_Datas[0].P,FB_Datas[1].P,FB_Datas[2].P,FB_Datas[3].P,FB_Datas[4].P,FB_Datas[5].P,FB_Datas[6].P,FB_Datas[7].P};
    float thp[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float thv[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float thf[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    TaiHu_Device_T1 -> Get_Motor_FB_Data(TaiHu_Device_1, &thp[0],&thv[0],&thf[0]);
    TaiHu_Device_T2 -> Get_Motor_FB_Data(TaiHu_Device_2, &thp[1],&thv[1],&thf[1]);
    TaiHu_Device_T3 -> Get_Motor_FB_Data(TaiHu_Device_3, &thp[2],&thv[2],&thf[2]);
    TaiHu_Device_T8-> Get_Motor_FB_Data(TaiHu_Device_8, &thp[3],&thv[3],&thf[3]);
    TaiHu_Device_T9-> Get_Motor_FB_Data(TaiHu_Device_9, &thp[4],&thv[4],&thf[4]);
    TaiHu_Device_T10 -> Get_Motor_FB_Data(TaiHu_Device_10, &thp[5],&thv[5],&thf[5]);
 for (int i =0; i<6 ; i++){
    cout << i+1 << "\t" << thp[i] << endl;
 }
 while(true){

    Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, p[0] , 0.0, 0.0, 500.0, 100.0);
    Motor_2_Control->Send_MIT_PD_Control_Data(Motor_2_D, p[1] , 0.0, 0.0, 500.0, 100.0);
    Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, p[2] , 0.0, 0.0, 500.0, 100.0);
    Motor_4_Control->Send_MIT_PD_Control_Data(Motor_4_D, p[3] , 0.0, 0.0, 500.0, 100.0);
    Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, p[4] , 0.0, 0.0, 500.0, 100.0);
    Motor_6_Control->Send_MIT_PD_Control_Data(Motor_6_D, p[5] , 0.0, 0.0, 500.0, 100.0);
    Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, p[6] , 0.0, 0.0, 500.0, 100.0);
    Motor_8_Control->Send_MIT_PD_Control_Data(Motor_8_D, p[7] , 0.0, 0.0, 500.0, 100.0);
    Y_bot->Send_Buff_Data();
    // usleep(100000);
    // TaiHu_Device_T1 -> Send_MIT_PD_Control_Data(TaiHu_Device_1,thp[0],0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T2 -> Send_MIT_PD_Control_Data(TaiHu_Device_2,thp[1],0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T3 -> Send_MIT_PD_Control_Data(TaiHu_Device_3,thp[2],0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T8 -> Send_MIT_PD_Control_Data(TaiHu_Device_8,thp[3],0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T9 -> Send_MIT_PD_Control_Data(TaiHu_Device_9,thp[4],0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T10 -> Send_MIT_PD_Control_Data(TaiHu_Device_10,thp[5],0.0f,0.0f,0.0f,0.0f);
    // Y_bot->Send_Buff_Data();

    usleep(loop_time_step);
    //  for(int i =0; i < 8 ; i++)
    //     {
    //         cout<< "Motor:" << i+1 << "Current:"<< FB_Datas[i].F <<"\t"<< "Position:" << FB_Datas[i].P << endl;
    //     }
        break;
 }
#endif
//  Motor_vector.at(m)->Get_Motor_FB_Data(Device_vector.at(m), &p, &v, &c);
#if 0 // Hip Abduction
while(true){
    Get_FB();
    usleep(2000000);
    Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, p[0]+300.0f , 0.0, 0.0, 100.0, 50.0);
    // cout<< "Motor_1 Move to: " << FB_Datas[0].P << endl;
    Y_bot->Send_Buff_Data();
    usleep(2000000);
    Motor_1_Control->Send_MIT_PD_Control_Data(Motor_1_D, p[0], 0.0, 0.0, 100.0, 50.0);
    Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, p[4]+300.0f, 0.0, 0.0, 100.0, 50.0);
    Y_bot->Send_Buff_Data();
    usleep(2000000);
    Motor_5_Control->Send_MIT_PD_Control_Data(Motor_5_D, p[4], 0.0, 0.0, 100.0, 50.0);
    Y_bot->Send_Buff_Data();
    
}
#endif
#if 1 // Knee 
while(true){
    Get_FB();
    TaiHu_Device_T2 -> Send_MIT_PD_Control_Data(TaiHu_Device_2,-90.0f,0.0f,0.0f,0.0f,0.0f);
    TaiHu_Device_T9 -> Send_MIT_PD_Control_Data(TaiHu_Device_9,-40.0f,0.0f,0.0f,0.0f,0.0f);
    Y_bot->Send_Buff_Data();
    usleep(100000);
    Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, p[2]+400.0f , 0.0, 0.0, 100.0, 50.0);
    Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, p[6]+400.0f, 0.0, 0.0, 100.0, 50.0);
    Y_bot->Send_Buff_Data();
    usleep(3000000);
    Motor_3_Control->Send_MIT_PD_Control_Data(Motor_3_D, p[2] , 0.0, 0.0, 100.0, 50.0);
    Motor_7_Control->Send_MIT_PD_Control_Data(Motor_7_D, p[6], 0.0, 0.0, 100.0, 50.0);
    Y_bot->Send_Buff_Data();
    usleep(100000);
    TaiHu_Device_T2 -> Send_MIT_PD_Control_Data(TaiHu_Device_2,-30.0f,0.0f,0.0f,0.0f,0.0f);
    TaiHu_Device_T9 -> Send_MIT_PD_Control_Data(TaiHu_Device_9,20.0f,0.0f,0.0f,0.0f,0.0f);
    usleep(1000000);
}
#endif
#if 0
int j = 0;
    float thp[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float thv[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float thf[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
while(true){
    // TaiHu_Device_T1 -> Send_MIT_PD_Control_Data(TaiHu_Device_1,1.0f,0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T2 -> Send_MIT_PD_Control_Data(TaiHu_Device_2,-30.0f,0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T3 -> Send_MIT_PD_Control_Data(TaiHu_Device_3,110.0f,0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T8 -> Send_MIT_PD_Control_Data(TaiHu_Device_8,-20.0f,0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T9 -> Send_MIT_PD_Control_Data(TaiHu_Device_9,20.0f,0.0f,0.0f,0.0f,0.0f);
    // TaiHu_Device_T10 -> Send_MIT_PD_Control_Data(TaiHu_Device_10,110.0f,0.0f,0.0f,0.0f,0.0f);
    TaiHu_Device_T1 -> Get_Motor_FB_Data(TaiHu_Device_1, &thp[0],&thv[0],&thf[0]);
    TaiHu_Device_T2 -> Get_Motor_FB_Data(TaiHu_Device_2, &thp[1],&thv[1],&thf[1]);
    TaiHu_Device_T3 -> Get_Motor_FB_Data(TaiHu_Device_3, &thp[2],&thv[2],&thf[2]);
    TaiHu_Device_T8 -> Get_Motor_FB_Data(TaiHu_Device_8, &thp[3],&thv[3],&thf[3]);
    TaiHu_Device_T9 -> Get_Motor_FB_Data(TaiHu_Device_9, &thp[4],&thv[4],&thf[4]);
    TaiHu_Device_T10 -> Get_Motor_FB_Data(TaiHu_Device_10, &thp[5],&thv[5],&thf[5]);

for (int i =0; i<6; i++){
        
cout <<  "P"<< i+1 << "= " << thp[i] << "v"<< i+1 << "= " << thv[i] << "f"<< i+1 << "= " << thf[i]<<endl;
    }
    Y_bot -> Send_Buff_Data();
    usleep(100000);
   j++;
   if (j >= 100.0){
    break;
   }
}
//  TaiHu_Device_T1 -> Send_MIT_PD_Control_Data(TaiHu_Device_1,thp[0],0.0f,0.0f,0.0f,0.0f);
#endif

#if 0
while(true){
    Get_FB();

    for(int j = 0; j <300; ++j){
        TaiHu_Device_T1 ->Get_Motor_FB_Data(TaiHu_Device_1,&thp[0],&thv[0],&thf[0]);
        TaiHu_Device_T2 ->Get_Motor_FB_Data(TaiHu_Device_2,&thp[1],&thv[1],&thf[1]);
        TaiHu_Device_T3 ->Get_Motor_FB_Data(TaiHu_Device_3,&thp[2],&thv[2],&thf[2]);
        TaiHu_Device_T8 ->Get_Motor_FB_Data(TaiHu_Device_8,&thp[3],&thv[3],&thf[3]);
        TaiHu_Device_T9 ->Get_Motor_FB_Data(TaiHu_Device_9,&thp[4],&thv[4],&thf[4]);
        TaiHu_Device_T10 ->Get_Motor_FB_Data(TaiHu_Device_10,&thp[5],&thv[5],&thf[5]);
        Y_bot->Send_Buff_Data();

        for (int i = 0; i <6; i++){
            cout << fixed << setprecision(10)<<"TaiHu " << i+1 << ": Pos: " << thp[i] << "\t" << "Vel: " << thv[i] <<"\t"<< "Cur: " << thf[i] << "mz:"<<  FB_Datas[0].F<< endl;
        }
        usleep(1000000);
    }

        
    

    }
    #endif 

#endif

    return 0;
}
