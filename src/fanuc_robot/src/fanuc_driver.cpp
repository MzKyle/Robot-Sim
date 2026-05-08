//
// Created by huang on 2026/1/11.
//
#include <cmath>  // 替换math.h（C++标准头文件）
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/int32.hpp"
#include <mutex>
#include <unordered_set>
#include "weld_interface/msg/tcp_pos.hpp"
#include "weld_interface/msg/fanuc_robot_info.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "weld_interface/srv/special_speedl.hpp"
#include "weld_interface/srv/move.hpp"
#include "weld_interface/srv/fanuc_mov_rate.hpp"
#include "weld_interface/srv/read_fanuc_register.hpp"
#include "weld_interface/msg/weld_register_info.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <thread>  // 必须包含，用于获取线程ID
#include <exception>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "weld_interface/topic_configs.h"
#include "weld_interface/service_configs.h"

// 全局互斥锁
std::mutex mtx;
std::unordered_set<int> g_dynamic_register_items;

// 动态库加载头文件
#include <dlfcn.h>

namespace {

std::string resolve_share_path(const std::string& package_name, const std::string& relative_path)
{
    if (relative_path.empty() || relative_path.front() == '/') {
        return relative_path;
    }

    try {
        return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
    } catch (const std::exception&) {
        return relative_path;
    }
}

const std::string DEFAULT_FANUC_SO_PATH = resolve_share_path("fanuc_robot", "lib/libFanucRobot.so");

}  // namespace

// TODO: parameterize quitting distance in MovWeldLoopJogCallback

// 定义函数指针类型（保持原有逻辑）
typedef void (*HelloTest)();
typedef void (*InitRobotObj)(char* cIp,int nPort);
typedef void (*AddItem)(char* cName,char* cVarType,char* cAddr);
typedef bool (*Connect)();
typedef bool (*DisConnect)();
typedef bool (*IsConnected)();
typedef bool (*SetValueXyzwpr)(
        int Index,
        float X, float Y, float Z, float W, float P, float R, float E1, float E2, float E3,
        short C0, short C1, short C2, short C3, short C4, short C5, short C6,
        short UF, short UT);
typedef bool (*UpdateCache)(bool bForceUpdate);
typedef bool (*GetItemValue)(char* cName,char* pOut,int nOutBuffLen);
typedef bool (*SetValue)(int nIndex, short sValue);

// 全局函数指针（保持原有逻辑）
SetValueXyzwpr setValueXyzwpr = NULL;
HelloTest g_fnHelloTest = NULL;
InitRobotObj g_fnInitRobotObj = NULL;
AddItem g_fnAddItem = NULL;
Connect g_fnConnect = NULL;
DisConnect g_fnDisConnect = NULL;
IsConnected g_fnIsConnected = NULL;
SetValueXyzwpr g_fnSetValueXyzwpr = NULL;
UpdateCache g_fnUpdateCache = NULL;
GetItemValue g_fnGetItemValue = NULL;
SetValue g_fnSetValue = NULL;

// 寄存器索引宏定义（保持原有）
#define CONTROL_R_INDEX 205
#define E_STOP_R_INDEX 206
#define WELD_MOV_RATE_R_INDEX 207
#define ANY_MOV_RATE_R_INDEX 208

#define WELD_ID_R_INDEX 901
#define WELD_TYPE_R_INDEX 902
#define WELD_LAYER_R_INDEX 903

#define SAFE_START_PR_INDEX 301
#define SAFE_END_PR_INDEX 302
#define WELD_START_PR_INDEX 303
#define WELD_LOOP_PR_INDEX 304
#define ANY_PR_INDEX 305

// 控制标识枚举（保持原有）
typedef enum
{
    STOP = 0,
    GO_SAFE_START_POS = 1,
    GO_SAFE_END_POS,
    GO_WELD_START_POS,
    GO_WELD_LOOP_POS,
    GO_ANY_POS,
    GO_ANY_POS_LOOP
}CTR_SIGN;

bool GetControlRValue(int* nOut);
bool GetRegisterValue(int nIndex, int* nOut);

// -------------------------- 原有工具函数（仅适配ROS日志） --------------------------
bool InitFanucLibrary(void* handle)
{
    bool bResult = false;
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "InitFanucLibrary");  // ROS2日志
    if (handle)
    {
        g_fnHelloTest = (HelloTest)dlsym(handle, "HelloTest");
        g_fnInitRobotObj = (InitRobotObj)dlsym(handle, "InitRobotObj");
        g_fnAddItem = (AddItem)dlsym(handle, "AddItem");
        g_fnConnect = (Connect)dlsym(handle, "Connect");
        g_fnDisConnect = (DisConnect)dlsym(handle, "DisConnect");
        g_fnIsConnected = (IsConnected)dlsym(handle, "IsConnected");
        g_fnSetValueXyzwpr = (SetValueXyzwpr)dlsym(handle, "SetValueXyzwpr");
        g_fnUpdateCache = (UpdateCache)dlsym(handle, "UpdateCache");
        g_fnGetItemValue = (GetItemValue)dlsym(handle, "GetItemValue");
        g_fnSetValue = (SetValue)dlsym(handle, "SetValue");

        if (g_fnInitRobotObj != NULL && g_fnAddItem != NULL && g_fnConnect != NULL &&
            g_fnDisConnect != NULL && g_fnIsConnected != NULL && g_fnSetValueXyzwpr != NULL &&
            g_fnUpdateCache != NULL && g_fnGetItemValue != NULL && g_fnSetValue != NULL)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "InitFanucLibrary sucessed!");
            bResult = true;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "InitFanucLibrary falided!");
        }
    }
    return bResult;
}

bool GetValue(char* cKey, float& fOutValue)
{
    bool bResult = false;
    char pOut[256] = { 0 };
    if (g_fnGetItemValue(cKey, pOut, 256))
    {
        fOutValue = std::stof(pOut);
        bResult = true;
    }
    return bResult;
}

bool GetValue(char* cKey, int& nOutValue)
{
    bool bResult = false;
    char pOut[256] = { 0 };
    if (g_fnGetItemValue(cKey, pOut, 256))
    {
        nOutValue = std::stoi(pOut);
        bResult = true;
    }
    return bResult;
}

bool MovStop()
{
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "Set CONTROL_R stop!");
    bool bResult = g_fnSetValue(CONTROL_R_INDEX, STOP);
    return bResult;
}

bool WeldStop()
{
    bool bResult=false;
    int nValue = 0;
    if(GetControlRValue(&nValue))
    {
        if(nValue==GO_WELD_LOOP_POS)
        {
            bResult = g_fnSetValue(CONTROL_R_INDEX, STOP);
        }
    }
    return bResult;
}

bool EStop()
{
    bool bResult = g_fnSetValue(E_STOP_R_INDEX, 1);
    bResult = g_fnSetValue(CONTROL_R_INDEX, STOP);
    return bResult;
}

bool GetControlRValue(int* nOut)
{
    bool bResult = false;
    char pOut[256] = { 0 };
    if (g_fnGetItemValue("R_CONTROL", pOut, 256))
    {
        std::string val = pOut;
        *nOut = std::stoi(val);
        bResult = true;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "Get R failed!");
    }
    return bResult;
}

bool GetEStopRValue(int* nOut)
{
    bool bResult = false;
    char pOut[256] = { 0 };
    if (g_fnGetItemValue("R_E_STOP", pOut, 256))
    {
        std::string val = pOut;
        *nOut = std::stoi(val);
        bResult = true;
    }
    return bResult;
}

bool EnsureRegisterItemAddedLocked(int nIndex)
{
    if (nIndex <= 0 || g_fnAddItem == NULL)
    {
        return false;
    }

    if (g_dynamic_register_items.find(nIndex) != g_dynamic_register_items.end())
    {
        return true;
    }

    char alias[64] = { 0 };
    char addr[64] = { 0 };
    char var_type[] = "INT";
    snprintf(alias, sizeof(alias), "R_DYNAMIC_%d", nIndex);
    snprintf(addr, sizeof(addr), "NumReg(%d)", nIndex);
    g_fnAddItem(alias, var_type, addr);
    g_dynamic_register_items.insert(nIndex);
    return true;
}

bool GetRegisterValue(int nIndex, int* nOut)
{
    if (nOut == nullptr || nIndex <= 0)
    {
        return false;
    }

    if (g_fnAddItem == NULL || g_fnGetItemValue == NULL || g_fnUpdateCache == NULL)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx);
    if (!EnsureRegisterItemAddedLocked(nIndex))
    {
        return false;
    }

    g_fnUpdateCache(true);

    char alias[64] = { 0 };
    char pOut[256] = { 0 };
    snprintf(alias, sizeof(alias), "R_DYNAMIC_%d", nIndex);
    if (g_fnGetItemValue(alias, pOut, 256))
    {
        *nOut = std::stoi(pOut);
        return true;
    }

    RCLCPP_WARN(rclcpp::get_logger("robot_driver_fanuc"), "Failed to read register R[%d]", nIndex);
    return false;
}

// 机器人移动相关函数（保持原有逻辑，仅修改日志）
bool GoSafeStartPos(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "GoSafeStartPos");
    bool bResult;
    bResult = g_fnSetValueXyzwpr(SAFE_START_PR_INDEX, X, Y, Z, W, P, R, E1, E2, E3, C0, C1, C2, C3, C4, C5, C6, UF, UT);
    if (bResult){
        bResult = g_fnSetValue(CONTROL_R_INDEX, GO_SAFE_START_POS);
    }
    return bResult;
}

bool GoSafeEndPos(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 0, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    bool bResult;
    bResult = g_fnSetValueXyzwpr(SAFE_END_PR_INDEX, X, Y, Z, W, P, R, E1, E2, E3, C0, C1, C2, C3, C4, C5, C6, UF, UT);
    if (bResult){
        bResult = g_fnSetValue(CONTROL_R_INDEX, GO_SAFE_END_POS);
    }
    return bResult;
}

bool GoAnyPos(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    bool bResult = false;
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",X,Y,Z,W,P,R,E1);
    bResult = g_fnSetValueXyzwpr(ANY_PR_INDEX,
                                 X, Y, Z, W, P, R, E1, E2, E3,
                                 C0, C1, C2, C3, C4, C5, C6,
                                 UF, UT);
    if (bResult){
        bResult = g_fnSetValue(CONTROL_R_INDEX, GO_ANY_POS);
    }
    return bResult;
}

bool GoAnyPosLoopPositionSet(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    bool bResult = false;
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",X,Y,Z,W,P,R,E1);
    bResult = g_fnSetValueXyzwpr(ANY_PR_INDEX,
                                 X, Y, Z, W, P, R, E1, E2, E3,
                                 C0, C1, C2, C3, C4, C5, C6,
                                 UF, UT);
    return bResult;
}

bool GoAnyPosLoopSignSet(bool sign)
{
    bool result = false;
    if(sign){
        result = g_fnSetValue(CONTROL_R_INDEX, GO_ANY_POS_LOOP);
    }
    else{
        result = g_fnSetValue(CONTROL_R_INDEX, STOP);
    }
    return result;
}

bool GoWeldStartPos(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"),"GoWeldStartPos");
    bool bResult;
    bResult = g_fnSetValueXyzwpr(WELD_START_PR_INDEX, X, Y, Z, W, P, R, E1, E2, E3, C0, C1, C2, C3, C4, C5, C6, UF, UT);
    if (bResult){
        bResult = g_fnSetValue(CONTROL_R_INDEX, GO_WELD_START_POS);
        if(!bResult)
        {
            RCLCPP_ERROR(rclcpp::get_logger("robot_driver_fanuc"), "GoWeldStartPos set r value failed!");
        }
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("robot_driver_fanuc"), "GoWeldStartPos set pr value failed!");
    }
    return bResult;
}

bool GoWeldLoopPos(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    bool bResult;
    bResult = g_fnSetValueXyzwpr(WELD_LOOP_PR_INDEX, X, Y, Z, W, P, R, E1, E2, E3, C0, C1, C2, C3, C4, C5, C6, UF, UT);
    if (bResult){
        bResult = g_fnSetValue(CONTROL_R_INDEX, GO_WELD_LOOP_POS);
    }
    return bResult;
}

void AddAllItem()
{
    g_fnAddItem("MainPgm", "STRING", "MainProgrameName");
    g_fnAddItem("CurPgm", "STRING", "ProgrameName");
    g_fnAddItem("CurSeq", "INT", "ProgrameLineNumber");
    g_fnAddItem("NcStatus", "STRING","MacState");
    g_fnAddItem("Mode", "STRING","SO(7)");

    g_fnAddItem("Voltage1", "FLOAT", "SystemVar($AWEPOR[1].$VOLTS_FDBK)");
    g_fnAddItem("Current1", "FLOAT", "SystemVar($AWEPOR[1].$AMPS_FDBK)");
    g_fnAddItem("WireSpeed1", "FLOAT", "SystemVar($AWEPOR[1].$FDBK4)");
    g_fnAddItem("WeldDetect1", "INT", "SystemVar($AWEPOR[1].$ARC_DET_ON)");
    g_fnAddItem("Voltage2", "FLOAT", "SystemVar($AWEPOR[2].$VOLTS_FDBK)");
    g_fnAddItem("Current2", "FLOAT", "SystemVar($AWEPOR[2].$AMPS_FDBK)");
    g_fnAddItem("WireSpeed2", "FLOAT", "SystemVar($AWEPOR[2].$FDBK4)");
    g_fnAddItem("WeldDetect2", "INT", "SystemVar($AWEPOR[2].$ARC_DET_ON)");

    g_fnAddItem("Alarm", "INT", "AlarmState");
    g_fnAddItem("Emg","INT", "EStop");
    g_fnAddItem("Override", "INT", "SystemVar($MOR.$OVERRIDE)");
    g_fnAddItem("WeldEnable", "INT", "SystemVar($AWEPOR[1].$ARC_ENABLE)");

    g_fnAddItem("世界坐标X", "FLOAT", "/G1/XWorldPosition");
    g_fnAddItem("世界坐标Y", "FLOAT", "/G1/YWorldPosition");
    g_fnAddItem("世界坐标Z", "FLOAT", "/G1/ZWorldPosition");
    g_fnAddItem("世界坐标W", "FLOAT", "/G1/AWorldPosition");
    g_fnAddItem("世界坐标P", "FLOAT", "/G1/BWorldPosition");
    g_fnAddItem("世界坐标R", "FLOAT", "/G1/CWorldPosition");
    g_fnAddItem("世界坐标E1", "FLOAT", "/G1/DWorldPosition");
    g_fnAddItem("世界坐标E2", "FLOAT", "/G1/EWorldPosition");
    g_fnAddItem("世界坐标E3", "FLOAT", "/G1/FWorldPosition");

    char pBuf[0x100] = { 0 };
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", CONTROL_R_INDEX);
    g_fnAddItem("R_CONTROL", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", E_STOP_R_INDEX);
    g_fnAddItem("R_E_STOP", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", WELD_MOV_RATE_R_INDEX);
    g_fnAddItem("R_WELD_MOV_RATE", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", ANY_MOV_RATE_R_INDEX);
    g_fnAddItem("R_ANY_MOV_RATE", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", SAFE_START_PR_INDEX);
    g_fnAddItem("SAFE_START_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", SAFE_END_PR_INDEX);
    g_fnAddItem("SAFE_END_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", WELD_START_PR_INDEX);
    g_fnAddItem("WELD_START_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", WELD_LOOP_PR_INDEX);
    g_fnAddItem("WELD_LOOP_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", ANY_PR_INDEX);
    g_fnAddItem("ANY_PR", "FLOAT", pBuf);
}

// 获取机器人位姿（保持原有逻辑）
weld_interface::msg::TcpPos get_robot_pose(bool bForceUpdate = true)  // ROS2消息用msg命名空间
{
    weld_interface::msg::TcpPos res ;
    g_fnUpdateCache(bForceUpdate);

    char pOut[256]={0};
    if(g_fnGetItemValue("世界坐标X",pOut,256)){
        std::string val = pOut;
        res.x = std::stod(val)/1000.0;
    }
    if(g_fnGetItemValue("世界坐标Y",pOut,256)){
        std::string val = pOut;
        res.y = std::stod(val)/1000.0;
    }
    if(g_fnGetItemValue("世界坐标Z",pOut,256)){
        std::string val = pOut;
        res.z = std::stod(val)/1000.0;
    }
    if(g_fnGetItemValue("世界坐标W",pOut,256)){
        std::string val = pOut;
        res.rx = std::stod(val) / 180.0 * M_PI;  // 用M_PI替换3.14（标准常量）
    }
    if(g_fnGetItemValue("世界坐标P",pOut,256)){
        std::string val = pOut;
        res.ry = std::stod(val)/ 180.0 * M_PI;
    }
    if(g_fnGetItemValue("世界坐标R",pOut,256)){
        std::string val = pOut;
        res.rz = std::stod(val)/ 180.0 * M_PI;
    }
    if(g_fnGetItemValue("世界坐标E1",pOut,256)){
        std::string val = pOut;
        res.e1 = std::stod(val)/ 1000;
    }
    if(g_fnGetItemValue("世界坐标E2",pOut,256)){
        std::string val = pOut;
        res.e2 = std::stod(val)/ 1000;
    }
    if(g_fnGetItemValue("世界坐标E3",pOut,256)){
        std::string val = pOut;
        res.e3 = std::stod(val)/ 1000;
    }
    return res;
}

// 获取机器人信息（保持原有逻辑）
weld_interface::msg::FanucRobotInfo get_robot_info()
{
    weld_interface::msg::FanucRobotInfo info;
    char pOut[256]={0};
    if(g_fnGetItemValue("MainPgm",pOut,256)){
        info.main_pgm = pOut;
    }
    if(g_fnGetItemValue("CurPgm",pOut,256)){
        info.cur_pgm = pOut;
    }
    if(g_fnGetItemValue("CurSeq",pOut,256)){
        info.cur_seq = std::stoi(pOut);
    }
    if(g_fnGetItemValue("NcStatus",pOut,256)){
        info.ncstatus = pOut;
    }
    if(g_fnGetItemValue("Mode",pOut,256)){
        info.mode = pOut;
    }

    if(g_fnGetItemValue("Voltage1",pOut,256)){
        info.voltage1 = std::stof(pOut);
    }
    if(g_fnGetItemValue("Current1",pOut,256)){
        info.current1 = std::stof(pOut);
    }
    if(g_fnGetItemValue("WireSpeed1",pOut,256)){
        info.wire_speed1 = std::stof(pOut);
    }
    if(g_fnGetItemValue("WeldDetect1",pOut,256)){
        info.weld_detect1 = std::stoi(pOut);
    }

    if(g_fnGetItemValue("Voltage2",pOut,256)){
        info.voltage2 = std::stof(pOut);
    }
    if(g_fnGetItemValue("Current2",pOut,256)){
        info.current2 = std::stof(pOut);
    }
    if(g_fnGetItemValue("WireSpeed2",pOut,256)){
        info.wire_speed2 = std::stof(pOut);
    }
    if(g_fnGetItemValue("WeldDetect2",pOut,256)){
        info.weld_detect2 = std::stoi(pOut);
    }

    if(g_fnGetItemValue("Alarm",pOut,256)){
        info.alarm = std::stoi(pOut);
    }
    if(g_fnGetItemValue("Emg",pOut,256)){
        info.emg = std::stoi(pOut);
    }
    if(g_fnGetItemValue("Override",pOut,256)){
        info.override = std::stoi(pOut);
    }
    if(g_fnGetItemValue("WeldEnable",pOut,256)){
        info.weld_enable = std::stoi(pOut);
    }
    return info;
}

// -------------------------- ROS2节点类（核心适配） --------------------------
class FanucRobotDriver : public rclcpp::Node
{
private:
    std::thread timer_thread_;

    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
public:
    FanucRobotDriver() : Node("robot_driver_fanuc"), tf_broadcaster_(this)  // 初始化TF广播器
    {
        // 初始化Fanuc库连接
        if (!FanucInterfaceInit())
        {
            //RCLCPP_ERROR(this->get_logger(), "Fanuc interface init failed!");
            rclcpp::shutdown();
            return;
        }
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&FanucRobotDriver::update_runtime_parameters, this, std::placeholders::_1));
        // 创建回调组（Reentrant模式支持并行处理）
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 1. 创建话题发布者
        pos_pub_ = this->create_publisher<weld_interface::msg::TcpPos>(TCP_PUBLISH_TOPIC_NAME, rclcpp::QoS(10));  // QoS默认10
        robot_info_pub_ = this->create_publisher<weld_interface::msg::FanucRobotInfo>(FANUC_ROBOT_INFO_TOPIC_NAME, rclcpp::QoS(10));
        target_register_value_pub_ = this->create_publisher<std_msgs::msg::Int32>(FANUC_TARGET_REGISTER_VALUE_TOPIC_NAME, rclcpp::QoS(10));
        weld_register_info_pub_ = this->create_publisher<weld_interface::msg::WeldRegisterInfo>(FANUC_WELD_REGISTER_INFO_TOPIC_NAME, rclcpp::QoS(10));

        // 2. 创建服务端（ROS2服务回调函数格式：std::function）
        move_safe_start_jog_service_ = this->create_service<weld_interface::srv::Move>(
                SAFE_START_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovSafeStartJogCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_end_start_jog_service_ = this->create_service<weld_interface::srv::Move>(
                SAFE_END_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovSafeEndJogCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_any_jog_service_ = this->create_service<weld_interface::srv::SpecialSpeedl>(
                ANY_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovAnyJogCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_any_offset_service_ = this->create_service<weld_interface::srv::SpecialSpeedl>(
                ANY_MOV_OFFSET_SRV_NAME,
                std::bind(&FanucRobotDriver::MovAnyOffsetCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_weld_start_jog_service_ = this->create_service<weld_interface::srv::SpecialSpeedl>(
                WELD_START_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovWeldStartJogCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_weld_loop_jog_service_ = this->create_service<weld_interface::srv::SpecialSpeedl>(
                WELD_LOOP_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovWeldLoopJogCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_stop_service_ = this->create_service<std_srvs::srv::Empty>(
                STOP_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovJogStopCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        move_e_stop_service_ = this->create_service<std_srvs::srv::Empty>(
                E_STOP_MOV_JOG_SRV_NAME,
                std::bind(&FanucRobotDriver::MovJogEStopCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        weld_stop_service_ = this->create_service<std_srvs::srv::Empty>(
                WELD_LOOP_STOP_SRV_NAME,
                std::bind(&FanucRobotDriver::MovWeldStopCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        weld_loop_set_rate_service_ = this->create_service<weld_interface::srv::FanucMovRate>(
                WELD_LOOP_RATE_SET_SRV_NAME,
                std::bind(&FanucRobotDriver::SetWeldLoopRateCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        any_move_loop_set_rate_service_ = this->create_service<weld_interface::srv::FanucMovRate>(
                ANY_MOV_LOOP_RATE_SET_SRV_NAME,
                std::bind(&FanucRobotDriver::SetAnyMoveLoopRateCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        fanuc_register_read_service_ = this->create_service<weld_interface::srv::ReadFanucRegister>(
                FANUC_REGISTER_READ_SRV_NAME,
                std::bind(&FanucRobotDriver::ReadFanucRegisterCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        mov_any_jog_loop_position_service_ = this->create_service<weld_interface::srv::SpecialSpeedl>(
                ANY_MOV_LOOP_POSITION_SET_SRV_NAME,
                std::bind(&FanucRobotDriver::MovAnyLoopJogCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        mov_any_jog_loop_sign_service_ = this->create_service<std_srvs::srv::SetBool>(
                ANY_MOV_LOOP_SIGN_SET_SRV_NAME,
                std::bind(&FanucRobotDriver::MovAnyLoopJogSetSignCallback, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_default,
                service_callback_group_
                );

        // 3. 创建定时器（替代ROS1的while循环发布话题）
        timer_thread_ = std::thread(&FanucRobotDriver::timerThreadLoop, this);

//        publish_timer_ = this->create_wall_timer(
//                std::chrono::milliseconds(33),  // 30Hz = 33ms
//                std::bind(&FanucRobotDriver::publish_robot_data, this));

        RCLCPP_INFO(this->get_logger(), "Fanuc robot driver node initialized!");
    }

    ~FanucRobotDriver()
    {
        // 析构时断开机器人连接
        if (g_fnDisConnect)
        {
            g_fnDisConnect();
            RCLCPP_INFO(this->get_logger(), "Disconnected from Fanuc robot!");
        }
    }

    // Fanuc库初始化（封装到类内）
    bool FanucInterfaceInit()
    {
        std::string so_path_;
        // 获取参数（ROS2方式）
        this->declare_parameter<std::string>("so_file_path", DEFAULT_FANUC_SO_PATH); // 默认动态库路径
        this->get_parameter("so_file_path", so_path_);
        so_path_ = resolve_share_path("fanuc_robot", so_path_);
        RCLCPP_INFO(this->get_logger(), "So path is: %s", so_path_.c_str());

        std::string robot_ip_;
        this->declare_parameter<std::string>("robot_ip", "10.16.141.114"); // 默认动态库路径
        this->get_parameter("robot_ip", robot_ip_);
        RCLCPP_INFO(this->get_logger(), "Robot ip is: %s", robot_ip_.c_str());

        int robot_port_ = 60008;
        this->declare_parameter("robot_port", 60008);
        this->get_parameter("robot_port", robot_port_);
        RCLCPP_INFO(this->get_logger(), "robot port is: %d", robot_port_);

        this->declare_parameter("target_register_index", 100);
        this->get_parameter("target_register_index", target_register_index_);
        RCLCPP_INFO(this->get_logger(), "target register index is: %d", target_register_index_);

        bool bResult = false;
        void* handle = dlopen(so_path_.c_str(), RTLD_LAZY);
        if (handle)
        {
            if (::InitFanucLibrary(handle))  // 调用全局InitFanucLibrary
            {
                g_fnInitRobotObj((char *)robot_ip_.c_str(), robot_port_);
                ::AddAllItem();  // 调用全局AddAllItem
                if (g_fnConnect())
                {
                    bResult = true;
                    RCLCPP_INFO(this->get_logger(), "Connected to Fanuc robot successfully!");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Fanuc robot!");
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load Fanuc library: %s", dlerror());
        }
        return bResult;
    }

private:
    // 话题发布者
    rclcpp::Publisher<weld_interface::msg::TcpPos>::SharedPtr pos_pub_;
    rclcpp::Publisher<weld_interface::msg::FanucRobotInfo>::SharedPtr robot_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_register_value_pub_;
    rclcpp::Publisher<weld_interface::msg::WeldRegisterInfo>::SharedPtr weld_register_info_pub_;

    // TF广播器（ROS2）
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 服务端
    rclcpp::Service<weld_interface::srv::Move>::SharedPtr move_safe_start_jog_service_;
    rclcpp::Service<weld_interface::srv::Move>::SharedPtr move_end_start_jog_service_;
    rclcpp::Service<weld_interface::srv::SpecialSpeedl>::SharedPtr move_any_jog_service_;
    rclcpp::Service<weld_interface::srv::SpecialSpeedl>::SharedPtr move_any_offset_service_;
    rclcpp::Service<weld_interface::srv::SpecialSpeedl>::SharedPtr move_weld_start_jog_service_;
    rclcpp::Service<weld_interface::srv::SpecialSpeedl>::SharedPtr move_weld_loop_jog_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_stop_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_e_stop_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr weld_stop_service_;
    rclcpp::Service<weld_interface::srv::FanucMovRate>::SharedPtr weld_loop_set_rate_service_;
    rclcpp::Service<weld_interface::srv::FanucMovRate>::SharedPtr any_move_loop_set_rate_service_;
    rclcpp::Service<weld_interface::srv::ReadFanucRegister>::SharedPtr fanuc_register_read_service_;
    rclcpp::Service<weld_interface::srv::SpecialSpeedl>::SharedPtr mov_any_jog_loop_position_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mov_any_jog_loop_sign_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    // 定时器（用于循环发布数据）
    rclcpp::TimerBase::SharedPtr publish_timer_;
    int target_register_index_{100};

    // 独立线程的循环函数（核心：精准定时+调用回调）
    void timerThreadLoop() {
        // 循环条件：ROS2未退出 + 线程运行标志为true
        while (rclcpp::ok() ) {
            // ========== 调用原有timerCallback逻辑 ==========
            this->publish_robot_data();

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // 线程退出日志
        RCLCPP_INFO(this->get_logger(), "独立定时器线程退出");
    }

    // 发布机器人数据（位姿、信息、TF）
    void publish_robot_data()
    {
        // 1. 发布位姿话题
        auto pose_msg = ::get_robot_pose(false);  // 调用全局get_robot_pose
        pos_pub_->publish(pose_msg);

        // 2. 发布机器人信息话题
        auto robot_info = ::get_robot_info();  // 调用全局get_robot_info
        robot_info_pub_->publish(robot_info);

        publish_target_register_value();
        publish_weld_register_info();

        // 3. 发布TF变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base";
        transform_stamped.child_frame_id = "tcp";

        transform_stamped.transform.translation.x = pose_msg.x;
        transform_stamped.transform.translation.y = pose_msg.y;
        transform_stamped.transform.translation.z = pose_msg.z;

        tf2::Quaternion quat;
        quat.setRPY(pose_msg.rx, pose_msg.ry, pose_msg.rz);
        transform_stamped.transform.rotation.x = quat.x();
        transform_stamped.transform.rotation.y = quat.y();
        transform_stamped.transform.rotation.z = quat.z();
        transform_stamped.transform.rotation.w = quat.w();

        tf_broadcaster_.sendTransform(transform_stamped);
    }

    void publish_target_register_value()
    {
        if (target_register_index_ <= 0 || target_register_value_pub_ == nullptr)
        {
            return;
        }

        int register_value = 0;
        if (::GetRegisterValue(target_register_index_, &register_value))
        {
            std_msgs::msg::Int32 msg;
            msg.data = register_value;
            target_register_value_pub_->publish(msg);
        }
    }

    void publish_weld_register_info()
    {
        if (weld_register_info_pub_ == nullptr)
        {
            return;
        }

        weld_interface::msg::WeldRegisterInfo msg;
        int value = 0;

        if (::GetRegisterValue(WELD_ID_R_INDEX, &value))
        {
            msg.weld_id = value;
        }

        if (::GetRegisterValue(WELD_TYPE_R_INDEX, &value))
        {
            msg.weld_type = value;
        }

        if (::GetRegisterValue(WELD_LAYER_R_INDEX, &value))
        {
            msg.weld_layer = value;
        }

        weld_register_info_pub_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult update_runtime_parameters(
            const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
            const std::string& name = parameter.get_name();
            if (name == "target_register_index")
            {
                target_register_index_ = static_cast<int>(parameter.as_int());
                RCLCPP_INFO(this->get_logger(), "target register index updated: %d", target_register_index_);
            }
            else if (name == "so_file_path" || name == "robot_ip" || name == "robot_port")
            {
                result.successful = false;
                result.reason = "Fanuc connection parameters require restarting robot_driver_fanuc";
                return result;
            }
        }

        return result;
    }

    // -------------------------- 服务回调函数（适配ROS2格式） --------------------------
    void MovSafeStartJogCallback(
            const std::shared_ptr<weld_interface::srv::Move::Request> req,
            std::shared_ptr<weld_interface::srv::Move::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovSafeStartJogCallback");
        if (::GoSafeStartPos(req->a * 1000, req->b * 1000, req->c * 1000,
                             req->d / M_PI * 180, req->e / M_PI * 180, req->f / M_PI * 180))
        {
            rclcpp::Rate loop_rate(30);
            while (rclcpp::ok())
            {
                auto tool_pose = ::get_robot_pose();

                if (fabs(tool_pose.x - req->a) < 0.005 && fabs(tool_pose.y - req->b) < 0.005 && fabs(tool_pose.z - req->c) < 0.005
                    && fabs(tool_pose.rx - req->d) < 0.005 && fabs(tool_pose.ry - req->e) < 0.005 && fabs(tool_pose.rz - req->f) < 0.005)
                {
                    RCLCPP_INFO(this->get_logger(), "req x : %.3lf  current x : %.3lf", req->a, tool_pose.x);
                    break;
                }
                int nValue = 0;
                if(::GetControlRValue(&nValue) == false || nValue == STOP)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd stop!");
                    break;
                }
                loop_rate.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "run done!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MovSafeStartJogCallback failed!");
        }
    }

    void MovSafeEndJogCallback(
            const std::shared_ptr<weld_interface::srv::Move::Request> req,
            std::shared_ptr<weld_interface::srv::Move::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovSafeEndJogCallback");
        if (::GoSafeEndPos(req->a * 1000, req->b * 1000, req->c * 1000,
                           req->d / M_PI * 180, req->e / M_PI * 180, req->f / M_PI * 180))
        {
            rclcpp::Rate loop_rate(30);
            while (rclcpp::ok())
            {
                auto tool_pose = ::get_robot_pose();

                if (fabs(tool_pose.x - req->a) < 0.005 && fabs(tool_pose.y - req->b) < 0.005 && fabs(tool_pose.z - req->c) < 0.005
                    && fabs(tool_pose.rx - req->d) < 0.005 && fabs(tool_pose.ry - req->e) < 0.005 && fabs(tool_pose.rz - req->f) < 0.005)
                {
                    RCLCPP_INFO(this->get_logger(), "req x : %.3lf  current x : %.3lf", req->a, tool_pose.x);
                    break;
                }
                int nValue = 0;
                if(::GetControlRValue(&nValue) == false || nValue == STOP)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd stop!");
                    break;
                }
                loop_rate.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "run done!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MovSafeEndJogCallback failed!");
        }
    }

    void MovAnyJogCallback(
            const std::shared_ptr<weld_interface::srv::SpecialSpeedl::Request> req,
            std::shared_ptr<weld_interface::srv::SpecialSpeedl::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovAnyJogCallback");

        auto tool_pose = ::get_robot_pose();
        RCLCPP_INFO(this->get_logger(), "TOOL POS %.3f %.3f %.3f %.3f", req->rx,req->ry,req->rz,req->e1);
        RCLCPP_INFO(this->get_logger(), "TOOL POS %.3f %.3f %.3f %.3f", tool_pose.rx,tool_pose.ry,tool_pose.rz,tool_pose.e1);

        if (::GoAnyPos(
                req->x * 1000, req->y * 1000, req->z * 1000,
                req->rx/ M_PI * 180, req->ry / M_PI * 180, req->rz / M_PI * 180,
                req->e1 * 1000, tool_pose.e2 * 1000, tool_pose.e3 * 1000,
                0, 0, 1, 1, 0, 0, 0,
                255, 255))
        {
            rclcpp::Rate loop_rate(30);
            while (rclcpp::ok())
            {
                tool_pose = ::get_robot_pose();
                float fX = fabs(tool_pose.x - req->x);
                float fY = fabs(tool_pose.y - req->y);
                float fZ = fabs(tool_pose.z - req->z);
                float fDist = sqrt(fX * fX + fY * fY + fZ * fZ);
                if (fDist < req->quit_distance)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd done!");
                    break;
                }
                int nValue = 0;
                if(::GetControlRValue(&nValue) == false || nValue == STOP)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd stop!");
                    break;
                }
                loop_rate.sleep();
            }
        }
        res->success = true;
    }

    void MovAnyLoopJogCallback(
            const std::shared_ptr<weld_interface::srv::SpecialSpeedl::Request> req,
            std::shared_ptr<weld_interface::srv::SpecialSpeedl::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovAnyLoopJogCallback");
        bool result = ::GoAnyPosLoopPositionSet(
                req->x * 1000, req->y * 1000, req->z * 1000,
                req->rx/ M_PI * 180, req->ry / M_PI * 180, req->rz / M_PI * 180,
                req->e1 * 1000, 0, 0,
                0, 0, 1, 1, 0, 0, 0,
                255, 255);
        if(result)
        {
            RCLCPP_INFO(this->get_logger(), "MovAnyLoopJogCallback DONE!");
        }
    }

    void MovAnyLoopJogSetSignCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
        bool result = ::GoAnyPosLoopSignSet(req->data);
        res->success = result;
        if (result)
        {
            res->message = "Loop sign set successfully!";
        }
        else
        {
            res->message = "Failed to set loop sign!";
        }
    }

    void MovAnyOffsetCallback(
            const std::shared_ptr<weld_interface::srv::SpecialSpeedl::Request> req,
            std::shared_ptr<weld_interface::srv::SpecialSpeedl::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovAnyOffsetCallback");
        auto tool_pose = ::get_robot_pose();
        if (::GoAnyPos(
                (tool_pose.x + req->x) * 1000, (tool_pose.y + req->y) * 1000, (tool_pose.z + req->z) * 1000,
                (tool_pose.rx + req->rx)/ M_PI * 180, (tool_pose.ry + req->ry) / M_PI * 180, (tool_pose.rz + req->rz) / M_PI * 180,
                (tool_pose.e1 + req->e1) * 1000, 0, 0,
                0, 0, 1, 1, 0, 0, 0,
                255, 255))
        {
            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok())
            {
                int nValue = 0;
                if(::GetControlRValue(&nValue) == false || nValue == STOP)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd stop!");
                    break;
                }
                loop_rate.sleep();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "MovAnyOffsetCallback failed!");
        }
        res->success = true;
    }

    void MovWeldStartJogCallback(
            const std::shared_ptr<weld_interface::srv::SpecialSpeedl::Request> req,
            std::shared_ptr<weld_interface::srv::SpecialSpeedl::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovWeldStartJogCallback");
        auto tool_pose = ::get_robot_pose();
        double roll, pitch, yaw;
        roll = tool_pose.rx;
        pitch = tool_pose.ry;
        yaw = tool_pose.rz;

        double lfX,lfY,lfZ;
        lfX = req->x + tool_pose.x;
        lfY = req->y + tool_pose.y;
        lfZ = req->z + tool_pose.z;
        float fE1Dst = tool_pose.e1 + req->y;

        if (::GoWeldStartPos(
                lfX * 1000, lfY* 1000, lfZ* 1000,
                roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180,
                fE1Dst * 1000, tool_pose.e2 * 1000, tool_pose.e3 * 1000,
                0, 0, 1, 1, 0, 0, 0,
                255, 255))
        {
            RCLCPP_INFO(this->get_logger(), "cur x : %.3lf y : %.3lf z : %.3lf rx : %.3lf ry : %.3lf rz : %.3lf",
                        tool_pose.x*1000, tool_pose.y*1000, tool_pose.z*1000,
                        roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180);
            RCLCPP_INFO(this->get_logger(), "dst x : %.3lf y : %.3lf z : %.3lf rx : %.3lf ry : %.3lf rz : %.3lf",
                        lfX * 1000, lfY* 1000, lfZ* 1000,
                        roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180);

            rclcpp::Rate loop_rate(20);
            while (rclcpp::ok())
            {
                tool_pose = ::get_robot_pose();
                float fX = fabs(tool_pose.x - lfX);
                float fY = fabs(tool_pose.y - lfY);
                float fZ = fabs(tool_pose.z - lfZ);
                float fDist = sqrt(fX * fX + fY * fY + fZ * fZ);
                if (fDist < req->quit_distance)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd done!");
                    break;
                }
                int nValue = 0;
                if(::GetControlRValue(&nValue) == false || nValue == STOP)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd stop!");
                    break;
                }
                loop_rate.sleep();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GoWeldStartPos failed!");
        }
        usleep(20*1000);
        res->success = true;
    }

    void MovJogStopCallback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovJogStopCallback");
        ::MovStop();
    }

    void MovWeldStopCallback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovWeldStopCallback");
        ::WeldStop();
    }

    void SetWeldLoopRateCallback(
            const std::shared_ptr<weld_interface::srv::FanucMovRate::Request> req,
            std::shared_ptr<weld_interface::srv::FanucMovRate::Response> res)
    {
        bool result = false;
        RCLCPP_INFO(this->get_logger(), "SetWeldLoopRateCallback");
        if(g_fnSetValue(WELD_MOV_RATE_R_INDEX, req->rate))
        {
            RCLCPP_INFO(this->get_logger(), "Set weld loop rate succeeded!");
            result = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Set weld loop rate failed!");
        }
    }

    bool SetAnyMoveLoopRate(int rate)
    {
        bool result = false;
        RCLCPP_INFO(this->get_logger(), "SetAnyMoveLoopRate");
        if(g_fnSetValue(ANY_MOV_RATE_R_INDEX, rate))
        {
            RCLCPP_INFO(this->get_logger(), "Set any move loop rate succeeded!");
            result = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Set any move loop rate failed!");
        }
        return result;
    }

    void SetAnyMoveLoopRateCallback(
            const std::shared_ptr<weld_interface::srv::FanucMovRate::Request> req,
            std::shared_ptr<weld_interface::srv::FanucMovRate::Response> res)
    {
        SetAnyMoveLoopRate(req->rate);
    }

    void MovJogEStopCallback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovJogEStopCallback");
        ::EStop();
    }

    void ReadFanucRegisterCallback(
            const std::shared_ptr<weld_interface::srv::ReadFanucRegister::Request> req,
            std::shared_ptr<weld_interface::srv::ReadFanucRegister::Response> res)
    {
        if (req->register_index <= 0)
        {
            res->success = false;
            res->value = 0;
            res->message = "register_index must be positive";
            return;
        }

        int value = 0;
        res->success = ::GetRegisterValue(req->register_index, &value);
        res->value = value;
        if (res->success)
        {
            res->message = "ok";
            RCLCPP_INFO(this->get_logger(), "Read R[%d] = %d", req->register_index, value);
        }
        else
        {
            res->message = "failed to read register";
            RCLCPP_WARN(this->get_logger(), "Failed to read R[%d]", req->register_index);
        }
    }

    void MovWeldLoopJogCallback(
            const std::shared_ptr<weld_interface::srv::SpecialSpeedl::Request> req,
            std::shared_ptr<weld_interface::srv::SpecialSpeedl::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "MovWeldLoopJogCallback");
        auto tool_pose = ::get_robot_pose();

        double roll, pitch, yaw;
        roll = tool_pose.rx;
        pitch = tool_pose.ry;
        yaw = tool_pose.rz;

        double lfX,lfY,lfZ;
        lfX = req->x + tool_pose.x;
        lfY = req->y + tool_pose.y;
        lfZ = req->z + tool_pose.z;
        float fE1Dst = tool_pose.e1 + req->y;

        if (::GoWeldLoopPos(
                lfX * 1000, lfY* 1000, lfZ* 1000,
                roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180,
                fE1Dst * 1000, tool_pose.e2 * 1000, tool_pose.e3 * 1000,
                0, 0, 1, 1, 0, 0, 0,
                255, 255))
        {
            rclcpp::Rate loop_rate(20);
            while (rclcpp::ok())
            {
                auto tool_pose = ::get_robot_pose();

                float fX = fabs(tool_pose.x - lfX);
                float fY = fabs(tool_pose.y - lfY);
                float fZ = fabs(tool_pose.z - lfZ);
                float fDist = sqrt(fX * fX + fY * fY + fZ * fZ);
                if (fDist < req->quit_distance)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd done!");
                    break;
                }
                int nValue = 0;
                if(::GetControlRValue(&nValue) == false || nValue == STOP)
                {
                    RCLCPP_INFO(this->get_logger(), "Cmd stop!");
                    break;
                }
                loop_rate.sleep();
            }
        }
        res->success = true;
    }
};

// 主函数（ROS2标准入口）
int main(int argc, char * argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点实例
    auto node = std::make_shared<FanucRobotDriver>();

    // 创建多线程执行器（替代ROS1的AsyncSpinner）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);  // 4线程
    executor.add_node(node);

    // 运行执行器
    executor.spin();

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}
