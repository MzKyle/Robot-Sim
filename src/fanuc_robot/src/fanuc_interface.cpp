//
// Created by huang on 2026/1/8.
//
#include "rclcpp/rclcpp.hpp"
#include "fanuc_interface.h"
// 动态库加载头文件
#include <dlfcn.h>
#include "weld_interface/srv/special_speedl.hpp"
#include "std_srvs/srv/empty.hpp"

using weld_interface::msg::TcpPos;

FanucInterface::FanucInterface(char ip[0x20], int port)
{
    memcpy(_ip,ip,sizeof(ip));
    _port = port;
    _fn_init_robot_obj = nullptr;
    _fn_add_item = nullptr;
    _fn_connect = nullptr;
    _fn_disconnect = nullptr;
    _fn_is_connected = nullptr;
    _fn_set_value_xyzwpr = nullptr;
    _fn_update_cache = nullptr;
    _fn_get_item_value = nullptr;
    _fn_set_value = nullptr;
}


FanucInterface::~FanucInterface()
{

}

bool FanucInterface::InitLibraryFunction(){
    bool result = false;
    void* handle = dlopen("/home/rootlink/catkin_ws/src/robot_control/lib/libFanucRobot.so", RTLD_LAZY);
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "InitFanucLibrary");  // ROS2日志
    if (handle){
        _fn_init_robot_obj = (InitRobotObj)dlsym(handle, "InitRobotObj");
        _fn_add_item = (AddItem)dlsym(handle, "AddItem");
        _fn_connect = (Connect)dlsym(handle, "Connect");
        _fn_disconnect = (DisConnect)dlsym(handle, "DisConnect");
        _fn_is_connected = (IsConnected)dlsym(handle, "IsConnected");
        _fn_set_value_xyzwpr = (SetValueXyzwpr)dlsym(handle, "SetValueXyzwpr");
        _fn_update_cache = (UpdateCache)dlsym(handle, "UpdateCache");
        _fn_get_item_value = (GetItemValue)dlsym(handle, "GetItemValue");
        _fn_set_value = (SetValue)dlsym(handle, "SetValue");
        if (_fn_init_robot_obj != NULL && _fn_add_item != NULL && _fn_connect != NULL &&
                _fn_disconnect != NULL && _fn_is_connected != NULL && _fn_set_value_xyzwpr != NULL &&
                _fn_update_cache != NULL && _fn_get_item_value != NULL && _fn_set_value != NULL){
            RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "InitFanucLibrary sucessed!");
            result = true;
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "InitFanucLibrary falided!");
        }
    }
    else{

    }
    return result;
}

void FanucInterface::RobotAddAllItem() {
    assert(_fn_add_item != nullptr);
    _fn_add_item("MainPgm", "STRING", "MainProgrameName");
    _fn_add_item("CurPgm", "STRING", "ProgrameName");
    _fn_add_item("CurSeq", "INT", "ProgrameLineNumber");
    _fn_add_item("NcStatus", "STRING","MacState");
    _fn_add_item("Mode", "STRING","SO(7)");

    _fn_add_item("Voltage1", "FLOAT", "SystemVar($AWEPOR[1].$VOLTS_FDBK)");
    _fn_add_item("Current1", "FLOAT", "SystemVar($AWEPOR[1].$AMPS_FDBK)");
    _fn_add_item("WireSpeed1", "FLOAT", "SystemVar($AWEPOR[1].$FDBK4)");
    _fn_add_item("WeldDetect1", "INT", "SystemVar($AWEPOR[1].$ARC_DET_ON)");
    _fn_add_item("Voltage2", "FLOAT", "SystemVar($AWEPOR[2].$VOLTS_FDBK)");
    _fn_add_item("Current2", "FLOAT", "SystemVar($AWEPOR[2].$AMPS_FDBK)");
    _fn_add_item("WireSpeed2", "FLOAT", "SystemVar($AWEPOR[2].$FDBK4)");
    _fn_add_item("WeldDetect2", "INT", "SystemVar($AWEPOR[2].$ARC_DET_ON)");

    _fn_add_item("Alarm", "INT", "AlarmState");
    _fn_add_item("Emg","INT", "EStop");
    _fn_add_item("Override", "INT", "SystemVar($MOR.$OVERRIDE)");
    _fn_add_item("WeldEnable", "INT", "SystemVar($AWEPOR[1].$ARC_ENABLE)");

    _fn_add_item("世界坐标X", "FLOAT", "/G1/XWorldPosition");
    _fn_add_item("世界坐标Y", "FLOAT", "/G1/YWorldPosition");
    _fn_add_item("世界坐标Z", "FLOAT", "/G1/ZWorldPosition");
    _fn_add_item("世界坐标W", "FLOAT", "/G1/AWorldPosition");
    _fn_add_item("世界坐标P", "FLOAT", "/G1/BWorldPosition");
    _fn_add_item("世界坐标R", "FLOAT", "/G1/CWorldPosition");
    _fn_add_item("世界坐标E1", "FLOAT", "/G1/DWorldPosition");
    _fn_add_item("世界坐标E2", "FLOAT", "/G1/EWorldPosition");
    _fn_add_item("世界坐标E3", "FLOAT", "/G1/FWorldPosition");

    char pBuf[0x100] = { 0 };
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", CONTROL_R_INDEX);
    _fn_add_item("R_CONTROL", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", E_STOP_R_INDEX);
    _fn_add_item("R_E_STOP", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", WELD_MOV_RATE_R_INDEX);
    _fn_add_item("R_WELD_MOV_RATE", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "NumReg(%d)", ANY_MOV_RATE_R_INDEX);
    _fn_add_item("R_ANY_MOV_RATE", "INT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", SAFE_START_PR_INDEX);
    _fn_add_item("SAFE_START_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", SAFE_END_PR_INDEX);
    _fn_add_item("SAFE_END_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", WELD_START_PR_INDEX);
    _fn_add_item("WELD_START_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", WELD_LOOP_PR_INDEX);
    _fn_add_item("WELD_LOOP_PR", "FLOAT", pBuf);

    memset(pBuf, 0, sizeof(pBuf));
    snprintf(pBuf, sizeof(pBuf), "X_PR(%d)", ANY_PR_INDEX);
    _fn_add_item("ANY_PR", "FLOAT", pBuf);
}

bool FanucInterface::RobotConnect(){
    assert(_fn_connect != nullptr && _fn_init_robot_obj != nullptr&&_fn_connect != nullptr);
    bool result = false;
    _fn_init_robot_obj(_ip, _port);
    RobotAddAllItem();
    if (_fn_connect()){
        result = true;
        RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "Connected to Fanuc robot successfully!");
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("robot_driver_fanuc"), "Failed to connect to Fanuc robot!");
    }
    return result;
}

bool FanucInterface::GetItemFloatValue(char* cKey, float& fOutValue)
{
    assert(_fn_get_item_value != nullptr);
    bool result = false;
    char pOut[256] = { 0 };
    if (_fn_get_item_value(cKey, pOut, 256))
    {
        fOutValue = std::stof(pOut);
        result = true;
    }
    return result;
}

bool FanucInterface::GetItemDoubleValue(char *key, double &value) {
    assert(_fn_get_item_value != nullptr);
    bool result = false;
    char buff[256] = { 0 };
    if (_fn_get_item_value(key, buff, 256)){
        value = std::stod(buff);
        result = true;
    }
    return result;
}

bool FanucInterface::GetItemIntValue(char* cKey, int& nOutValue)
{
    assert(_fn_get_item_value != nullptr);
    bool result = false;
    char pOut[256] = { 0 };
    if (_fn_get_item_value(cKey, pOut, 256))
    {
        nOutValue = std::stoi(pOut);
        result = true;
    }
    return result;
}

bool FanucInterface::SetRValue(int index, short value)
{
    assert(_fn_set_value != nullptr);
    return _fn_set_value(index,value);
}

bool FanucInterface::Stop()
{
    return SetRValue(CONTROL_R_INDEX, STOP);
}

bool FanucInterface::EmgStop()
{
    bool result = Stop();
    result &= SetRValue(E_STOP_R_INDEX, 1);
    return result;
}

bool FanucInterface::AnyMovePositionSet(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    assert(_fn_set_value_xyzwpr != nullptr);
    RCLCPP_INFO(rclcpp::get_logger("robot_driver_fanuc"), "%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",X,Y,Z,W,P,R,E1);
    bool result = _fn_set_value_xyzwpr(ANY_PR_INDEX,
                                 X, Y, Z, W, P, R, E1, E2, E3,
                                 C0, C1, C2, C3, C4, C5, C6,
                                 UF, UT);
    return result;
}

bool FanucInterface::AnyMoveLoopSignSet(bool sign){
    return SetRValue(CONTROL_R_INDEX,sign? GO_ANY_POS_LOOP:STOP);
}

bool FanucInterface::AnyMoveSignSet(bool sign = true){
    return SetRValue(CONTROL_R_INDEX,sign? GO_ANY_POS:STOP);
}

bool FanucInterface::WeldLoopMovePositionSet(
        float X, float Y, float Z, float W, float P, float R, float E1 = 0, float E2 = 0, float E3 = 0,
        short C0 = 1, short C1 = 0, short C2 = 1, short C3 = 1, short C4 = 0, short C5 = 0, short C6 = 0,
        short UF = 255, short UT = 255)
{
    assert(_fn_set_value_xyzwpr != nullptr);
    return _fn_set_value_xyzwpr(WELD_LOOP_PR_INDEX,
                                       X, Y, Z, W, P, R, E1, E2, E3,
                                       C0, C1, C2, C3, C4, C5, C6,
                                       UF, UT);
}

bool FanucInterface::WeldLoopMoveSignSet(bool sign = true){
    return SetRValue(CONTROL_R_INDEX,sign? GO_WELD_LOOP_POS:STOP);
}

weld_interface::msg::TcpPos FanucInterface::GetRobotPose(bool force_update){
    TcpPos pos;
    _fn_update_cache(force_update);
    GetItemDoubleValue("世界坐标X",pos.x);
    GetItemDoubleValue("世界坐标Y",pos.y);
    GetItemDoubleValue("世界坐标Z",pos.z);
    GetItemDoubleValue("世界坐标W",pos.rx);
    GetItemDoubleValue("世界坐标P",pos.ry);
    GetItemDoubleValue("世界坐标R",pos.rz);
    GetItemDoubleValue("世界坐标E1",pos.e1);
    GetItemDoubleValue("世界坐标E2",pos.e2);
    GetItemDoubleValue("世界坐标E3",pos.e3);
    return pos;
}
