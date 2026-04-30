//
// Created by huang on 2026/1/8.
//

#ifndef SRC_FANUC_INTERFACE_H
#define SRC_FANUC_INTERFACE_H

#include "weld_interface/msg/tcp_pos.hpp"

class FanucInterface {
    // 定义函数指针类型（保持原有逻辑）
    typedef void (*InitRobotObj)(char* cIp,int nPort);
    typedef void (*AddItem)(const char* cName,const char* cVarType,const char* cAddr);
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

    // 寄存器索引宏定义（保持原有）
    #define CONTROL_R_INDEX 205
    #define E_STOP_R_INDEX 206
    #define WELD_MOV_RATE_R_INDEX 207
    #define ANY_MOV_RATE_R_INDEX 208

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
public:
    FanucInterface(char ip[0x20],int port);

    ~FanucInterface();

public:
    bool InitLibraryFunction();

    void RobotAddAllItem();

    bool RobotConnect();

private:
    bool GetItemFloatValue(char* cKey, float& fOutValue);

    bool GetItemDoubleValue(char *key, double &value);

    bool GetItemIntValue(char* cKey, int& nOutValue);

    bool SetRValue(int index, short value);

    bool Stop();

    bool EmgStop();

    bool AnyMovePositionSet(float X, float Y, float Z, float W, float P, float R, float E1 , float E2 , float E3 ,
                            short C0 , short C1 , short C2 , short C3 , short C4 , short C5 , short C6 ,
                            short UF , short UT );

    bool AnyMoveLoopSignSet(bool sign);

    bool AnyMoveSignSet(bool sign);

    bool WeldLoopMovePositionSet(float X, float Y, float Z, float W, float P, float R, float E1 , float E2 , float E3 ,
                                 short C0 , short C1 , short C2 , short C3 , short C4 , short C5 , short C6 ,
                                 short UF , short UT );

    bool WeldLoopMoveSignSet(bool sign);

    weld_interface::msg::TcpPos GetRobotPose(bool force_update);
private:
    char _ip[0x20];
    int _port;
    InitRobotObj  _fn_init_robot_obj ;
    AddItem _fn_add_item;
    Connect _fn_connect;
    DisConnect _fn_disconnect;
    IsConnected _fn_is_connected;
    SetValueXyzwpr _fn_set_value_xyzwpr;
    UpdateCache _fn_update_cache;
    GetItemValue _fn_get_item_value;
    SetValue _fn_set_value;
};


#endif //SRC_FANUC_INTERFACE_H
