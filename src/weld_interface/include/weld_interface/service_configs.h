//
// Created by huang on 2026/3/5.
//

#ifndef ROS2_MAIN_SERVICE_CONFIGS_H
#define ROS2_MAIN_SERVICE_CONFIGS_H

//服务名定义：
//auto_cover 节点
#define UPDATE_TARGET_LINE_SRV_NAME "/update_target_line" //更新目标焊缝线系数服务 weld_interface::srv::UpdateLineCoeffs
#define UPDATE_TARGET_HEIGHT_SRV_NAME "/update_target_height" //更新目标焊缝高度服务 weld_interface::srv::UpdateHeight
#define UPDATE_MOVE_AXES_SRV_NAME "/update_move_axes" //更新移动轴参数服务 weld_interface::srv::UpdateMoveAxes
#define MOVE_SRV_NAME "/move" //机器人移动控制服务 weld_interface::srv::Move
#define START_PREPARE_SRV_NAME "/start_prepare" //开始准备阶段服务 std_srvs::srv::Empty
#define STOP_PREPARE_SRV_NAME "/stop_prepare" //停止准备阶段服务 std_srvs::srv::Empty
#define START_ADJUST_SRV_NAME "/start_adjust" //开始调整阶段服务 std_srvs::srv::Empty
#define START_FORWARD_SRV_NAME "/start_forward" //开始前进焊接服务 weld_interface::srv::StartForward
#define STOP_RUN_SRV_NAME "/stop_run" //停止运行服务 std_srvs::srv::Empty

#define ENABLE_RECON_SRV_NAME "/enable_recon" //重建使能服务 IntDataSrv
#define SET_MODE_SRV_NAME "/set_mode" //模式设置服务 IntDataSrv
#define COVER_PASS_RESET_SRV_NAME "/cover_pass_reset" //盖面焊重置服务 std_srvs::srv::Empty

#define START_TEACHING_SRV_NAME "/start_teaching" //开始示教服务 std_srvs::srv::Empty
#define RECORD_CURRENT_TEACHING_SRV_NAME "/record_current_teaching" //记录当前示教点服务 weld_interface::srv::RecordCurrentTeaching
#define MOVE_TO_NEXT_TEACHING_POINT_SRV_NAME "/move_to_next_teaching_point" //移动到下一个示教点服务 weld_interface::srv::MoveToNextTeachingPoint
#define MOVE_TO_FIRST_TEACHING_POINT_SRV_NAME "/move_to_first_teaching_point" //移动到第一个示教点服务 weld_interface::srv::MoveToFirstTeachingPoint

//camera_3d_driver节点
#define START_FIX_SCAN_SRV_NAME "start_fix_scan" //开始固定扫描服务 std_srvs::srv::Empty
#define STOP_FIX_SCAN_SRV_NAME "stop_fix_scan" //停止固定扫描服务 std_srvs::srv::Empty
#define SCAN_3D_SRV_NAME "scan_3d" //3D扫描服务（单次触发） weld_interface::srv::Scan3d
#define UPDATE_CAMERA_3D_NODE_TARGET_HEIGHT_SRV_NAME "/update_camera_3d_node_target_height" //更新3D相机目标高度服务 weld_interface::srv::UpdateHeight
#define UPDATE_CAMERA_3D_NODE_CROPPING_Z_RANGE_SRV_NAME "/update_camera_3d_node_cropping_z_range" //更新Z轴裁剪范围服务 weld_interface::srv::UpdateRange
#define RELOAD_CAMERA_3D_CONFIG_SRV_NAME "/reload_camera_3d_config" //重新加载3D相机配置服务 std_srvs::srv::Trigger

//data_collect节点
#define DATA_COLLECT_ACTIVATE_SRV_NAME "/data_collect_activate" //数据采集激活服务 std_srvs::srv::Empty
#define DATA_COLLECT_DEACTIVATE_SRV_NAME "/data_collect_deactivate" //数据采集停用服务 std_srvs::srv::Empty
#define DATA_COLLECT_TARGET_REGISTER_SET_SRV_NAME "/data_collect_target_register_set" //设置数据采集目标寄存器编号服务 weld_interface::srv::IntData
#define DATA_COLLECT_SET_TASK_SRV_NAME "/data_collect_set_task" //设置当前采集任务信息服务 weld_interface::srv::SetCollectionTask

//fanuc_robot节点
#define SAFE_START_MOV_JOG_SRV_NAME "safe_start_mov_jog" //安全起始点JOG移动服务 weld_interface::srv::Move
#define SAFE_END_MOV_JOG_SRV_NAME "safe_end_mov_jog" //安全结束点JOG移动服务 weld_interface::srv::Move
#define ANY_MOV_JOG_SRV_NAME "any_mov_jog" //任意方向JOG移动服务 weld_interface::srv::SpecialSpeedl
#define ANY_MOV_OFFSET_SRV_NAME "any_mov_offset" //任意方向偏移移动服务 weld_interface::srv::SpecialSpeedl
#define WELD_START_MOV_JOG_SRV_NAME "weld_start_mov_jog" //焊接起始点JOG移动服务 weld_interface::srv::SpecialSpeedl
#define WELD_LOOP_MOV_JOG_SRV_NAME "weld_loop_mov_jog" //焊接循环JOG移动服务 weld_interface::srv::SpecialSpeedl
#define STOP_MOV_JOG_SRV_NAME "stop_mov_jog" //停止移动服务 std_srvs::srv::Empty
#define E_STOP_MOV_JOG_SRV_NAME "e_stop_mov_jog" //紧急停止服务 std_srvs::srv::Empty
#define WELD_LOOP_STOP_SRV_NAME "weld_loop_stop" //焊接停止服务 std_srvs::srv::Empty
#define WELD_LOOP_RATE_SET_SRV_NAME "weld_loop_rate_set" //焊接循环速度设置服务 weld_interface::srv::FanucMovRate
#define ANY_MOV_LOOP_RATE_SET_SRV_NAME "any_mov_loop_rate_set" //任意移动循环速度设置服务 weld_interface::srv::FanucMovRate
#define ANY_MOV_LOOP_POSITION_SET_SRV_NAME "any_mov_loop_position_set" //任意JOG循环位置设置服务 weld_interface::srv::SpecialSpeedl
#define ANY_MOV_LOOP_SIGN_SET_SRV_NAME "any_mov_loop_sign_set" //任意JOG循环方向设置服务 std_srvs::srv::SetBool
#define FANUC_REGISTER_READ_SRV_NAME "/fanuc_register_read" //读取指定FANUC数值寄存器服务 weld_interface::srv::ReadFanucRegister

//ros2_bridge节点
#define SCAN_SRV_NAME "/scan" //记录当前示教点 std_srvs::srv::Empty
#define POSE_ESTIMATE_SRV_NAME "/pose_estimate" //调用位姿估计服务 std_srvs::srv::Empty
#define INIT_POINT_SRV_NAME "/init_point" //移动到初始点 weld_interface::srv::InitPoint


//client定义:

//auto_cover 节点
#define SCAN_3D_CLIENT_NAME "/scan_3d" //3D扫描客户端 weld_interface::srv::Scan3d
#define WELD_START_MOV_JOG_CLIENT_NAME "/weld_start_mov_jog" //焊接启动移动客户端 weld_interface::srv::SpecialSpeedl
#define WELD_LOOP_MOV_JOG_CLIENT_NAME "/weld_loop_mov_jog" //焊接循环移动客户端 weld_interface::srv::SpecialSpeedl
#define WELD_LOOP_RATE_SET_CLIENT_NAME "/weld_loop_rate_set" //焊接速度设置客户端 weld_interface::srv::FanucMovRate
#define STOP_MOV_JOG_CLIENT_NAME "/stop_mov_jog" //停止移动客户端 std_srvs::srv::Empty
#define START_FIX_SCAN_CLIENT_NAME "/start_fix_scan" //开始固定扫描客户端 std_srvs::srv::Empty
#define STOP_FIX_SCAN_CLIENT_NAME "/stop_fix_scan" //停止固定扫描客户端 std_srvs::srv::Empty
#define UPDATE_CAMERA_3D_NODE_TARGET_HEIGHT_CLIENT_NAME "/update_camera_3d_node_target_height" //更新3D相机目标高度客户端 weld_interface::srv::UpdateHeight
#define UPDATE_CAMERA_3D_NODE_CROPPING_Z_RANGE_CLIENT_NAME "/update_camera_3d_node_cropping_z_range" //更新3D相机裁剪Z轴范围客户端 weld_interface::srv::UpdateRange
#define RELOAD_CAMERA_3D_CONFIG_CLIENT_NAME "/reload_camera_3d_config" //重新加载3D相机配置客户端 std_srvs::srv::Trigger
#define DATA_COLLECT_ACTIVATE_CLIENT_NAME "/data_collect_activate" //数据采集激活客户端 std_srvs::srv::Empty
#define DATA_COLLECT_DEACTIVATE_CLIENT_NAME "/data_collect_deactivate" //数据采集停用客户端 std_srvs::srv::Empty
#define DATA_COLLECT_SET_TASK_CLIENT_NAME "/data_collect_set_task" //设置当前采集任务信息客户端 weld_interface::srv::SetCollectionTask
#define ENABLE_RECON_CLIENT_NAME "/enable_recon" //重建使能客户端 IntDataSrv
#define SET_MODE_CLIENT_NAME "/set_mode" //模式设置客户端 IntDataSrv
#define COVER_PASS_RESET_CLIENT_NAME "/cover_pass_reset" //盖面焊重置客户端 std_srvs::srv::Empty

#define UPDATE_TARGET_LINE_CLIENT_NAME "/update_target_line" //更新目标焊缝线客户端 weld_interface::srv::UpdateLineCoeffs
#define UPDATE_TARGET_HEIGHT_CLIENT_NAME "/update_target_height" //更新目标焊缝高度客户端 weld_interface::srv::UpdateHeight
#define UPDATE_MOVE_AXES_CLIENT_NAME "/update_move_axes" //更新移动轴参数客户端 weld_interface::srv::UpdateMoveAxes
#define MOVE_CLIENT_NAME "/move" //机器人移动控制客户端 weld_interface::srv::Move
#define ANY_MOV_JOG_CLIENT_NAME "/any_mov_jog" //任意移动JOG客户端 weld_interface::srv::SpecialSpeedl

//ros2_bridge节点
#define RECORD_CURRENT_TEACHING_CLIENT_NAME "/record_current_teaching" //记录当前示教点 weld_interface::srv::RecordCurrentTeaching


#endif //ROS2_MAIN_SERVICE_CONFIGS_H
