
/*
* @file abc_vtol.h
* @brief 三旋翼VTOL（垂直起降）飞行器类型头文件
*
* @author wangsl 	<wangsenlin.cn@gmail.com>
*
* -----------------------------------------------------------------------------
* ABC-VTOL 机体坐标系约定（唯一规范定义，PX4 与 Gazebo 均以此为准）
* -----------------------------------------------------------------------------
* MC 与 FW 使用同一机体坐标系，不做单独「FW 坐标系」定义。
*
* 机头（前向 / 机体 X 轴）口径统一为 MC：
*   - 机头 = 1、3 号旋翼的水平前方（多旋翼前向），不是物理机头（外形上的机头）。
*   - 避免“机身竖直时 X=机头=朝上”的 FW 口径混用；前向一律指 1、3 旋翼的水平前方。
*
* 唯一机体坐标系：PX4 FRD（Forward-Right-Down），即 base_link。
*   - X 前、Y 右、Z 下。（X 前 = 上述机头方向）
*   - Roll/Pitch/Yaw 含义统一：Pitch 0° = 机头水平（水平前飞）；正 pitch = 机头上仰。
*
* 因此：同一姿态下，MC 与 FW 计算/输出的欧拉角 (roll, pitch, yaw) 应相同。
* 若打印中 MC 与 FW 设定值不同，是因为二者来自不同控制器（MC 多旋翼、FW 固定翼），
* 目标姿态可能不同（例如 FW 要水平前飞、MC 要悬停），并非坐标系不同。
* -----------------------------------------------------------------------------
*/

#ifndef ABC_VTOL_4R_H
#define ABC_VTOL_4R_H

#include "vtol_type.h"  // 包含VTOL基类

#include <parameters/param.h>  // 参数库
#include <drivers/drv_hrt.h>   // 高分辨率定时器
#include <matrix/matrix/math.hpp>  // 矩阵数学库

// uORB话题头文件
#include <uORB/topics/vehicle_torque_setpoint.h>

// [rad] 自动转换到固定翼模式所需的俯仰角阈值
static constexpr float ABC4R_PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW = -1.396f; // -80°

// [rad] 自动转换到悬停模式所需的俯仰角阈值
static constexpr float ABC4R_PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC = -0.26f; // -15°

// [s] 从固定翼模式到返回转换油门的推力混合持续时间
static constexpr float ABC4R_B_TRANS_THRUST_BLENDING_DURATION = 0.5f;

/**
 * @brief 倾转旋翼VTOL类型类
 *
 * 继承自VtolType基类，实现了倾转旋翼特定的控制逻辑。
 */
class AbcVtol4R : public VtolType
{

public:
    /**
     * @brief 构造函数
     * @param _att_controller 指向VTOL姿态控制器的指针
     */
    AbcVtol4R(VtolAttitudeControl *_att_controller);

    /// @brief 析构函数（使用默认实现）
    ~AbcVtol4R() override = default;

    /// @brief 更新VTOL状态
    void update_vtol_state() override;

    /// @brief 更新转换状态
    void update_transition_state() override;

    /// @brief 更新固定翼状态
    void update_fw_state() override;

    /// @brief 填充执行器输出
    void fill_actuator_outputs() override;

    /// @brief 在TECS运行前的特殊处理
    void waiting_on_tecs() override;

    /**
     * @brief 在前向转换后混合油门
     * @param scale 混合比例因子
     */
    void blendThrottleAfterFrontTransition(float scale) override;

    /**
     * @brief 在返回转换开始时混合油门
     * @param scale 混合比例因子
     */
    void blendThrottleBeginningBackTransition(float scale);

	// 在初入FW时，TECS可能尚未介入，此时的姿态、推力和扭力都可能异常，那么需要使用TR最后的扭力填充
	struct vehicle_torque_setpoint_s 		_last_tr_torque{};
    bool bWaitingOnTECS{false};  ///< 是否正在等待TECS（总能量控制系统）
private:
    /// @brief VTOL飞行模式枚举
    enum class vtol_mode {
        MC_MODE = 0,            ///< VTOL处于多旋翼模式
        TRANSITION_FRONT_P0,    ///< VTOL处于前向转换准备阶段（小倾角拉出前向速度）
        TRANSITION_FRONT_P1,    ///< VTOL处于前向转换第一部分模式（大倾角完成向前转换）
        TRANSITION_BACK,        ///< VTOL处于返回转换模式
        FW_MODE                 ///< VTOL处于固定翼模式
    };

    vtol_mode _vtol_mode{vtol_mode::MC_MODE};  ///< VTOL飞行模式，由枚举vtol_mode定义

    bool _flag_was_in_trans_mode = false;  ///< 如果模式刚刚切换到转换模式，则为true

    matrix::Quatf _q_trans_start;  ///< 转换开始的四元数姿态
    matrix::Quatf _q_trans_sp;     ///< 转换过程中的姿态设定值
    matrix::Vector3f _trans_rot_axis;  ///< 转换旋转轴

    /// @brief 更新参数
    void parameters_update() override;

    /**
     * @brief 检查前向转换是否完成（基础条件）
     * @return 如果完成返回true，否则返回false
     */
    bool isFrontTransitionCompletedBase() override;

    /*
     * @brief 获取地速
     * @return 地速[m/s]
     */
    float get_ground_speed();

    // 参数定义
    DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
                    (ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off  ///< 固定翼俯仰设定点偏移
                   )
};
#endif
