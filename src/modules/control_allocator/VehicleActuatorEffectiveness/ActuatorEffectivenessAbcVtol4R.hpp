/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessAbcVtol4R.hpp
 * @brief 垂起固定翼VTOL飞行器的执行器效应矩阵头文件
 *
 * 定义用于垂起固定翼VTOL（垂直起降）飞行器的执行器效应矩阵计算类
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

#include <uORB/topics/normalized_unsigned_setpoint.h>

#include <uORB/Subscription.hpp>

/**
 * @class ActuatorEffectivenessAbcVtol4R
 * @brief 垂起固定翼VTOL飞行器的执行器效应矩阵计算类
 *
 * 该类继承自ModuleParams和ActuatorEffectiveness，用于计算垂起固定翼VTOL飞行器的执行器效应矩阵。
 * 支持多旋翼和固定翼两种飞行模式，分别使用不同的分配方法和控制策略。
 */
class ActuatorEffectivenessAbcVtol4R : public ModuleParams, public ActuatorEffectiveness
{
public:
	/**
	 * @brief 构造函数
	 * @param parent 父模块参数指针
	 */
	ActuatorEffectivenessAbcVtol4R(ModuleParams *parent);

	/**
	 * @brief 析构函数（默认）
	 */
	virtual ~ActuatorEffectivenessAbcVtol4R() = default;

	/**
	 * @brief 获取执行器效应矩阵
	 * @param configuration 配置参数引用
	 * @param external_update 外部更新原因
	 * @return 成功获取返回true，否则返回false
	 */
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	/**
	 * @brief 获取矩阵数量
	 * @return 返回矩阵数量（垂起固定翼VTOL使用2个矩阵）
	 */
	int numMatrices() const override { return 2; }

	/**
	 * @brief 获取期望的分配方法
	 * @param allocation_method_out 分配方法输出数组
	 *
	 * 第一个矩阵使用顺序去饱和法，第二个矩阵使用伪逆法
	 */
	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		static_assert(MAX_NUM_MATRICES >= 2, "expecting at least 2 matrices");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;  // 多旋翼模式使用顺序去饱和
		allocation_method_out[1] = AllocationMethod::PSEUDO_INVERSE;           // 固定翼模式使用伪逆
	}

	/**
	 * @brief 获取是否对RPY进行归一化
	 * @param normalize 归一化标志输出数组
	 *
	 * 第一个矩阵（多旋翼）需要归一化，第二个矩阵（固定翼）不需要
	 */
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;   // 多旋翼模式需要归一化
		normalize[1] = false;  // 固定翼模式不需要归一化
	}

	/**
	 * @brief 分配辅助控制量
	 * @param dt 时间步长
	 * @param matrix_index 矩阵索引
	 * @param actuator_sp 执行器设置点向量
	 */
	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	/**
	 * @brief 更新设置点
	 * @param control_sp 控制设置点向量
	 * @param matrix_index 矩阵索引
	 * @param actuator_sp 执行器设置点向量
	 * @param actuator_min 执行器最小限幅值
	 * @param actuator_max 执行器最大限幅值
	 */
	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	/**
	 * @brief 设置飞行阶段
	 * @param flight_phase 飞行阶段
	 *
	 * 根据飞行阶段切换多旋翼和固定翼模式
	 */
	void setFlightPhase(const FlightPhase &flight_phase) override;

	/**
	 * @brief 获取模块名称
	 * @return 返回模块名称字符串
	 */
	const char *name() const override { return "VTOL Tailsitter"; }

protected:
	ActuatorEffectivenessRotors _mc_rotors;                     ///< 多旋翼转子执行器效应计算对象
	ActuatorEffectivenessControlSurfaces _control_surfaces;     ///< 控制面执行器效应计算对象

	uint32_t _forwards_motors_mask{};                           ///< 前向电机掩码（用于固定翼模式）

	int _first_control_surface_idx{0};                          ///< 第一个控制面的索引（应用于矩阵1）

	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};         ///< 襟翼设置点订阅
	uORB::Subscription _spoilers_setpoint_sub{ORB_ID(spoilers_setpoint)};   ///< 扰流板设置点订阅
};
