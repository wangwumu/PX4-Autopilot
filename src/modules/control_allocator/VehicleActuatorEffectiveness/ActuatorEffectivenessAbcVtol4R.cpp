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
 * @file ActuatorEffectivenessTailsitterVTOL.cpp
 * @brief 垂起固定翼VTOL飞行器的执行器效应矩阵实现文件
 *
 * 实现垂起固定翼VTOL（垂直起降）飞行器的执行器效应矩阵计算和相关功能
 */

#include "ActuatorEffectivenessAbcVtol4R.hpp"

using namespace matrix;

/**
 * @brief 构造函数
 * @param parent 父模块参数指针
 *
 * 初始化模块参数、多旋翼转子效应计算对象和控制面效应计算对象，
 * 并设置初始飞行阶段为悬停模式
 */
ActuatorEffectivenessAbcVtol4R::ActuatorEffectivenessAbcVtol4R(ModuleParams *parent)
	: ModuleParams(parent), _mc_rotors(this), _control_surfaces(this)
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);  // 初始设置为悬停飞行模式
}

/**
 * @brief 获取执行器效应矩阵
 * @param configuration 配置参数引用
 * @param external_update 外部更新原因
 * @return 如果成功获取效应矩阵返回true，否则返回false
 *
 * 此函数仅在外部更新时才会计算效应矩阵，否则直接返回false。
 * 分别配置多旋翼模式（矩阵0）和固定翼模式（矩阵1）的执行器效应矩阵。
 */
bool
ActuatorEffectivenessAbcVtol4R::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// 如果没有外部更新请求，直接返回false
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// 配置多旋翼模式（矩阵0）
	configuration.selected_matrix = 0;
	// 如果转子数量大于3，启用通过差动推力进行偏航控制
	_mc_rotors.enableYawByDifferentialThrust(_mc_rotors.geometry().num_rotors > 3);
	// 添加多旋翼转子执行器到配置中
	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// 配置固定翼模式（矩阵1）
	configuration.selected_matrix = 1;
	// 记录第一个控制面的索引位置
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	// 添加控制面执行器到配置中
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	// 返回两个矩阵是否都成功添加
	return (mc_rotors_added_successfully && surfaces_added_successfully);
}

/**
 * @brief 分配辅助控制量
 * @param dt 时间步长
 * @param matrix_index 矩阵索引
 * @param actuator_sp 执行器设置点向量
 *
 * 对于固定翼模式（矩阵1），应用襟翼和扰流板的设置点
 */
void ActuatorEffectivenessAbcVtol4R::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	// 只处理固定翼模式（矩阵1）
	if (matrix_index == 1) {
		// 应用襟翼设置
		normalized_unsigned_setpoint_s flaps_setpoint;

		// 获取襟翼设置点并应用
		if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
			_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}

		// 应用扰流板设置
		normalized_unsigned_setpoint_s spoilers_setpoint;

		// 获取扰流板设置点并应用
		if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
			_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}
	}
}

/**
 * @brief 更新设置点
 * @param control_sp 控制设置点向量
 * @param matrix_index 矩阵索引
 * @param actuator_sp 执行器设置点向量
 * @param actuator_min 执行器最小限幅值
 * @param actuator_max 执行器最大限幅值
 *
 * 对于多旋翼模式（矩阵0），在零推力时停止前向电机
 */
void ActuatorEffectivenessAbcVtol4R::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	// 只处理多旋翼模式（矩阵0）
	if (matrix_index == 0) {
		// 在零推力时停止前向电机
		stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);
	}
}

/**
 * @brief 设置飞行阶段
 * @param flight_phase 飞行阶段
 *
 * 根据飞行阶段更新前向电机掩码和停止电机掩码，
 * 用于在不同飞行模式下控制电机的启停
 */
void ActuatorEffectivenessAbcVtol4R::setFlightPhase(const FlightPhase &flight_phase)
{
	// 如果飞行阶段未改变，直接返回
	if (_flight_phase == flight_phase) {
		return;
	}

	// 调用基类方法设置飞行阶段
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// 根据飞行阶段更新电机掩码
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:  // 前飞模式
		// 获取向上推进的电机掩码（在分配框架中保持向上）
		_forwards_motors_mask = _mc_rotors.getUpwardsMotors();
		break;

	case FlightPhase::HOVER_FLIGHT:            // 悬停模式
	case FlightPhase::TRANSITION_FF_TO_HF:     // 前飞到悬停的过渡模式
	case FlightPhase::TRANSITION_HF_TO_FF:     // 悬停到前飞的过渡模式
		// 清零前向电机掩码和停止电机掩码
		_forwards_motors_mask = 0;
		_stopped_motors_mask = 0;
		break;
	}
}
