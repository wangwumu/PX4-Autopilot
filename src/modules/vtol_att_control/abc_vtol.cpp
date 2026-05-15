
/**
* @file abc_vtol.cpp
* @author wangsl 	<wangsenlin.cn@gmail.com>
* @brief 尾座式无人机（AbcVtol）VTOL控制实现
*
* 尾座式无人机是一种垂直起降飞行器，通过改变姿态实现多旋翼和固定翼模式间的转换。
*/

#include "abc_vtol.h"
#include "px4_platform_common/defines.h"
#include "vtol_att_control_main.h"

using namespace matrix;

// 构造函数
AbcVtol::AbcVtol(VtolAttitudeControl *attc)
	: VtolType(attc) // 调用基类构造函数
{}

// 参数更新函数
void AbcVtol::parameters_update() {
	VtolType::updateParams(); // 调用基类参数更新
}

// 更新VTOL状态函数
void AbcVtol::update_vtol_state() {
	/* 使用双向开关进行模式转换的简单逻辑
	* 拨动开关后，飞行器将在多旋翼（MC）控制模式下开始倾斜，加速前进
	* 当达到足够的速度和俯仰角后，转入固定翼（FW）模式
	* 回退转换时，再次使用MC模式控制俯仰，达到足够俯仰角后切换回全MC控制
	*/

	// 如果固定翼系统出现故障，立即切换到MC模式
	if (_vtol_vehicle_status->fixed_wing_system_failure) {
        if (_vtol_mode != vtol_mode::MC_MODE) {
            _transition_start_timestamp = hrt_absolute_time(); // 记录转换开始时间
        }
        _vtol_mode = vtol_mode::MC_MODE; // 强制切换到MC模式

        // 用户请求MC模式
	}
	else if (!_attc->is_fixed_wing_requested()) {
        switch (_vtol_mode) {
        case vtol_mode::MC_MODE:
            break; // 已经是MC模式，无需操作

        case vtol_mode::FW_MODE:
            resetTransitionStates();		 // 重置转换状态
            _vtol_mode = vtol_mode::TRANSITION_BACK; // 开始回退转换
            break;

        case vtol_mode::TRANSITION_FRONT_P0:
        case vtol_mode::TRANSITION_FRONT_P1:
            _vtol_mode = vtol_mode::MC_MODE; // 故障保护，直接回MC模式
            break;

        case vtol_mode::TRANSITION_BACK:
            const float pitch = Eulerf(Quatf(_v_att->q)).theta(); // 获取当前俯仰角

            // 检查是否达到切换回MC模式的俯仰角阈值或超时
            if (pitch >= ABC_PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC ||
                _time_since_trans_start > _param_vt_b_trans_dur.get()) {
            _vtol_mode = vtol_mode::MC_MODE; // 切换回MC模式
            }
            break;
        }

	// 用户请求FW模式
	}
	else {
        switch (_vtol_mode) {
        case vtol_mode::MC_MODE:
            // 从MC进入前向转换准备阶段（P0）：小倾角拉出前向速度
            _vtol_mode = vtol_mode::TRANSITION_FRONT_P0;
            resetTransitionStates();			 // 重置转换状态
            break;

        case vtol_mode::TRANSITION_FRONT_P0: {
            // P0阶段：仅当地速超过转换空速VT_ARSP_TRANS（经重量缩放）时，才进入P1
            const float speed = get_ground_speed();
            const float trans_speed = getTransitionAirspeed();

            if (PX4_ISFINITE(speed) && speed > trans_speed) {
                // 速度足够，进入P1：从当前 P0 的设定姿态接续，避免 pitch 回落
                _vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
                _q_trans_start = _q_trans_sp;   // P1 的“零位”沿用当前设定姿态（约 30°），不重新初始化
                resetTransitionStates();       // 仅重置计时，t/angle 从 0 继续
                // 不置 _flag_was_in_trans_mode = false，避免重新初始化 _q_trans_start / _trans_rot_axis
            }
            break;
        }

        case vtol_mode::FW_MODE:
            break; // 已经是FW模式，无需操作

        case vtol_mode::TRANSITION_FRONT_P1:
            if (isFrontTransitionCompleted()) {
                _vtol_mode = vtol_mode::FW_MODE;	  // 完成转换，进入FW模式
                _trans_finished_ts = hrt_absolute_time(); // 记录转换完成时间
            }
            break;

        case vtol_mode::TRANSITION_BACK:
            _vtol_mode = vtol_mode::FW_MODE;	  // 故障保护，直接进入FW模式
            _trans_finished_ts = hrt_absolute_time(); // 记录转换完成时间
            break;
        }
	}

	// 将ABC VTOL特定的控制阶段映射到简单控制模式
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
        _common_vtol_mode = mode::ROTARY_WING; // 旋翼模式
        _flag_was_in_trans_mode = false;	   // 重置转换标志
    	break;

	case vtol_mode::FW_MODE:
        _common_vtol_mode = mode::FIXED_WING; // 固定翼模式
        _flag_was_in_trans_mode = false;	  // 重置转换标志
        break;

	case vtol_mode::TRANSITION_FRONT_P0:
	case vtol_mode::TRANSITION_FRONT_P1:
        _common_vtol_mode = mode::TRANSITION_TO_FW; // 向固定翼转换
        break;

	case vtol_mode::TRANSITION_BACK:
        _common_vtol_mode = mode::TRANSITION_TO_MC; // 向多旋翼转换
        break;
	}
}

// 更新转换状态函数
void AbcVtol::update_transition_state()
{
	VtolType::update_transition_state(); // 调用基类转换状态更新

	const hrt_abstime now = hrt_absolute_time(); // 获取当前时间

	// 检查虚拟MC姿态设定值是否最新，否则保持上一设定值
	if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
		return;
	}

	// 首次进入转换模式时的初始化
	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true; // 设置转换标志

		// 回退转换初始化
		if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
			_q_trans_start = Quatf(_v_att->q);		// 记录初始姿态四元数
			Vector3f z = -_q_trans_start.dcm_z();		    // 机体朝上方向(FRD)
			_trans_rot_axis = z.cross(Vector3f(0.F, 0.F, -1.F)); // 旋转轴 = 机体朝上 × 地理朝上

			// 设置偏航设定值为机体指向的方向
			const float yaw_sp = atan2f(z(1), z(0)); // 计算偏航角

			// 回退转换的初始姿态设定值结合当前FW俯仰设定值和计算的偏航设定值
			if (_fw_virtual_att_sp->timestamp > (now - 1_s)) { // 如果FW姿态设定值最新
				const float pitch_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).theta(); // 获取FW俯仰角
				_q_trans_start = Eulerf(0.f, pitch_body, yaw_sp);   // 构建四元数(roll=0, pitch=FW俯仰角, yaw=偏航角)
			}
			else {
				_q_trans_start = Eulerf(0.f, 0.f, yaw_sp); // 使用默认俯仰角(roll=0, pitch=0, yaw=偏航角)
			}

			// 将期望姿态旋转到多旋翼坐标系
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0)); // 旋转-90度

			// 向前转换初始化
		}
		else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P0
			 || _vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			const Eulerf setpoint_euler(Quatf(_mc_virtual_att_sp->q_d)); // 获取MC姿态欧拉角
			_q_trans_start = Eulerf(0.f, setpoint_euler.theta(), setpoint_euler.psi()); // 保持滚转为0(roll=0, pitch=MC俯仰角, yaw=MC偏航角)
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1.f, 0.f, 0.f); // 获取机体X轴向量(机体X轴向量在多旋翼坐标系中为(1,0,0))
			_trans_rot_axis = -x.cross(Vector3f(0.f, 0.f, -1.f)); // 计算旋转轴

			// 向前转换初始化
		}

		_q_trans_sp = _q_trans_start; // 初始化姿态设定值
	}

	_q_trans_sp.normalize(); // 确保四元数归一化

	// 计算倾斜角（机体鼻尖朝上时为0）
	const float cos_tilt = math::constrain(
		_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) -
			_q_trans_sp(2) * _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3),
		-1.f, 1.f);
	const float tilt = acosf(cos_tilt); // 当前设定姿态的倾斜角 [rad]


	// 向前转换过程（P0/P1分阶段倾转）
	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P0
	    || _vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// 计算俯仰速率，确保转换时间至少0.1秒（以90度为基准）
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);
		const float angle_rad = _time_since_trans_start * trans_pitch_rate; // 相对本阶段 _q_trans_start 的旋转角 [rad]

		// P0：使用较小目标倾角（例如30度），避免在速度不足时出现过大倾角导致失速
		if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P0) {
			const float tilt_target_p0 = math::radians(30.f);

			if (tilt < tilt_target_p0) {
				_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis, angle_rad)) * _q_trans_start;
			}

		// P1：使用原有目标倾角（90度-FW_PSP_OFF），完成向前转换
		} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			const float tilt_target_p1 = M_PI_2_F - math::radians(_param_fw_psp_off.get());

			if (tilt < tilt_target_p1) {
				_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis, angle_rad)) * _q_trans_start;
			}
		}
	}
	// 回退转换过程
	else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
	    // 计算俯仰速率，确保转换时间至少0.1秒
	    const float trans_pitch_rate = // 回退转换的俯仰速率(90度/转换时间)
		    M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f); // 回退转换的俯仰速率(90度/转换时间，如果转换时间小于0.1秒，则使用0.1秒)

	    // 如果还有倾斜，继续旋转(90度-偏航角偏移)
	    if (tilt > 0.01f) {
		    _q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis, _time_since_trans_start *
															trans_pitch_rate)) * _q_trans_start;
	    }
	}

	// 设置推力设定值
	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	// 回退转换开始时混合推力
	if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
		const float progress = math::constrain(_time_since_trans_start / ABC_B_TRANS_THRUST_BLENDING_DURATION,
			0.f, 1.f);
		blendThrottleBeginningBackTransition(progress); // 混合推力
	}

	_v_att_sp->timestamp = hrt_absolute_time(); // 更新时间戳

	// 强制设定姿态的 roll 为 0，仅保留 pitch、yaw
	const Eulerf euler_trans = Eulerf(Quatf(_q_trans_sp));
	_q_trans_sp = Quatf(Eulerf(0.f, euler_trans.theta(), euler_trans.psi()));

	_q_trans_sp.normalize();

	_q_trans_sp.copyTo(_v_att_sp->q_d); // 设置姿态设定值
}

// 等待TECS（总能量控制系统）函数
void AbcVtol::waiting_on_tecs() {
	_v_att_sp->thrust_body[0] = -_last_thr_in_mc;
}

// 更新固定翼状态函数
void AbcVtol::update_fw_state() {
	VtolType::update_fw_state(); // 调用基类固定翼状态更新
}

/**
* 写入执行器输出数据
*/
void AbcVtol::fill_actuator_outputs() {
	// 初始化扭矩和推力设定值
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// 电机控制
	if (_vtol_mode == vtol_mode::FW_MODE) { // 固定翼模式
        // 推力：virtual_fw 推力在本机始终为 0，用转换前 MC 推力；扭矩用 virtual_fw（正常）
        const float fw_thrust = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
        _thrust_setpoint_0->xyz[2] = -fabsf((fabsf(fw_thrust) > FLT_EPSILON) ? fw_thrust : _last_thr_in_mc);

		// 扭矩始终来自 virtual_fw（差动推力/姿态控制）
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] *
										_param_vt_fw_difthr_s_y.get();
		}
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) {
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] *
										_param_vt_fw_difthr_s_p.get();
		}
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] *
										_param_vt_fw_difthr_s_r.get();
		}

        // 刚切换到FW模式时，如果没有推力数据，使用最后的MC推力保持电机运转
        if (hrt_elapsed_time(&_trans_finished_ts) < 50_ms) {
            _thrust_setpoint_0->xyz[2] = _last_thr_in_mc;
            _torque_setpoint_0->xyz[0] = 0.f;
            _torque_setpoint_0->xyz[1] = 0.f;
            _torque_setpoint_0->xyz[2] = 0.f;
        }
    }
	else { // MC模式或转换模式
        _thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2]; // 设置MC推力

        // 回退转换开始时，如果没有推力数据，使用最后的FW推力保持电机运转
        if ((_vtol_mode != vtol_mode::TRANSITION_FRONT_P1 && _vtol_mode != vtol_mode::TRANSITION_FRONT_P0)
                    && hrt_elapsed_time(&_transition_start_timestamp) < 50_ms)   {
            _thrust_setpoint_0->xyz[2] = -_last_thr_in_fw_mode;
        }

		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
	}
	//PX4_INFO("_vtol_mode:%d, _torque[%.2f, %.2f, %.2f]", (int)_vtol_mode, (double)_torque_setpoint_0->xyz[0], (double)_torque_setpoint_0->xyz[1], (double)_torque_setpoint_0->xyz[2]);
}

// 检查前向转换是否完成
bool AbcVtol::isFrontTransitionCompletedBase() {
	const bool airspeed_triggers_transition =
		PX4_ISFINITE(_attc->get_calibrated_airspeed()); // 是否使用空速触发转换
	const bool minimum_trans_time_elapsed = _time_since_trans_start > getMinimumFrontTransitionTime();  // 最小转换时间是否已过
	const bool openloop_trans_time_elapsed = _time_since_trans_start > getOpenLoopFrontTransitionTime();  // 开环转换时间是否已过

	bool transition_to_fw = false;
	const float pitch = Eulerf(Quatf(_v_att->q)).theta(); // 获取当前俯仰角 [rad]
    /*
	// 与 update_transition_state 中打印的 pitch 一致：均为 _v_att 实际姿态，此处改为度便于对比
	PX4_INFO("P1 done check: airspeed=%.2f trans_sp=%.2f pitch_act=%.1f deg",
		 (double)_attc->get_calibrated_airspeed(), (double)getTransitionAirspeed(), (double)math::degrees(pitch));
    */
	// 检查俯仰角是否达到阈值
	if (pitch <= ABC_PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW) {
        if (airspeed_triggers_transition) {
            // 如果使用空速触发，需要同时满足：最小转换时间已过 AND 空速达到阈值
            transition_to_fw = minimum_trans_time_elapsed
                && _attc->get_calibrated_airspeed() >= getTransitionToFwAirspeed();
        }
        else {
            // 不使用空速触发，需要等待开环转换时间
            transition_to_fw = openloop_trans_time_elapsed;
        }
	}

	return transition_to_fw;
}

// 前向转换后混合油门
void AbcVtol::blendThrottleAfterFrontTransition(float scale) {
	// MC油门为负（-Z），FW油门为正（+X），进行混合
	_v_att_sp->thrust_body[0] = scale * _v_att_sp->thrust_body[0] + (1.f - scale) * (_last_thr_in_mc);
}

// 回退转换开始时混合油门
void AbcVtol::blendThrottleBeginningBackTransition(float scale) {
	_v_att_sp->thrust_body[2] = scale * _v_att_sp->thrust_body[2] +
								(1.f - scale) * (-_last_thr_in_fw_mode);
}

// 获取地速
float AbcVtol::get_ground_speed() {
    float velocity_ned_x = 0.0f;
    if (_local_pos->v_xy_valid && _local_pos->v_z_valid) {
        velocity_ned_x = _local_pos->vx; // 北向速度
    }
    return sqrtf(velocity_ned_x * velocity_ned_x + (_local_pos->vy * _local_pos->vy));
}
