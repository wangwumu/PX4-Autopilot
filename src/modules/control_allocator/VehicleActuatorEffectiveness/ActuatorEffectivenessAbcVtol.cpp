
#include "ActuatorEffectivenessAbcVtol.hpp"

using namespace matrix;

ActuatorEffectivenessAbcVtol::ActuatorEffectivenessAbcVtol(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilts(this)
{
}

bool
ActuatorEffectivenessAbcVtol::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// Allow update on NO_EXTERNAL_UPDATE so matrix is rebuilt after MC→FW transition (see earlier analysis).
	// In FW: yaw must come ONLY from tilt so that (roll,pitch,yaw,thrust) 4x4 has rank 4. If rotors also
	// contribute yaw (km), the tilt column is redundant and the matrix rank stays 3 → underactuated.
	if (_flight_phase == FlightPhase::FORWARD_FLIGHT) {
		_mc_rotors.enableYawByDifferentialThrust(true);  // yaw from tilt only → full rank 4
	}
	else {
		_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	}

	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);


	// Tilts
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Set offset such that tilts point upwards when control input == 0 (trim is 0 if min_angle == -max_angle).
	// Note that we don't set configuration.trim here, because in the case of trim == +-1, yaw is always saturated
	// and reduced to 0 with the sequential desaturation method. Instead we add it after.
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	return (rotors_added_successfully && tilts_added_successfully);
}

/*
 * ActuatorEffectivenessAbcVtol::updateSetpoint 函数用于更新执行机构的设置点
 * 根据倾斜机构的设置点计算执行机构的设置点
 * 支持倾斜机构的设置点计算
 * 支持执行机构的设置点计算
 * 支持执行机构的设置点计算
 */
void ActuatorEffectivenessAbcVtol::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	actuator_sp += _tilt_offsets;
	// TODO: dynamic matrix update

	bool yaw_saturated_positive = true;	// 偏航饱和标志，正偏航饱和标志
	bool yaw_saturated_negative = true; // 偏航饱和标志，负偏航饱和标志

	for (int i = 0; i < _tilts.count(); ++i) {

		// custom yaw saturation logic: only declare yaw saturated if all tilts are at the negative or positive yawing limit
		if (_tilts.getYawTorqueOfTilt(i) > FLT_EPSILON) {

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_positive = false;
			}

			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

		} else if (_tilts.getYawTorqueOfTilt(i) < -FLT_EPSILON) {
			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_positive = false;
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive;
}

/*
 * ActuatorEffectivenessAbcVtol::getUnallocatedControl 函数用于获取未分配的控制量
 * 根据偏航饱和标志获取未分配的控制量
 * 支持获取未分配的控制量
*/
void ActuatorEffectivenessAbcVtol::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}
