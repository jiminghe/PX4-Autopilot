/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "DifferentialVelControl.hpp"

using namespace time_literals;

DifferentialVelControl::DifferentialVelControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_velocity_status_pub.advertise();
	updateParams();
}

void DifferentialVelControl::updateParams()
{
	ModuleParams::updateParams();

	// Set up PID controller
	_pid_speed.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed.setIntegralLimit(1.f);
	_pid_speed.setOutputLimit(1.f);

	// Set up slew rate
	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_adjusted_speed_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}
}

void DifferentialVelControl::updateVelControl()
{
	updateSubscriptions();

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;
	float max_speed = _param_ro_speed_limit.get();


	// Attitude Setpoint
	if (PX4_ISFINITE(_bearing_setpoint)) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = _bearing_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		if (_param_ro_speed_red.get() > FLT_EPSILON) {
			const float course_error = fabsf(matrix::wrap_pi(_bearing_setpoint - _vehicle_yaw));
			const float speed_reduction = math::constrain(_param_ro_speed_red.get() * math::interpolate(course_error,
						      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
			max_speed = math::constrain(_param_ro_max_thr_speed.get() * (1.f - speed_reduction), 0.f, max_speed);
		}
	}

	// Throttle Setpoint
	if (PX4_ISFINITE(_speed_setpoint)) {
		const float speed_setpoint = calcSpeedSetpoint(max_speed);
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;
		rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_adjusted_speed_setpoint, _pid_speed,
				speed_setpoint, _vehicle_speed, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
				_param_ro_max_thr_speed.get(), dt);
		rover_throttle_setpoint.throttle_body_y = 0.f;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	}

	// Publish velocity controller status (logging only)
	rover_velocity_status_s rover_velocity_status;
	rover_velocity_status.timestamp = _timestamp;
	rover_velocity_status.measured_speed_body_x = _vehicle_speed;
	rover_velocity_status.adjusted_speed_body_x_setpoint = _adjusted_speed_setpoint.getState();
	rover_velocity_status.pid_throttle_body_x_integral = _pid_speed.getIntegral();
	rover_velocity_status.measured_speed_body_y = NAN;
	rover_velocity_status.adjusted_speed_body_y_setpoint = NAN;
	rover_velocity_status.pid_throttle_body_y_integral = NAN;
	_rover_velocity_status_pub.publish(rover_velocity_status);
}

void DifferentialVelControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_xyz = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
		Vector2f velocity_2d = Vector2f(velocity_xyz(0), velocity_xyz(1));
		_vehicle_speed = velocity_2d.norm() > _param_ro_speed_th.get() ? sign(velocity_2d(0)) * velocity_2d.norm() : 0.f;
	}

	if (_rover_velocity_setpoint_sub.updated()) {
		rover_velocity_setpoint_s rover_velocity_setpoint;
		_rover_velocity_setpoint_sub.copy(&rover_velocity_setpoint);
		_speed_setpoint = rover_velocity_setpoint.speed;
		_bearing_setpoint = rover_velocity_setpoint.bearing;
	}

}

float DifferentialVelControl::calcSpeedSetpoint(const float max_speed)
{
	const float heading_error = matrix::wrap_pi(_bearing_setpoint - _vehicle_yaw);

	if (_current_state == DrivingState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
		_current_state = DrivingState::SPOT_TURNING;

	} else if (_current_state == DrivingState::SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
		_current_state = DrivingState::DRIVING;
	}

	float speed_setpoint = 0.f;

	if (_current_state == DrivingState::DRIVING) {
		speed_setpoint = math::constrain(_speed_setpoint, -max_speed, max_speed);

		const float speed_setpoint_normalized = math::interpolate<float>(speed_setpoint,
							-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

		if (_rover_steering_setpoint_sub.updated()) {
			rover_steering_setpoint_s rover_steering_setpoint{};
			_rover_steering_setpoint_sub.copy(&rover_steering_setpoint);
			_normalized_speed_diff = rover_steering_setpoint.normalized_steering_setpoint;
		}

		if (fabsf(speed_setpoint_normalized) > 1.f - fabsf(
			    _normalized_speed_diff)) { // Adjust speed setpoint if it is infeasible due to the desired speed difference of the left/right wheels
			speed_setpoint = math::interpolate<float>(sign(speed_setpoint_normalized) * (1.f - fabsf(_normalized_speed_diff)), -1.f,
					 1.f,
					 - _param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
		}
	}

	return speed_setpoint;
}

bool DifferentialVelControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("differential_posVel_control_conf_invalid_speed_lim"), events::Log::Error,
				    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());
	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;
		events::send<float, float>(events::ID("differential_posVel_control_conf_invalid_speed_control"), events::Log::Error,
					   "Invalid configuration for speed control: Neither feed forward (RO_MAX_THR_SPEED) nor feedback (RO_SPEED_P) is setup",
					   _param_ro_max_thr_speed.get(), _param_ro_speed_p.get());
	}

	return ret;
}
