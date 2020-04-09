#include "RPY.h"

RPY::RPY(Quaternionf _q, Vector3f _mag) {
	nominal_x_ = VectorXf(7);
	error_x_ = VectorXf(6);

	y_ = VectorXf(3);

	P_ = MatrixXf(6, 6);

	Q_ = MatrixXf(6, 6);
	V_ = MatrixXf(3, 3);

	F_ = MatrixXf(6, 6);
	L_ = MatrixXf(6, 6);
	H_ = MatrixXf(3, 6);
	G_ = MatrixXf(6, 6);

	L_ << MatrixXf(6, 6).Identity();

	nominal_x_.setZero();
	nominal_x_[0] = _q.w();
	nominal_x_[1] = _q.x();
	nominal_x_[2] = _q.y();
	nominal_x_[3] = _q.z();

	mag_var_ = 1.0e-4f;

	P_ << MatrixXf(6, 6).Identity();
	V_ << MatrixXf(3, 3).Identity() * mag_var_;

	gyro_var_ = 0.2f;
	gyro_bias_var_ = 1.0e-3f;

	dt_ = 0.05;

	mag_ = _q.toRotationMatrix() * _mag;
}

void RPY::Filtering(const Vector3f _gyro, const float _dt) {
	gyro_ = _gyro;
	dt_ = _dt;

	Predict_();
}

void RPY::Filtering(const Vector3f _gyro, const VectorXf _y, const float _dt) {
	gyro_ = _gyro;
	dt_ = _dt;

	Predict_();

	if (nominal_x_[3] == 0) {
		nominal_x_[3] += 1.0e-7f;
	}

	y_ = _y;

	Correct_;

	mag_ = _y;
	Quaternionf q(nominal_x_[0], nominal_x_[1], nominal_x_[2], nominal_x_[3]);
	mag_ = q.toRotationMatrix() * mag_;
}

void RPY::Predict_() {
	nominal_x_ = Nominal_f_(nominal_x_, gyro_);

	CalcQ_();
	CalcJacobianF_();

	P_ = F_ * P_ * F_.transpose() + L_ * Q_ * L_.transpose();
}

void RPY::Correct_() {
	CalcJacobianH_();

	MatrixXf K;
	MatrixXf S;

	S = H_ * P_ * H_.transpose() + V_;
	K = P_ * H_.transpose() * S.inverse();

	error_x_ = K * (y_ - Measurement_h_(nominal_x_));

	P_ = (MatrixXf(15, 15).Identity() - K * H_) * P_;

	Combine_(nominal_x_, error_x_);
	Reset_();
}

VectorXf RPY::Nominal_f_(VectorXf _nominal_x, Vector3f _gyro) {
	VectorXf temp_x(7);
	Vector3f gyro;

	for (int i = 0; i < 3; i++) {
		gyro[i] = _gyro[i] - _nominal_x[4 + i];
	}

	Quaternionf q(_nominal_x[0], _nominal_x[1], _nominal_x[2], _nominal_x[3]);

	Quaternionf v(0.0f, gyro[0] * dt_, gyro[1] * dt_, gyro[2] * dt_);

	float theta = v.norm();
	v.normalized();
	Vector3f u;
	u << v.x(), v.y(), v.z();
	u = u * sinf(theta * 0.5f);

	Quaternionf q_v(cosf(theta * 0.5f), u[0], u[1], u[2]);

	q = q * q_v;

	temp_x[0] = q.w();
	temp_x[1] = q.x();
	temp_x[2] = q.y();
	temp_x[3] = q.z();

	for (int i = 4; i < 7; i++) {
		temp_x[i] = _nominal_x[i];
	}

	return temp_x;
}

VectorXf RPY::Measurement_h_(VectorXf _nominal_x) {
	VectorXf temp_x(3);

	Quaternionf q(_nominal_x[0], _nominal_x[1], _nominal_x[2], _nominal_x[3]);
	temp_x = q.inverse().toRotationMatrix() * mag_;

	return temp_x;
}

VectorXf RPY::Combine_(VectorXf _nominal_x, VectorXf _error_x) {
	VectorXf temp_x(7);

	Quaternionf q(_nominal_x[0], _nominal_x[1], _nominal_x[2], _nominal_x[3]);
	Quaternionf v(0.0f, _error_x[0] * dt_, _error_x[1] * dt_, _error_x[2] * dt_);

	float theta = v.norm();
	v.normalized();
	Vector3f u;
	u << v.x(), v.y(), v.z();
	u = u * sinf(theta * 0.5f);

	Quaternionf q_v(cosf(theta * 0.5f), u[0], u[1], u[2]);

	q = q * q_v;

	temp_x[0] = q.w();
	temp_x[1] = q.x();
	temp_x[2] = q.y();
	temp_x[3] = q.z();

	for (int i = 4; i < 7; i++) {
		temp_x[i] = _nominal_x[i] + _error_x[i - 1];
	}

	return temp_x;
}

void RPY::Reset_() {
	CalcJacobianG_();
	error_x_.setZero();
	P_ = G_ * P_ * G_.transpose();
}

void RPY::CalcQ_() {
	Matrix3f gyro_cov = Matrix3f::Identity() * (gyro_var_ * dt_ * dt_);
	Matrix3f gyro_bias_cov = Matrix3f::Identity() * (gyro_bias_var_ * dt_);

	Q_ << gyro_cov, Matrix3f::Zero(),
		Matrix3f::Zero(), gyro_bias_cov;
}

void RPY::CalcJacobianF_() {
	Quaternionf q(nominal_x_[0], nominal_x_[1], nominal_x_[2], nominal_x_[3]);

	Matrix3f R;
	R = q.toRotationMatrix();

	Vector3f gyro;
	gyro[0] = gyro_[0] - nominal_x_[4];
	gyro[1] = gyro_[1] - nominal_x_[5];
	gyro[2] = gyro_[2] - nominal_x_[6];

	F_ << R.transpose() * (gyro * dt_), Matrix3f::Identity() * -dt_,
		Matrix3f::Zero(), Matrix3f::Identity();
}

void RPY::CalcJacobianH_() {
	MatrixXf H_x(3, 7);
	MatrixXf X_x(7, 6);
	MatrixXf quat_term(4, 3);
	MatrixXf mag_mat(3, 4);

	quat_term << -nominal_x_[7], -nominal_x_[8], -nominal_x_[9],
		nominal_x_[6], -nominal_x_[9], nominal_x_[8],
		nominal_x_[9], nominal_x_[6], -nominal_x_[7],
		-nominal_x_[8], nominal_x_[7], nominal_x_[6];

	mag_mat << 2 * (nominal_x_[9] * mag_[1] - nominal_x_[8] * mag_[2]),
		2 * (nominal_x_[8] * mag_[1] + nominal_x_[9] * mag_[2]),
		2 * (-nominal_x_[8] * mag_[0]),
		2 * (-nominal_x_[9] * mag_[0] + mag_[0] / nominal_x_[9] * 0.5f),

		2 * (-nominal_x_[9] * mag_[0] + nominal_x_[7] * mag_[2]),
		2 * (nominal_x_[8] * mag_[0] - nominal_x_[7] * mag_[1]),
		2 * (nominal_x_[8] * mag_[2]),
		2 * (-nominal_x_[9] * mag_[1] + mag_[1] / nominal_x_[9] * 0.5f),

		2 * (nominal_x_[8] * mag_[0] - nominal_x_[7] * mag_[1]),
		2 * (nominal_x_[9] * mag_[0] - nominal_x_[7] * mag_[2]),
		2 * (-nominal_x_[8] * mag_[2]),
		2 * (nominal_x_[8] * mag_[1] + mag_[2] / nominal_x_[9] * 0.5f);

	H_x << mag_mat, Matrix3f::Zero();

	X_x << quat_term, MatrixXf(4, 3).Zero(),
		Matrix3f::Zero(), Matrix3f::Identity();

	H_ = H_x * X_x;
}

void RPY::CalcJacobianG_() {
	Vector3f theta;
	theta << error_x_[0], error_x_[1], error_x_[2];
	theta = theta * 0.5f;
	Matrix3f skew_symmetric_theta;
	skew_symmetric_theta << 0.0f, -theta(2, 0), theta(1, 0),
		theta(2, 0), 0.0f, -theta(0, 0),
		-theta(1, 0), theta(0, 0), 0.0f;

	G_ << Matrix3f::Identity() - skew_symmetric_theta, Matrix3f::Zero(),
		Matrix3f::Zero(), Matrix3f::Identity();
}