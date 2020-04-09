#include <Eigen/Dense>

using namespace Eigen;

class RPY {
public:
	RPY(Quaternionf _q, Vector3f _mag);

	void Filtering(const Vector3f _gyro, const float _dt);
	void Filtering(const Vector3f _gyro, const VectorXf _y, const float _dt);

private:
	VectorXf nominal_x_;
	VectorXf error_x_;

	VectorXf y_;

	MatrixXf P_;

	MatrixXf Q_;
	MatrixXf V_;

	MatrixXf F_;
	MatrixXf L_;
	MatrixXf H_;
	MatrixXf G_;

	Vector3f gyro_;

	Vector3f mag_;
	float mag_var_;

	float gyro_var_;
	float gyro_bias_var_;

	float dt_;

	void Predict_();
	void Correct_();

	void CalcQ_();
	void CalcJacobianF_();
	void CalcJacobianH_();
	void CalcJacobianG_();

	VectorXf Nominal_f_(VectorXf _nominal_x, Vector3f _gyro);
	VectorXf Measurement_h_(VectorXf _nominal_x);

	VectorXf Combine_(VectorXf _nominal_x, VectorXf _error_x);
	void Reset_();
};