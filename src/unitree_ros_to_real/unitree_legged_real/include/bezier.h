float subterm_bez(int bez_para, int bez_k, float t_norminal);

float fcn_bezier(Eigen::MatrixXf coeff, float t_norminal);

Eigen::VectorXf diff_coeff(const Eigen::VectorXf& coeff_d);

float fcn_dbezier(Eigen::Matrix<float,6,1> coeff_d,float t_norminal);

float fcn_d2bezier(Eigen::VectorXf coeff_d,float t_norminal);