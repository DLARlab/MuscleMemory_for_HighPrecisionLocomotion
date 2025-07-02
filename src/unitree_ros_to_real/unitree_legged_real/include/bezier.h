float subterm_bez(int bez_para, int bez_k, float t_norminal);

float fcn_bezier(Eigen::MatrixXf coeff, float t_norminal);

Eigen::Matrix<float,5,1> diff_coeff(Eigen::Matrix<float,6,1> coeff_d);

float fcn_dbezier(Eigen::Matrix<float,6,1> coeff_d,float t_norminal);
