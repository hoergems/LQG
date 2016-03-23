#include "integrate.hpp"

using namespace boost::numeric::odeint;
using std::endl;
using std::cout;

namespace shared{

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

Integrate::Integrate():
	steady_states_setup_(false),
	g_(0.0),
	f_x_(0.0),
	f_y_(0.0),
	f_z_(0.0),
	f_roll_(0.0),
	f_pitch_(0.0),
	f_yaw_(0.0),
	viscous_(),
	acceleration_limit_(10000.0),
	xstar(){
	setupSteadyStates();
	rho_vec_ = VectorXd(3);
	vel_ = VectorXd(3);	
}

void Integrate::setGravityConstant(double g) {
	g_ = g;
}

void Integrate::setExternalForce(double &f_x, 
		                         double &f_y, 
								 double &f_z,
								 double &f_roll,
								 double &f_pitch,
								 double &f_yaw) {
	f_x_ = f_x;
	f_y_ = f_y;
	f_z_ = f_z;
	f_roll_ = f_roll;
	f_pitch_ = f_pitch;
	f_yaw_ = f_yaw;
	
}

void Integrate::setAccelerationLimit(double &accelerationLimit) {
	acceleration_limit_ = accelerationLimit;
}

void Integrate::setJointDamping(std::vector<double> &viscous) {
	viscous_.clear();
	for (auto &k: viscous) {
		viscous_.push_back(k);
	} 
}

double Integrate::factorial_(int num) const {
	double factor = 1;
	for (int i = 1; i < num + 1; i++) {
		factor = factor * i;
	}
	return factor;
}

MatrixXd Integrate::power_series_(MatrixXd &m, double t, int depth) const {	
	MatrixXd A_t = -t * m;
	MatrixXd A_i = -t * m;
	MatrixXd term = MatrixXd::Identity(m.rows(), m.cols());
	for (size_t i = 1; i < depth + 1; i++) {		
		term = term + A_i / factorial_(i + 1);
		A_i = A_i * A_t;
	}
	return t * term;
}

void Integrate::calc_inverse_inertia_matrix(MatrixXd &M) const {    
	M_inv_ = M.inverse();	
}

std::vector<double> Integrate::getResult() {
	return result_;
}

MatrixXd Integrate::get_end_effector_jacobian(const state_type &x, const state_type &rho, const state_type &zeta) const {
	return getEEJacobian(x, rho, zeta);	
}

void Integrate::getProcessMatrices(std::vector<double> &x, 
    			                   std::vector<double> &rho, 
    			                   double t_e,
    			                   std::vector<MatrixXd> &matrices) const {	
	std::vector<double> zeta_nil;
	MatrixXd M = getM0(x, rho, zeta_nil);
	calc_inverse_inertia_matrix(M);
	MatrixXd AMatrix = getA0(x, rho, zeta_nil);
	MatrixXd BMatrix = getB0(x, rho, zeta_nil);	
	MatrixXd VMatrix = getV0(x, rho, zeta_nil);
	MatrixXd A_matrx1 = (t_e * AMatrix).exp();	
	MatrixXd integral = power_series_(AMatrix, t_e, 20);	
	MatrixXd B_matrx = A_matrx1 * integral * BMatrix;
	MatrixXd V_matrx = A_matrx1 * integral * VMatrix;
	
	matrices.push_back(A_matrx1);
    matrices.push_back(B_matrx);
	matrices.push_back(V_matrx);
}

std::vector<double> Integrate::getProcessMatricesVec(std::vector<double> &x, 
		                                             std::vector<double> &rho, 
												     double t_e) const {
	std::vector<MatrixXd> matrices;	
	getProcessMatrices(x, rho, t_e, matrices);
	std::vector<double> res;
	
	for (size_t i = 0; i < matrices[0].size(); i++) {		
		res.push_back(matrices[0](i));
	}
		
	for (size_t i = 0; i < matrices[1].size(); i++) {
		res.push_back(matrices[1](i));
	}
		
	for (size_t i = 0; i < matrices[2].size(); i++) {
		res.push_back(matrices[2](i));
	}
	
	return res;
}

std::vector<double> Integrate::getProcessMatricesSteadyStatesVec(std::vector<double> &x, double t_e) const {
	std::vector<double> rho_nil;
	std::vector<double> zeta_nil;
	std::pair<int, std::vector<double>> closest_steady_state = getClosestSteadyState(x);	
	for (size_t i = 0; i < closest_steady_state.second.size(); i++) {
		if (closest_steady_state.second[i] == -1) {
			closest_steady_state.second[i] = x[i];
		}		
	}
	
	std::pair<AB_funct, std::pair<AB_funct, AB_funct>> ab_functions = getClosestSteadyStateFunctions(closest_steady_state_.first);	
	auto A = ab_functions.first;
	auto B = ab_functions.second.first;
	auto V = ab_functions.second.second;
	MatrixXd AMatrix = (this->*A)(closest_steady_state.second, rho_nil, zeta_nil);
	MatrixXd BMatrix = (this->*B)(closest_steady_state.second, rho_nil, zeta_nil);	
	MatrixXd VMatrix = (this->*V)(closest_steady_state.second, rho_nil, zeta_nil);	
	MatrixXd A_matrx1 = (t_e * AMatrix).exp();	
	MatrixXd integral = power_series_(AMatrix, t_e, 20);	
	MatrixXd B_matrx = A_matrx1 * integral * BMatrix;
	
	MatrixXd B_matrx_temp = MatrixXd::Identity(B_matrx.rows(), B_matrx.cols() * 2);
	MatrixXd V_matrx_temp = MatrixXd::Identity(VMatrix.rows(), VMatrix.cols() * 2);
	
	for (size_t i = 0; i < B_matrx.rows(); i++) {
		for (size_t j = 0; j < B_matrx.cols(); j++) {
			B_matrx_temp(i, j) = B_matrx(i, j);
			V_matrx_temp(i, j) = VMatrix(i, j);
		}
	}	
	
	std::vector<double> res;
	for (size_t i = 0; i < A_matrx1.size(); i++) {
		res.push_back(A_matrx1(i));
	}
	
	for (size_t i = 0; i < B_matrx_temp.size(); i++) {
		res.push_back(B_matrx_temp(i));
	}
	
	for (size_t i = 0; i < V_matrx_temp.size(); i++) {
		res.push_back(V_matrx_temp(i));
	}
	return res;
}

void Integrate::do_integration_first_order(std::vector<double> &x,
    			                            std::vector<double> &control,
    			    			            std::vector<double> &control_error,
											std::vector<double> &nominal_state,
										    std::vector<double> &nominal_control,
										    std::vector<double> &int_times,
										    std::vector<double> &result) {
	xstar.clear();
	zetastar.clear();
	for (size_t i = 0; i < x.size(); i++) {
		xstar.push_back(nominal_state[i]);
		zetastar.push_back(0.0);
	}
	
	rhostar.clear();
	for (size_t i =0; i < control.size(); i++) {
		rhostar.push_back(nominal_control[i]);
	}
	double t0 = int_times[0];
	double te = int_times[1];
	double step_size = int_times[2];
	rho_ = control;
	zeta_ = control_error;
	
	std::vector<double> state;	
	for (size_t i = 0; i < x.size(); i++) {
		state.push_back(x[i] - nominal_state[i]);		
	}	
	
	size_t k = integrate_const(adams_bashforth<2, state_type>() ,
			                   std::bind(&Integrate::ode_first_order , this , pl::_1 , pl::_2 , pl::_3),
			                   state , t0 , te , step_size);
	result = state;
}

void Integrate::do_integration_second_order(std::vector<double> &x,
    			                            std::vector<double> &control,
    			    			            std::vector<double> &control_error,
											std::vector<double> &nominal_state,
											std::vector<double> &nominal_control,
										    std::vector<double> &int_times,
										    std::vector<double> &result) {
	xstar.clear();
	zetastar.clear();
	for (size_t i = 0; i < x.size(); i++) {
		xstar.push_back(nominal_state[i]);
		zetastar.push_back(0.0);
	}
		
	rhostar.clear();
	for (size_t i =0; i < control.size(); i++) {
		rhostar.push_back(nominal_control[i]);
	}
	double t0 = int_times[0];
	double te = int_times[1];
	double step_size = int_times[2];
	rho_ = control;
	zeta_ = control_error;
		
	std::vector<double> state;	
	for (size_t i = 0; i < x.size(); i++) {
		state.push_back(x[i] - nominal_state[i]);		
	}	
		
	size_t k = integrate_const(adams_bashforth<2, state_type>() ,
				               std::bind(&Integrate::ode_second_order , this , pl::_1 , pl::_2 , pl::_3),
				               state , t0 , te , step_size);
	result = state;
	
}

void Integrate::do_integration(std::vector<double> &x, 
		                       std::vector<double> &control,
		                       std::vector<double> &control_error,
		                       std::vector<double> &int_times,
		                       std::vector<double> &result) const {
	double t0 = int_times[0];
	double te = int_times[1];
	double step_size = int_times[2];
	rho_ = control;
	zeta_ = control_error;
	
	size_t k = integrate_const(adams_bashforth<2, state_type>() ,
		                       std::bind(&Integrate::ode , this , pl::_1 , pl::_2 , pl::_3),
		                       x , t0 , te , step_size);
	result = x;
}

void Integrate::setupSteadyStates() const {
	
}

std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> Integrate::getClosestSteadyStateFunctions(int &idx) const {
	return std::make_pair(a_map_.find(idx)->second, std::make_pair(b_map_.find(idx)->second, v_map_.find(idx)->second));
}

std::pair<int, std::vector<double>> Integrate::getClosestSteadyState(const state_type &x) const {
	int min_idx = 0;
	double dist = 0.0;
	double min_dist = 10000000.0;
	double steady_state_val = 0.0;
	for (size_t i = 0; i < steady_states_.size(); i++) {
		dist = 0.0;
		for (size_t j = 0; j < steady_states_[i].size(); j++) {
			if (steady_states_[i][j] == -1) {
				steady_state_val = x[j];
			}
			else {
				steady_state_val = steady_states_[i][j];
			}
			
			dist += std::pow(x[j] - steady_state_val, 2);
		}
		
		dist = std::sqrt(dist);
		if (dist < min_dist) {
			min_dist = dist;
			min_idx = i;
		}
	}	
	return std::make_pair(min_idx, steady_states_[min_idx]);
}

void Integrate::ode_first_order(const state_type &x , state_type &dxdt , double t) const {
	/**std::vector<double> x_true;
	for (size_t i = 0; i < x.size(); i++) {
		x_true.push_back(x[i] + xstar[i]);
	}
	MatrixXd M = getM0(x_true, rho_, zeta_);
	calc_inverse_inertia_matrix(M);
	MatrixXd res = getFirst0(x_true, rho_, zeta_);
	dxdt.clear();
	for (size_t i = 0; i < res.size(); i++) {
		dxdt.push_back(res(i));
	}*/
}

void Integrate::ode_second_order(const state_type &x , state_type &dxdt , double t) const {
	/**std::vector<double> x_true;
	for (size_t i = 0; i < x.size(); i++) {
		x_true.push_back(x[i] + xstar[i]);
	}
	MatrixXd M = getM0(x_true, rho_, zeta_);
	calc_inverse_inertia_matrix(M);
	MatrixXd res = getSec0(x_true, rho_, zeta_);
	dxdt.clear();
	for (size_t i = 0; i < res.size(); i++) {
		dxdt.push_back(res(i));
	}*/
	
}

void Integrate::ode(const state_type &x , state_type &dxdt , double t) const {	
	MatrixXd M = getM0(x, rho_, zeta_);
	calc_inverse_inertia_matrix(M);	
	
	/**for (size_t i = 0; i < x.size() / 2; i++) {
		rho_vec_(i, 0) = rho_[i];
		vel_(i, 0) = x[i + x.size() / 2];
	}
	MatrixXd C = getC0(x, rho_, zeta_);
	MatrixXd N = getN0(x, rho_, zeta_);
	MatrixXd res_low = M_inv_ * (rho_vec_ - C * vel_ - N);
	dxdt = std::vector<double>(x.size());
	for (size_t i = 0; i < x.size() / 2; i++) {
		dxdt[i] = x[i + x.size() / 2];
		dxdt[i + x.size() / 2] = res_low(i); 
	}*/
	
    MatrixXd res = getF0(x, rho_, zeta_);
	dxdt.clear();	
	for (size_t i = 0; i < res.size(); i++) {		
		dxdt.push_back(res(i));
	}
	
}

BOOST_PYTHON_MODULE(libintegrate) {
    using namespace boost::python;
    
    class_<std::vector<double> > ("v_double")
             .def(vector_indexing_suite<std::vector<double> >());
    
    class_<Integrate>("Integrate", init<>())
                        .def("doIntegration", &Integrate::do_integration)                        
                        .def("getResult", &Integrate::getResult)
                        .def("getProcessMatricesSteadyStates", &Integrate::getProcessMatricesSteadyStatesVec)
						.def("getProcessMatrices", &Integrate::getProcessMatricesVec)
						.def("setGravityConstant", &Integrate::setGravityConstant)
    ;
}
MatrixXd Integrate::getA0(const state_type &x, const state_type &rho, const state_type &zeta) const{ 
MatrixXd m(4, 4); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(0, 2) = 1; 
m(0, 3) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(1, 2) = 0; 
m(1, 3) = 1; 
m(2, 0) = M_inv_(0, 0)*(0.01*pow(x[2], 2)*cos(2*x[0]) - 0.01*pow(x[3], 2)*cos(2*x[0])) + M_inv_(0, 1)*(0.01*x[2]*x[3]*cos(2*x[0]) - x[2]*(-0.02*x[2]*cos(2*x[0]) - 0.01*x[3]*cos(2*x[0]))); 
m(2, 1) = M_inv_(0, 0)*(0.5*x[2]*x[3]*cos(x[1]) - x[3]*(-0.5*x[2]*cos(x[1]) - 0.5*x[3]*cos(x[1]))) + M_inv_(0, 1)*(-0.5*g_*sin(x[1]) - 0.5*pow(x[2], 2)*cos(x[1])); 
m(2, 2) = M_inv_(0, 0)*(-viscous_[0] + 0.01*x[2]*sin(2*x[0]) + 1.0*x[3]*sin(x[1])) + M_inv_(0, 1)*(-2*x[2]*(-0.01*sin(2*x[0]) + 0.5*sin(x[1])) + 0.01*x[3]*sin(2*x[0])); 
m(2, 3) = M_inv_(0, 0)*(1.0*x[2]*sin(x[1]) - 2*x[3]*(0.005*sin(2*x[0]) - 0.5*sin(x[1]))) + M_inv_(0, 1)*(-viscous_[1] + 0.01*x[2]*sin(2*x[0])); 
m(3, 0) = M_inv_(1, 0)*(0.01*pow(x[2], 2)*cos(2*x[0]) - 0.01*pow(x[3], 2)*cos(2*x[0])) + M_inv_(1, 1)*(0.01*x[2]*x[3]*cos(2*x[0]) - x[2]*(-0.02*x[2]*cos(2*x[0]) - 0.01*x[3]*cos(2*x[0]))); 
m(3, 1) = M_inv_(1, 0)*(0.5*x[2]*x[3]*cos(x[1]) - x[3]*(-0.5*x[2]*cos(x[1]) - 0.5*x[3]*cos(x[1]))) + M_inv_(1, 1)*(-0.5*g_*sin(x[1]) - 0.5*pow(x[2], 2)*cos(x[1])); 
m(3, 2) = M_inv_(1, 0)*(-viscous_[0] + 0.01*x[2]*sin(2*x[0]) + 1.0*x[3]*sin(x[1])) + M_inv_(1, 1)*(-2*x[2]*(-0.01*sin(2*x[0]) + 0.5*sin(x[1])) + 0.01*x[3]*sin(2*x[0])); 
m(3, 3) = M_inv_(1, 0)*(1.0*x[2]*sin(x[1]) - 2*x[3]*(0.005*sin(2*x[0]) - 0.5*sin(x[1]))) + M_inv_(1, 1)*(-viscous_[1] + 0.01*x[2]*sin(2*x[0])); 
return m; 

} 
MatrixXd Integrate::getB0(const state_type &x, const state_type &rho, const state_type &zeta) const{ 
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = M_inv_(0, 0); 
m(2, 1) = M_inv_(0, 1); 
m(3, 0) = M_inv_(1, 0); 
m(3, 1) = M_inv_(1, 1); 
return m; 

} 
MatrixXd Integrate::getV0(const state_type &x, const state_type &rho, const state_type &zeta) const{ 
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = M_inv_(0, 0); 
m(2, 1) = M_inv_(0, 1); 
m(3, 0) = M_inv_(1, 0); 
m(3, 1) = M_inv_(1, 1); 
return m; 

} 
MatrixXd Integrate::getM0(const state_type &x, const state_type &rho, const state_type &zeta) const{ 
MatrixXd m(2, 2); 
m(0, 0) = -1.0L/100.0L*pow(sin(x[0]), 2) + cos(x[1]) + 159.0L/100.0L; 
m(0, 1) = -1.0L/100.0L*pow(sin(x[0]), 2) + (1.0L/2.0L)*cos(x[1]) + 31.0L/100.0L; 
m(1, 0) = -1.0L/100.0L*pow(sin(x[0]), 2) + (1.0L/2.0L)*cos(x[1]) + 31.0L/100.0L; 
m(1, 1) = -1.0L/100.0L*pow(sin(x[0]), 2) + 31.0L/100.0L; 
return m; 

} 
MatrixXd Integrate::getF0(const state_type &x, const state_type &rho, const state_type &zeta) const{ 
VectorXd m(4); 
m(0, 0) = x[2]; 
m(1, 0) = x[3]; 
m(2, 0) = M_inv_(0, 0)*(rho[0] - viscous_[0]*x[2] + x[2]*(0.005*x[2]*sin(2*x[0]) + 0.5*x[3]*sin(x[1])) - x[3]*(-0.5*x[2]*sin(x[1]) + x[3]*(0.005*sin(2*x[0]) - 0.5*sin(x[1]))) + zeta[0]) + M_inv_(0, 1)*(0.5*g_*cos(x[1]) + rho[1] - viscous_[1]*x[3] + 0.005*x[2]*x[3]*sin(2*x[0]) - x[2]*(x[2]*(-0.01*sin(2*x[0]) + 0.5*sin(x[1])) - 0.005*x[3]*sin(2*x[0])) + zeta[1]); 
m(3, 0) = M_inv_(1, 0)*(rho[0] - viscous_[0]*x[2] + x[2]*(0.005*x[2]*sin(2*x[0]) + 0.5*x[3]*sin(x[1])) - x[3]*(-0.5*x[2]*sin(x[1]) + x[3]*(0.005*sin(2*x[0]) - 0.5*sin(x[1]))) + zeta[0]) + M_inv_(1, 1)*(0.5*g_*cos(x[1]) + rho[1] - viscous_[1]*x[3] + 0.005*x[2]*x[3]*sin(2*x[0]) - x[2]*(x[2]*(-0.01*sin(2*x[0]) + 0.5*sin(x[1])) - 0.005*x[3]*sin(2*x[0])) + zeta[1]); 
return m; 

} 
MatrixXd Integrate::getEEJacobian(const state_type &x, const state_type &rho, const state_type &zeta) const{ 
MatrixXd m(6, 2); 
m(0, 0) = -sin(x[0])*cos(x[1]) - sin(x[0]); 
m(0, 1) = -sin(x[1])*cos(x[0]); 
m(1, 0) = cos(x[0])*cos(x[1]) + cos(x[0]); 
m(1, 1) = -sin(x[0])*sin(x[1]); 
m(2, 0) = 0; 
m(2, 1) = -cos(x[1]); 
m(3, 0) = 0; 
m(3, 1) = -sin(x[0]); 
m(4, 0) = 0; 
m(4, 1) = cos(x[0]); 
m(5, 0) = 1; 
m(5, 1) = 0; 
return m; 

} 
 
}