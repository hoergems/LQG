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
	steady_states_setup_(false){
	setupSteadyStates();
}

double Integrate::factorial_(int num) const {
	double factor = 1;
	for (int i = 1; i < num + 1; i++) {
		factor = factor * i;
	}
	return factor;
}

MatrixXd Integrate::power_series_(MatrixXd &m, double t, int depth) const {	
	MatrixXd A_t = -m * t;
	MatrixXd A_i = -m * t;
	MatrixXd term = MatrixXd::Identity(m.rows(), m.cols());
	for (size_t i = 1; i < depth + 1; i++) {		
		term = term + A_i / factorial_(i + 1);
		A_i = A_i * A_t;
	}
	return t * term;
}

std::vector<double> Integrate::getResult() {
	return result_;
}

std::vector<double> Integrate::getProcessMatrices(std::vector<double> &x, 
		                                          std::vector<double> &rho, 
												  double t_e) const {	
	MatrixXd AMatrix = getA0(x, rho);
	MatrixXd BMatrix = getB0(x, rho);	
	MatrixXd VMatrix = getV0(x, rho);	
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

std::vector<double> Integrate::getProcessMatricesSteadyStates(std::vector<double> &x, double t_e) const {
	std::vector<double> rho_nil;
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
	MatrixXd AMatrix = (this->*A)(closest_steady_state.second, rho_nil);
	MatrixXd BMatrix = (this->*B)(closest_steady_state.second, rho_nil);	
	MatrixXd VMatrix = (this->*V)(closest_steady_state.second, rho_nil);	
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

void Integrate::do_integration(std::vector<double> &x, 
		                       std::vector<double> &control,
		                       std::vector<double> &int_times) const {
	double t0 = int_times[0];
	double te = int_times[1];
	double step_size = int_times[2];
	
	rho.clear();
	for (size_t i = 0; i < control.size(); i++) {
		rho.push_back(control[i]);
	}
	
	closest_steady_state_ = getClosestSteadyState(x);
	for (size_t i = 0; i < closest_steady_state_.second.size(); i++) {
		if (closest_steady_state_.second[i] == -1) {
			closest_steady_state_.second[i] = x[i];
		}		
	}
	ab_functions_ = getClosestSteadyStateFunctions(closest_steady_state_.first);
	
	size_t k = integrate_const(runge_kutta4<state_type>() ,
		                       std::bind(&Integrate::ode , this , pl::_1 , pl::_2 , pl::_3),
		                       x , t0 , te , step_size);
	for (size_t i = 0; i < x.size(); i++) {
		result_.push_back(x[i]);
	}
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

void Integrate::ode(const state_type &x , state_type &dxdt , double t) const {
	VectorXd res(x.size());
	
	VectorXd state(x.size());
	VectorXd control_state(x.size() / 2);
	VectorXd input_noise(x.size() / 2);
	for (size_t i = 0; i < x.size() / 2; i++) {
		state[i] = x[i] - closest_steady_state_.second[i];
		state[i + x.size() / 2] = x[i + x.size() / 2] - closest_steady_state_.second[i + x.size() / 2];
		control_state[i] = rho[i];
		input_noise[i] = 0.0;
	}
	std::vector<double> rho_nil;
	auto A = ab_functions_.first;
	auto B = ab_functions_.second.first;
	auto V = ab_functions_.second.second;	
	res = (this->*A)(closest_steady_state_.second, rho_nil) * state + 
		  (this->*B)(closest_steady_state_.second, rho_nil) * control_state;
		  (this->*V)(closest_steady_state_.second, rho_nil) * input_noise;	
 	dxdt.clear(); 	
	for (size_t i = 0; i < x.size(); i++) {		
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
                        .def("getProcessMatricesSteadyStates", &Integrate::getProcessMatricesSteadyStates)
						.def("getProcessMatrices", &Integrate::getProcessMatrices)
    ;
}
MatrixXd Integrate::getA0(const state_type &x, const state_type &rho) const{ 
MatrixXd m(4, 4); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(0, 2) = 1; 
m(0, 3) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(1, 2) = 0; 
m(1, 3) = 1; 
m(2, 0) = 0; 
m(2, 1) = rho[0]*(-1250*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*pow(625*pow(cos(x[1]), 2) - 896, 2)) + 2500*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 2*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 3)*(625*pow(cos(x[1]), 2) - 896)) + ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + sin(x[1])/pow(cos(x[1]) + 39.0L/25.0L, 2)) + 1250*rho[1]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*pow(625*pow(cos(x[1]), 2) - 896, 2)) - 2500*rho[1]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + rho[1]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 1.0L/2.0L*rho[1]*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 0.5*pow(x[2], 2)*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 625.0*pow(x[2], 2)*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*pow(sin(x[1]), 2)*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*pow(625*pow(cos(x[1]), 2) - 896, 2)) + 1250.0*pow(x[2], 2)*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*pow(sin(x[1]), 2)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 0.5*pow(x[2], 2)*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*pow(sin(x[1]), 2)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 0.25*pow(x[2], 2)*(2500*cos(x[1]) + 3900)*pow(sin(x[1]), 2)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 1.0*x[2]*(-1250*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*pow(625*pow(cos(x[1]), 2) - 896, 2)) + 2500*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 2*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 3)*(625*pow(cos(x[1]), 2) - 896)) + ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + sin(x[1])/pow(cos(x[1]) + 39.0L/25.0L, 2)) - 1250.0*x[3]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*pow(625*pow(cos(x[1]), 2) - 896, 2)) + 2500.0*x[3]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 1.0*x[3]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 0.5*x[3]*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + (0.5*x[2]*x[3]*sin(x[1]) + 0.5*x[3]*(x[2] + x[3])*sin(x[1]))*(-1250*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*pow(625*pow(cos(x[1]), 2) - 896, 2)) + 2500*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 2*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 3)*(625*pow(cos(x[1]), 2) - 896)) + ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + sin(x[1])/pow(cos(x[1]) + 39.0L/25.0L, 2)) + (0.5*x[2]*x[3]*cos(x[1]) + 0.5*x[3]*(x[2] + x[3])*cos(x[1]))*(-pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 1.0/(cos(x[1]) + 39.0L/25.0L)); 
m(2, 2) = -1.0*x[2]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + 1.0*x[3]*(-pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 1.0/(cos(x[1]) + 39.0L/25.0L))*sin(x[1]) + 1.0*pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 1.0/(cos(x[1]) + 39.0L/25.0L); 
m(2, 3) = (-pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 1.0/(cos(x[1]) + 39.0L/25.0L))*(0.5*x[2]*sin(x[1]) + 0.5*x[3]*sin(x[1]) + 0.5*(x[2] + x[3])*sin(x[1])) - 1.0*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 0) = 0; 
m(3, 1) = 1250*rho[0]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*pow(625*pow(cos(x[1]), 2) - 896, 2)) - 2500*rho[0]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + rho[0]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 1.0L/2.0L*rho[0]*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 1250*rho[1]*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/pow(625*pow(cos(x[1]), 2) - 896, 2) + 2500*rho[1]*sin(x[1])/(625*pow(cos(x[1]), 2) - 896) + 0.5*pow(x[2], 2)*(2500*cos(x[1]) + 3900)*cos(x[1])/(625*pow(cos(x[1]), 2) - 896) + 625.0*pow(x[2], 2)*(2500*cos(x[1]) + 3900)*pow(sin(x[1]), 2)*cos(x[1])/pow(625*pow(cos(x[1]), 2) - 896, 2) - 1250.0*pow(x[2], 2)*pow(sin(x[1]), 2)/(625*pow(cos(x[1]), 2) - 896) - 1250.0*x[2]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*pow(625*pow(cos(x[1]), 2) - 896, 2)) + 2500.0*x[2]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 1.0*x[2]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 0.5*x[2]*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + 1250.0*x[3]*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/pow(625*pow(cos(x[1]), 2) - 896, 2) - 2500.0*x[3]*sin(x[1])/(625*pow(cos(x[1]), 2) - 896) + 1250*(0.5*x[2]*x[3]*sin(x[1]) + 0.5*x[3]*(x[2] + x[3])*sin(x[1]))*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])*cos(x[1])/((cos(x[1]) + 39.0L/25.0L)*pow(625*pow(cos(x[1]), 2) - 896, 2)) - 2500*(0.5*x[2]*x[3]*sin(x[1]) + 0.5*x[3]*(x[2] + x[3])*sin(x[1]))*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + (0.5*x[2]*x[3]*sin(x[1]) + 0.5*x[3]*(x[2] + x[3])*sin(x[1]))*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) - 1.0L/2.0L*(0.5*x[2]*x[3]*sin(x[1]) + 0.5*x[3]*(x[2] + x[3])*sin(x[1]))*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + (0.5*x[2]*x[3]*cos(x[1]) + 0.5*x[3]*(x[2] + x[3])*cos(x[1]))*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 2) = 1.0*x[2]*(2500*cos(x[1]) + 3900)*sin(x[1])/(625*pow(cos(x[1]), 2) - 896) + 1.0*x[3]*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*sin(x[1])/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) - 1.0*((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 3) = ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)*(0.5*x[2]*sin(x[1]) + 0.5*x[3]*sin(x[1]) + 0.5*(x[2] + x[3])*sin(x[1]))/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)) + 1.0*(2500*cos(x[1]) + 3900)/(625*pow(cos(x[1]), 2) - 896); 
return m; 

} 
MatrixXd Integrate::getB0(const state_type &x, const state_type &rho) const{ 
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = -pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 1.0/(cos(x[1]) + 39.0L/25.0L); 
m(2, 1) = ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 0) = ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 1) = -(2500*cos(x[1]) + 3900)/(625*pow(cos(x[1]), 2) - 896); 
return m; 

} 
MatrixXd Integrate::getV0(const state_type &x, const state_type &rho) const{ 
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = -pow((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L, 2)*(2500*cos(x[1]) + 3900)/(pow(cos(x[1]) + 39.0L/25.0L, 2)*(625*pow(cos(x[1]), 2) - 896)) + 1.0/(cos(x[1]) + 39.0L/25.0L); 
m(2, 1) = ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 0) = ((1.0L/2.0L)*cos(x[1]) + 7.0L/25.0L)*(2500*cos(x[1]) + 3900)/((cos(x[1]) + 39.0L/25.0L)*(625*pow(cos(x[1]), 2) - 896)); 
m(3, 1) = -(2500*cos(x[1]) + 3900)/(625*pow(cos(x[1]), 2) - 896); 
return m; 

} 
 
}