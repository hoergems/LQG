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

Integrate::Integrate() {
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

std::vector<double> Integrate::getProcessMatrices(std::vector<double> &x) const {
	double t = 0.3;
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
	MatrixXd AMatrix = (this->*A)(closest_steady_state.second);
	MatrixXd BMatrix = (this->*B)(closest_steady_state.second);	
	MatrixXd VMatrix = (this->*V)(closest_steady_state.second);	
	MatrixXd A_matrx1 = (t * AMatrix).exp();	
	MatrixXd integral = power_series_(AMatrix, t, 20);	
	MatrixXd B_matrx = A_matrx1 * integral * BMatrix;
	std::vector<double> res;
	for (size_t i = 0; i < A_matrx1.size(); i++) {
		res.push_back(A_matrx1(i));
	}
	
	for (size_t i = 0; i < B_matrx.size(); i++) {
		res.push_back(B_matrx(i));
	}
	
	for (size_t i = 0; i < VMatrix.size(); i++) {
		res.push_back(VMatrix(i));
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
std::vector<double> steady_state_0({-1, -1, -1, 0.0, 0.0, 0.0}); 
steady_states_.push_back(steady_state_0); 
a_map_.insert(std::make_pair(0, &Integrate::getA0)); 
b_map_.insert(std::make_pair(0, &Integrate::getB0)); 
v_map_.insert(std::make_pair(0, &Integrate::getV0)); 
	
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
	
	auto A = ab_functions_.first;
	auto B = ab_functions_.second.first;
	auto V = ab_functions_.second.second;	
	res = (this->*A)(closest_steady_state_.second) * state + 
		  (this->*B)(closest_steady_state_.second) * control_state;
		  (this->*V)(closest_steady_state_.second) * input_noise;	
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
                        .def("getProcessMatrices", &Integrate::getProcessMatrices)
    ;
}
MatrixXd Integrate::getA0(const state_type &x) const{ 
MatrixXd m(6, 6); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(0, 2) = 0; 
m(0, 3) = 1; 
m(0, 4) = 0; 
m(0, 5) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(1, 2) = 0; 
m(1, 3) = 0; 
m(1, 4) = 1; 
m(1, 5) = 0; 
m(2, 0) = 0; 
m(2, 1) = 0; 
m(2, 2) = 0; 
m(2, 3) = 0; 
m(2, 4) = 0; 
m(2, 5) = 1; 
m(3, 0) = 0; 
m(3, 1) = 0; 
m(3, 2) = 0; 
m(3, 3) = 100.0*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) - 1.0/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 1.0*pow(8*cos(x[1]) + 289.0L/25.0L, 2)*(47500*cos(x[1]) + 56525)/(pow((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L, 2)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(3, 4) = 100.0*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) - 1.0*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(3, 5) = 100.0*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(4, 0) = 0; 
m(4, 1) = 0; 
m(4, 2) = 0; 
m(4, 3) = -100.0*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) - 1.0*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(4, 4) = -100.0*pow(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L, 2)*pow(47500*cos(x[1]) + 56525, 2)/((640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)*(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373)) + 1.0*(47500*cos(x[1]) + 56525)/(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907); 
m(4, 5) = -100.0*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 0) = 0; 
m(5, 1) = 0; 
m(5, 2) = 0; 
m(5, 3) = -100.0*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 4) = -100.0*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 5) = -100.0*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
return m; 

} 
MatrixXd Integrate::getB0(const state_type &x) const{ 
MatrixXd m(6, 3); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(0, 2) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(1, 2) = 0; 
m(2, 0) = 0; 
m(2, 1) = 0; 
m(2, 2) = 0; 
m(3, 0) = -100*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) + 1.0/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) - pow(8*cos(x[1]) + 289.0L/25.0L, 2)*(47500*cos(x[1]) + 56525)/(pow((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L, 2)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(3, 1) = -100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) + (8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(3, 2) = -100*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(4, 0) = 100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) + (8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(4, 1) = 100*pow(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L, 2)*pow(47500*cos(x[1]) + 56525, 2)/((640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)*(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373)) - (47500*cos(x[1]) + 56525)/(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907); 
m(4, 2) = 100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 0) = 100*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 1) = 100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 2) = 100*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
return m; 

} 
MatrixXd Integrate::getV0(const state_type &x) const{ 
MatrixXd m(6, 3); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(0, 2) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(1, 2) = 0; 
m(2, 0) = 0; 
m(2, 1) = 0; 
m(2, 2) = 0; 
m(3, 0) = -100*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) + 1.0/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) - pow(8*cos(x[1]) + 289.0L/25.0L, 2)*(47500*cos(x[1]) + 56525)/(pow((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L, 2)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(3, 1) = -100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) + (8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(3, 2) = -100*((-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) + ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(4, 0) = 100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373) + (8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)); 
m(4, 1) = 100*pow(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L, 2)*pow(47500*cos(x[1]) + 56525, 2)/((640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)*(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373)) - (47500*cos(x[1]) + 56525)/(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907); 
m(4, 2) = 100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 0) = 100*(-(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)*(47500*cos(x[1]) + 56525)/(((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L)*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)) - ((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L))*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 1) = 100*(-((5.0L/2.0L)*cos(x[1]) + 189.0L/50.0L)*(8*cos(x[1]) + 289.0L/25.0L)/((19.0L/4.0L)*cos(x[1]) + 2261.0L/400.0L) + 189.0L/50.0L)*(47500*cos(x[1]) + 56525)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
m(5, 2) = 100*(640000*pow(cos(x[1]), 2) + 1300500*cos(x[1]) + 682907)/(2970000*pow(cos(x[1]), 2) + 5719500*cos(x[1]) + 3003373); 
return m; 

} 
 
}