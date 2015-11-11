#ifndef INTEGRATE_HPP_
#define INTEGRATE_HPP_
#include <Eigen/Dense>
#include <list>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <unsupported/Eigen/MatrixFunctions>


namespace pl = std::placeholders;

using namespace Eigen;

namespace shared {    

    typedef std::vector<double> state_type;

    class Integrate {
    public:
    	
    	
    	Integrate();
    	
    	void do_integration(std::vector<double> &x,
    			            std::vector<double> &control,
    			            std::vector<double> &int_times) const;
    	
    	std::vector<double> getProcessMatrices(std::vector<double> &x) const;
    	
    	void ode(const state_type &x , state_type &dxdt , double t) const;
    	
    	std::vector<double> getResult();
    	
    private:
MatrixXd getV0(const state_type &x) const; 
MatrixXd getB0(const state_type &x) const; 
MatrixXd getA0(const state_type &x) const; 
    	
    	// A fuction type of he form MatrixXd function(const state_type&) const
        typedef MatrixXd ABFuncType(const state_type&) const; 
        
        // A member function pointer for the above declared member function type
    	typedef ABFuncType Integrate::* AB_funct;
    	
    	MatrixXd power_series_(MatrixXd &m, double t, int depth) const;
    	
    	double factorial_(int num) const;
    	
    	void setupSteadyStates() const;
    	
    	std::pair<int, std::vector<double>> getClosestSteadyState(const state_type &x) const;
    	
    	std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> getClosestSteadyStateFunctions(int &idx) const;
    	
    	mutable std::vector<double> rho;
    	
    	mutable std::vector<double> result_;
    	
    	mutable std::vector<std::vector<double>> steady_states_;
    	
    	mutable std::pair<int, std::vector<double>> closest_steady_state_;
    	
    	mutable std::map<int, AB_funct> a_map_;
    	
    	mutable std::map<int, AB_funct> b_map_;
    	
    	mutable std::map<int, AB_funct> v_map_; 
    	
    	mutable std::pair<AB_funct, std::pair<AB_funct, AB_funct>> ab_functions_;
    	
    };

    
}

#endif