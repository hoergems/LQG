#ifndef INTEGRATE_HPP_
#define INTEGRATE_HPP_
#include <Eigen/Dense>
#include <list>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
//#include <boost/numeric/odeint/stepper/runge_kutta_fehlberg78.hpp>

#include <boost/numeric/odeint/stepper/euler.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <unsupported/Eigen/MatrixFunctions>


namespace pl = std::placeholders;

using namespace Eigen;

namespace shared {    

    typedef std::vector<double> state_type;

    class Integrate {
    public:   	
    	
    	Integrate();
    	
    	/**
    	 * Sets the gravity constant
    	 */
    	void setGravityConstant(double g);
    	
    	/**
    	 * Sets the external forces acting on the end-effector
    	 */
    	void setExternalForce(double &f_x, double &f_y, double &f_z);
    	
    	/**
    	 * Sets the viscous joint damping constants
    	 */
    	void setJointDamping(std::vector<double> &viscous);
    	
    	void do_integration(std::vector<double> &x,
    			            std::vector<double> &control,
    			            std::vector<double> &control_error,
    			            std::vector<double> &int_times,
    			            std::vector<double> &result) const;
    	
    	void getProcessMatrices(std::vector<double> &x, 
    			                std::vector<double> &rho, 
    			                double t_e,
    			                std::vector<MatrixXd> &matrices) const;
    	
    	MatrixXd get_F(const state_type &x, const state_type &rho, const state_type &zeta) const;
    	
    	MatrixXd get_end_effector_jacobian(const state_type &x, const state_type &rho, const state_type &zeta) const;
    	
    	std::vector<double> getProcessMatricesSteadyStatesVec(std::vector<double> &x, double t_e) const;
    	
    	std::vector<double> getProcessMatricesVec(std::vector<double> &x, 
    			                                  std::vector<double> &rho, 
    										      double t_e) const;
    	
    	void ode(const state_type &x , state_type &dxdt , double t) const;
    	
    	std::vector<double> getResult();
    	
    private:
MatrixXd getEEJacobian(const state_type &x, const state_type &rho, const state_type &zeta) const; 
MatrixXd getF0(const state_type &x, const state_type &rho, const state_type &zeta) const; 
MatrixXd getV0(const state_type &x, const state_type &rho, const state_type &zeta) const; 
MatrixXd getB0(const state_type &x, const state_type &rho, const state_type &zeta) const; 
MatrixXd getA0(const state_type &x, const state_type &rho, const state_type &zeta) const; 
    	
    	// A fuction type of he form MatrixXd function(const state_type&) const
        typedef MatrixXd ABFuncType(const state_type&, const state_type&, const state_type&) const; 
        
        // A member function pointer for the above declared member function type
    	typedef ABFuncType Integrate::* AB_funct;
    	
    	MatrixXd power_series_(MatrixXd &m, double t, int depth) const;
    	
    	double factorial_(int num) const;
    	
    	void setupSteadyStates() const;
    	
    	std::pair<int, std::vector<double>> getClosestSteadyState(const state_type &x) const;
    	
    	std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> getClosestSteadyStateFunctions(int &idx) const;
    	
    	/**
    	 * Gravity constant
    	 */
    	mutable double g_;
    	
    	/**
    	 * External force components
    	 */
    	mutable double f_x_;
    	mutable double f_y_;
    	mutable double f_z_;
    	
    	/**
    	 * Viscous joint dampings
    	 */
    	mutable std::vector<double> viscous_;
    	
    	mutable std::vector<double> rho_;
    	
    	mutable std::vector<double> zeta_;
    	
    	mutable std::vector<double> xstar;
    	
    	mutable std::vector<double> rhostar;
    	
    	mutable std::vector<double> result_;
    	
    	mutable std::vector<std::vector<double>> steady_states_;
    	
    	mutable std::pair<int, std::vector<double>> closest_steady_state_;
    	
    	mutable std::map<int, AB_funct> a_map_;
    	
    	mutable std::map<int, AB_funct> b_map_;
    	
    	mutable std::map<int, AB_funct> v_map_; 
    	
    	mutable std::pair<AB_funct, std::pair<AB_funct, AB_funct>> ab_functions_;
    	
    	mutable bool steady_states_setup_;
    };

    
}

#endif