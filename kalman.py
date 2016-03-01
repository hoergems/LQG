import numpy as np
from scipy import linalg
from librobot import v_string, v_double, v2_double

def kalman_predict(x_tilde, u, A, B, P_t, V, M):    
    x_tilde_dash_t = np.dot(A, x_tilde) + np.dot(B, u)                 
    P_hat_t = compute_p_hat_t(A, P_t, V, M)    
    return x_tilde_dash_t, P_hat_t
    
def compute_p_hat_t(A, P_t, V, M):     
    return np.dot(A, np.dot(P_t, A.T)) + np.dot(np.dot(V, M), V.T)
    
def kalman_update(x_tilde_dash_t, z_dash_t, H, P_dash_t, W, N):
    K_t = compute_kalman_gain(H, P_dash_t, W, N)   
    x_tilde_t = compute_state_estimate(x_tilde_dash_t, z_dash_t, H, K_t)
    P_t = compute_P_t(K_t, H, P_dash_t)    
    return x_tilde_t, P_t
    
def compute_kalman_gain(H, P_dash_t, W, N):    
    return np.dot(P_dash_t, np.dot(H.T, linalg.inv(np.dot(H, np.dot(P_dash_t, H.T)) + np.dot(W, np.dot(N, W.T)))))
    
def compute_state_estimate(x_tilde_dash_t, z_dash_t, H, K_t):
    return x_tilde_dash_t + np.dot(K_t, (z_dash_t - np.dot(H, x_tilde_dash_t))) 
    
def compute_P_t(K_t, H, P_hat_t):
    KtH = np.dot(K_t, H)           
    return np.dot((np.identity(KtH.shape[0]) - KtH), P_hat_t)  
    
def compute_gain(A, B, C, D, l):
    S = np.copy(C)
    A = A[0:l][::-1]
    B = B[0:l][::-1]    
    Ls = []   
    for i in xrange(l):
        L = np.dot(-linalg.inv(np.dot(B[i].T, np.dot(S, B[i])) + D), np.dot(B[i].T, np.dot(S, A[i])))      
        #L = -np.dot(linalg.inv(np.dot(B[i].T, np.dot(S, B[i])) + D)), np.dot(B[i].T, np.dot(S, A[i]))
        Ls.append(L)
        S = C + np.dot(A[i].T, np.dot(S, A[i])) + np.dot(A[i].T, np.dot(S, np.dot(B[i], L)))
        ''''L = -np.dot(linalg.pinv(np.add(np.dot(np.dot(np.transpose(B[i]), S), B[i]), D)), np.dot(np.dot(np.transpose(B[i]), S), A[i]))
        Ls.append(L)
        S = np.add(C, np.add(np.dot(np.dot(np.transpose(A[i]), S), A[i]), np.dot(np.dot(np.dot(np.transpose(A[i]), S), B[i]), L)))'''    
    Ls = Ls[::-1]         
    return Ls

def predict_state(robot,
                  x_tilde,
                  xs0,                  
                  us, 
                  control_duration,
                  simulation_step_size,
                  A,
                  B,
                  V, 
                  M,
                  P_t):        
    """
    Predidcts the state at the next time step using an extended kalman filter
    """
    x_estimate = x_tilde + xs0
    u = us
    u_dash = np.array([0.0 for i in xrange(len(u))])
    current_state = v_double()
    current_state[:] = x_estimate
    control = v_double()
    control[:] = u
    control_error = v_double()
    control_error[:] = [0.0 for i in xrange(len(u))]
    result = v_double()
    robot.propagate(current_state,
                    control,
                    control_error,
                    simulation_step_size,
                    control_duration,
                    result)
    x_predicted = np.array([result[i] for i in xrange(len(result))])
        
    x_tilde_dash, P_dash = kalman_predict(x_tilde, u_dash, A, B, P_t, V, M)
    return (x_predicted, P_dash)

def get_linear_model_matrices(robot, 
                              state_path, 
                              control_path, 
                              control_durations, 
                              dynamic_problem,
                              M,
                              H,
                              W,
                              N):
        """ Get the linearized model matrices along a given nominal path
        """
        As = []
        Bs = []
        Vs = []
        Ms = []
        Hs = []
        Ws = []
        Ns = []
        if dynamic_problem:
            for i in xrange(len(state_path)):
                state = v_double()
                control = v_double()
                state[:] = state_path[i]
                control[:] = control_path[i]
                A = robot.getProcessMatrices(state, control, control_durations[i])       
                Matr_list = [A[j] for j in xrange(len(A))]
                A_list = np.array([Matr_list[j] for j in xrange(len(state)**2)])
                start_index = len(state)**2
                B_list = np.array([Matr_list[j] for j in xrange(start_index, 
                                                                start_index + (len(state) * (len(state) / 2)))])
                start_index = start_index + (len(state) * (len(state) / 2))
                V_list = np.array([Matr_list[j] for j in xrange(start_index, 
                                                                start_index + (len(state) * (len(state) / 2)))])
                A_Matr = A_list.reshape(len(state), len(state)).T
                B_Matr = B_list.reshape(len(state)/ 2, len(state)).T
                V_Matr = V_list.reshape(len(state) / 2, len(state)).T
                
                As.append(A_Matr)
                Bs.append(B_Matr)
                Vs.append(V_Matr)                
                Ms.append(M)
                Hs.append(H)
                Ws.append(W)
                Ns.append(N)
        else:
            for i in xrange(len(state_path) + 1):
                As.append(self.A)
                Bs.append(self.B)
                Vs.append(self.V)
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
        return As, Bs, Vs, Ms, Hs, Ws, Ns