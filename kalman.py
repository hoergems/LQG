import numpy as np
from scipy import linalg

def kalman_predict(x_tilde, u, A, B, P_t, V, M):
    x_tilde_dash_t = np.add(np.dot(A, x_tilde), np.dot(B, u))               
    P_hat_t = compute_p_hat_t(A, P_t, V, M)    
    return x_tilde_dash_t, P_hat_t
    
def compute_p_hat_t(A, P_t, V, M):
    return np.add(np.dot(np.dot(A, P_t), np.transpose(A)), np.dot(np.dot(V, M), np.transpose(V)))        
    
def kalman_update(x_tilde_dash_t, z_dash_t, H, P_dash_t, W, N, num_links):
    K_t = compute_kalman_gain(H, P_dash_t, W, N)    
    x_tilde_t = compute_state_estimate(x_tilde_dash_t, z_dash_t, H, K_t)
    P_t = compute_P_t(K_t, H, P_dash_t, num_links)    
    return x_tilde_t, P_t
    
def compute_kalman_gain(H, P_dash_t, W, N):
    return np.dot(np.dot(P_dash_t, np.transpose(H)), linalg.pinv(np.add(np.dot(np.dot(H, P_dash_t), np.transpose(H)), np.dot(np.dot(W, N), np.transpose(W)))))        
    
def compute_state_estimate(x_tilde_dash_t, z_dash_t, H, K_t):        
    return np.add(x_tilde_dash_t, np.dot(K_t, np.subtract(z_dash_t, np.dot(H, x_tilde_dash_t))))
    
def compute_P_t(K_t, H, P_hat_t, num_links):
    I = np.identity(num_links)    
    m1 = np.subtract(I, np.dot(K_t, H))    
    return np.dot(np.subtract(np.identity(num_links), np.dot(K_t, H)), P_hat_t)        
    
def compute_gain(A, B, C, D, l):
    S = np.copy(C)
    Ls = []        
    for i in xrange(l):
        L = -np.dot(linalg.pinv(np.add(np.dot(np.dot(np.transpose(B), S), B), D)), np.dot(np.dot(np.transpose(B), S), A))
        Ls.append(L)
        S = np.add(C, np.add(np.dot(np.dot(np.transpose(A), S), A), np.dot(np.dot(np.dot(np.transpose(A), S), B), L)))    
    Ls = Ls[::-1]       
    return Ls