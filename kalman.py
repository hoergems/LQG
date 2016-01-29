import numpy as np
from scipy import linalg

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