__author__ = 'GongLi'
 
#from pulp import *
import numpy as np
import cv

def histEMD(hist1, hist2, hist1weights, hist2weights):    
    a64 = cv.fromarray(np.hstack((hist1weights, hist1)).copy())
    a32 = cv.CreateMat(a64.rows, a64.cols, cv.CV_32FC1)
    cv.Convert(a64, a32)
 
    b64 = cv.fromarray(np.hstack((hist2weights, hist2)).copy())
    b32 = cv.CreateMat(b64.rows, b64.cols, cv.CV_32FC1)
    cv.Convert(b64, b32)
    
    return cv.CalcEMD2(a32,b32,cv.CV_DIST_L2)
 
def EMD(feature1, feature2, w1, w2):
    os.environ['PATH'] += os.pathsep + '/usr/local/bin'
 
    H = feature1.shape[0]    
    I = feature2.shape[0]
 
    distances = np.zeros((H, I), dtype=object)
    for i in xrange(H):
        for j in xrange(I):
            distances[i][j] = np.linalg.norm(feature1[i] - feature2[j])
 
    # Set variables for EMD calculations
    variablesList = []
    for i in xrange(H):
        tempList = []
        for j in xrange(I):
            tempList.append(LpVariable("x"+str(i)+" "+str(j), lowBound = 0))
 
        variablesList.append(tempList)
 
    problem = LpProblem("EMD", LpMinimize)
 
    # objective function
    constraint = []
    objectiveFunction = []
    for i in xrange(H):
        for j in xrange(I):
            objectiveFunction.append(variablesList[i][j] * distances[i][j])
 
            constraint.append(variablesList[i][j])
 
    problem += lpSum(objectiveFunction)
 
 
    tempMin = min(sum(w1), sum(w2))
    problem += lpSum(constraint) == tempMin
 
    # constraints
    for i in xrange(H):
        constraint1 = [variablesList[i][j] for j in range(I)]
        problem += lpSum(constraint1) <= w1[i]
 
    for j in xrange(I):
        constraint2 = [variablesList[i][j] for i in range(H)]
        problem += lpSum(constraint2) <= w2[j]
 
    # solve
    #problem.writeLP("EMD.lp")
    problem.solve(GLPK_CMD())
 
    flow = value(problem.objective)
 
    if tempMin == 0.0:
        print "Solution is infinite"
        return -1.0
    return flow / tempMin
    
 
 
'''if __name__ == '__main__':
    feature1 = np.array([[100, 40, 22], [211,20,2], [32, 190, 150], [ 2, 100, 100]])
    feature2 = np.array([[0,0,0], [50, 100, 80], [255, 255, 255]])
 
    w1 = [0.4,0.3,0.2,0.1]
    w2 = [0.5, 0.3, 0.2]
 
 
    emdDistance = EMD(feature1, feature2, w1, w2)
    print str(emdDistance)'''