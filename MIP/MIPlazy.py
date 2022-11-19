import numpy as np
from numpy import linalg as LA
from gurobipy import *

from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import time
import csv
import sys

CUTCOUNT = 0

def mipLazy(test, name, time_limit, CWD, param='new'):
    # MIP with lazy GCS constrs 
    
    global CUTCOUNT
    
    testPath = CWD + r'\test'
    sys.path.insert(1, testPath)
    from load_test import load_test
    
    N = test['N']
    Ns = test['Ns']
    Nt = test['Nt']
    Nst = test['Nst']
    
    # M = len(Nst)+1
    # Set of arcs
    A = [(i, j) for i in Ns for j in Nt if i!=j if not (i=='s' and j=='t')]
    # M is just a large number
    M = len(A)+1
    # EPS is just a small number
    EPS = 0.8
    
    # Parameters for Solomon and Augerat
    if param == 'new':
        # Truck speed
        VT = 1 
        # Truck service time
        ST = 10
        # Drone speed (standard)
        VD = 2.5
        # Drone service time
        SD = 5
        BMax = [100, 50]
        
        # Set of drones
        L = [0, 1]
        # Drone speed param
        Alpha = [0.4, 0.2]
        
        Beta = [0.4, 0.2]
        
        # Travel time for truck to go from node i to node j
        def tTruck(i, j):
            return (LA.norm(np.subtract(np.array(Ns[i]), np.array(Nt[j]))) / VT)
        
        # Travel time for standard drone to go from node i to node j
        def tDrone(i, j, l):
            return tTruck(i,j)*Alpha[l]
        
        def serviceDrone(i, l):
            return Beta[l]*ST
        
        # Delivery time for a truck (travel+service)
        tv = {(i,j): 
              round((tTruck(i, j)+ST),2) 
              for i in Ns for j in N if i!=j}
        for i in N:
            # No service time when returns to the depot
            tv[i, 't'] = round(tTruck(i, 't'),2)
        
        # Battery (Flight duration) consumption of drone
        bd = {(i, j, l): 
              round(2*tDrone(i, j, l)+serviceDrone(j, l),2)
              for i in Ns for j in N if i!=j for l in L}
        
        # Delivery time for drone l in L
        td = {(i, j, l): 
              round(2*tDrone(i, j, l)+serviceDrone(j, l),2)
              for i in Ns for j in N if i!=j for l in L}
    else:
        # Truck speed
        VT = 1 
        # Truck service time
        ST = 10
        # Drone speed (standard)
        VD = 2.5
        # Drone service time
        SD = 5
        
        # Set of drones
        L = [0, 1]
        # Drone speed param
        Alpha = [1, 1.5]
        
        # Drone max duration
        # BMax = 100
        # BMax = [100/Alpha[l] for l in L]
        BMax = [100, 130]
        
        # Travel time for truck to go from node i to node j
        def tTruck(i, j):
            return LA.norm(np.subtract(np.array(Ns[i]), np.array(Nt[j]))) / VT
        
        # Travel time for standard drone to go from node i to node j
        def tDrone(i, j):
            return LA.norm(np.subtract(np.array(Ns[i]), np.array(N[j]))) / VD
        
        # Delivery time for a truck (travel+service)
        tv = {(i,j): 
              round((tTruck(i, j)+ST),2) 
              for i in Ns for j in N if i!=j}
        for i in N:
            # No service time when returns to the depot
            tv[i, 't'] = round(tTruck(i, 't'),2)
        
        # Delivery time for a standard drone (2*travel+service)
        bStd = {(i, j): 
                round((2*tDrone(i, j)+SD),2) 
                for i in Ns for j in N if i!=j}
        
        # Battery (Flight duration) consumption of drone
        bd = {(i, j, l): 
              round((bStd[i,j]*Alpha[l]),2) 
              for i in Ns for j in N if i!=j for l in L}
        
        # Delivery time for drone l in L
        td = {(i, j, l): 
              round((2*tDrone(i, j)/Alpha[l]+SD),2) 
              for i in Ns for j in N if i!=j for l in L}
            
    m = Model('MIP')
    
    # Vars
    X = {(i, j): m.addVar(vtype=GRB.BINARY) for (i, j) in A}
    H = {(i, j, l): m.addVar(vtype=GRB.BINARY) for (i, j) in A for l in L}
    # V = {i: m.addVar(vtype=GRB.CONTINUOUS) for i in Nst}
    W = {i: m.addVar(vtype=GRB.CONTINUOUS) for i in Nst}
    
    m._x = list(X.values())
    
    # Objective
    m.setObjective(quicksum(tv[i,j]*X[i,j] for (i, j) in A)+quicksum(W[i] for i in Ns), 
                   GRB.MINIMIZE)
    
    # Constr
    StartDepot = m.addConstr(quicksum(X['s', j] for j in N)==1)
    
    EndDepot = m.addConstr(quicksum(X[i, 't'] for i in N)==1)
    
    ConservationOfFlow = {i: 
        m.addConstr(quicksum(X[i,j] for j in Nt if i!=j)==
                    quicksum(X[j,i] for j in Ns if i!=j)) 
        for i in N}
    
    # NoSubTour = {(i,j): 
    #     m.addConstr(V[i]-V[j]<=M*(1-X[i,j])-1) 
    #     for (i,j) in A}
    
    OnceVisitPerNode = {j: 
        m.addConstr(quicksum(X[i,j] for i in Ns if i!=j)+
                    quicksum(H[i,j,l] for i in Ns if i!=j for l in L)==1) 
        for j in N}
        
    DispatchDrone = {i: 
        m.addConstr(M*quicksum(X[i,j] for j in Nt if i!=j 
                               if not (i=='s' and j=='t'))>=
                    quicksum(H[i,j,l] for j in N if i!=j for l in L)) 
        for i in Ns}
        
    BatteryLimit = {l: 
        m.addConstr(quicksum(bd[i,j,l]*H[i,j,l] for i in Ns for j in N if i!=j)<=
                    BMax[l]) 
        for l in L}  
        
    WaitForDrone = {(i,l): 
        m.addConstr(W[i]>=quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
        for i in Ns for l in L}
    
    def adjacencyMatrix(V, E):
        # Construct an adjacency matrix for graph G(V, E)
        # V is a list of vertices (nodes) and E is a list of edges (arcs)
        M = np.zeros(shape=(len(V),len(V)))
        for (i,j) in E:
            M[V.index(i)][V.index(j)] = 1
        return M
        
    def generateComponent(label):
        # Generate a list of strongly connnnected components (as a list)
        S = {}
        for i in range(len(label)):
            if label[i] not in S:
                # print('opps')
                S[label[i]] = [list(Nst)[i]]
            else:
                # print('!!!!!!!!!!!!!!!!!!!!!!!!')
                S[label[i]].append(list(Nst)[i])
        # print(S)
        return list(S.values())
    
    # def delta(s):
        # return [(i, j) for (i, j) in A if i in s and j not in s]
    
    def Callback(model,where):
        global CUTCOUNT
        # GCS Separation (Algorithm 1 from the paper)
        if where==GRB.Callback.MIPSOL:
            XV = model.cbGetSolution(X)
            AA = [(i, j) for (i, j) in A if XV[i,j]>0]
            # print(AA)
            GG = csr_matrix(adjacencyMatrix(list(Nst), AA))
            # print(GG.toarray())
            # Find all SCC on G
            _, label = connected_components(GG,directed=True,connection='strong')
            # print(label)
            S = generateComponent(label)
            for s in S:
                for k in s:
                    deltaS = [(i, j) for (i, j) in A if i in s and j not in s]
                    deltaK = [(i, j) for (i, j) in A if i==k and j!=k]
                    v = sum(XV[i,j] for (i,j) in deltaK)- \
                        sum(XV[i,j] for (i,j) in deltaS)
                    if v > EPS:
                        model.cbLazy(quicksum(X[i,j] for (i,j) in deltaS)>=
                                      quicksum(X[i,j] for (i,j) in deltaK))
                        CUTCOUNT += 1
                        
        elif where == GRB.Callback.MIPNODE:
            status = model.cbGet(GRB.Callback.MIPNODE_STATUS)
            if status == GRB.OPTIMAL:
                XV = model.cbGetNodeRel(model._x)
                # Sec4.1: GCS separation
                AA = [(i, j) for (i, j) in A if XV[list(A).index((i,j))]>0]
                GG = csr_matrix(adjacencyMatrix(list(Nst), AA))
                # Find all SCC on G
                _, label = connected_components(GG,directed=True,connection='strong')
                S = generateComponent(label)
                for s in S:
                    for k in s:
                        deltaS = [(i, j) for (i, j) in A if i in s and j not in s]
                        deltaK = [(i, j) for (i, j) in A if i==k and j!=k]
                        v = sum(XV[list(A).index((i,j))] for (i,j) in deltaK)- \
                            sum(XV[list(A).index((i,j))] for (i,j) in deltaS)
                        if v > EPS:
                            model.cbLazy(quicksum(X[i,j] for (i,j) in deltaS)>=
                                          quicksum(X[i,j] for (i,j) in deltaK))
                            CUTCOUNT += 1
    # # A Callback function for another form of GCS cut, same result
    # def Callback(model,where):
    #     # GCS Separation (Algorithm 1 from the paper)
    #     if where==GRB.Callback.MIPSOL:
    #         XV = model.cbGetSolution(X)
    #         AA = [(i, j) for (i, j) in A if XV[i,j]>0]
    #         GG = csr_matrix(adjacencyMatrix(list(Nst), AA))
    #         # Find all SCC on G
    #         _, label = connected_components(GG,directed=True,connection='strong')
    #         S = generateComponent(label)
    #         for s in S:
    #             if len(s) >= 2:
    #                 # print(2)
    #                 model.cbLazy(quicksum(X[i,j] for i in s for j in s if i!=j)
    #                               <=len(s)-1)
    
    logName = CWD + r'\MIP\MIPLazyTestLog\MIPLazy-%s.txt' %(name)
    logfile = open(logName, 'w+')
    m.setParam('LogFile', logName)     
    m.setParam('TimeLimit', time_limit)               
    m.setParam('LazyConstraints', 1)
    m.setParam('BranchDir', 0)
    m.optimize(Callback)
    logfile.close()
    
    
    # Plot truck routs
    verts = []
    for (i,j) in A:
        if X[i,j].x > 0.9:
            # print(i,j)
            verts.append(tuple(Nst[i]))
            verts.append(tuple(Nst[j]))
            
    # print(verts)
    codes = []
    for i in range(int(len(verts)/2)):
        # print(i)
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO) 
    # print(codes)
    path = Path(verts, codes)
    fig, ax = plt.subplots()
    patch = patches.PathPatch(path, color='blue', lw=2)
    ax.add_patch(patch)
    
    xs, ys = zip(*verts)
    ax.plot(xs, ys, 'x', lw=2, color='black', ms=10)
    
    # Plot drone 0 routs
    verts2 = []
    for i in Ns:
        for j in N:
            for l in [0]:
                if i!=j and H[i,j,l].x > 0.9:
                    # print(i,j)
                    verts2.append(tuple(Nst[i]))
                    verts2.append(tuple(Nst[j]))
                    
    # print(verts)
    codes2 = []
    for i in range(int(len(verts2)/2)):
        # print(i)
        codes2.append(Path.MOVETO)
        codes2.append(Path.LINETO) 
    # print(codes)
    path2 = Path(verts2, codes2)
    patch2 = patches.PathPatch(path2, color='orange', lw=1)
    ax.add_patch(patch2)
    
    xs2, ys2 = zip(*verts2)
    ax.plot(xs2, ys2, '.', lw=2, color='orange', ms=8)
    
    # Plot drone 1 routs
    verts3 = []
    for i in Ns:
        for j in N:
            for l in [1]:
                if i!=j and H[i,j,l].x > 0.9:
                    # print(i,j)
                    verts3.append(tuple(Nst[i]))
                    verts3.append(tuple(Nst[j]))
                    
    # print(verts)
    codes3 = []
    for i in range(int(len(verts3)/2)):
        # print(i)
        codes3.append(Path.MOVETO)
        codes3.append(Path.LINETO) 
    # print(codes)
    path3 = Path(verts3, codes3)
    patch3 = patches.PathPatch(path3, color='red', lw=1)
    ax.add_patch(patch3)
    
    xs3, ys3 = zip(*verts3)
    ax.plot(xs3, ys3, '.', lw=2, color='red', ms=8)
    
    for n in Ns:
        x, y = Ns[n]
        ax.text(x-1.5,y+2, n)
    
    ax.set_xlim(min(c[0] for c in Ns.values())-5, max(c[0] for c in Ns.values())+5)
    ax.set_ylim(min(c[1] for c in Ns.values())-5, max(c[1] for c in Ns.values())+5)
    plt.title('MIPLazy-{}'.format(name))

    plotPath = CWD + r'\MIP\MIPLazyTestLog\MIPLazy-{}.png'.format(name)
    plt.savefig(plotPath)
    plt.show()
        
    for l in L:
        print('Drone ', l, ' Battery Consumption: ', 
              sum(bd[i,j,l]*H[i,j,l].x for i in Ns for j in N if i!=j))
        
    numCut = CUTCOUNT
    CUTCOUNT = 0
    return m.objVal, numCut

def main(): # If you want batch testing, go to test.py
    ######## Please change ###################################################
    CWD = r'C:\Users\Loong\OneDrive\Year3-Semester2\MATH3205\46272784_45285714_MATH3205_Project'
    ##########################################################################
    testPath = CWD + r'\test'
    sys.path.insert(1, testPath)
    from load_test import load_test
    
    #List of test sets ['P', 'Solomon', 'A', 'B']
    timeLimit = 3600
    ##### Find a test, and change me #########################################
    name = 'P-n16-k8'
    test = load_test('P')[name]
    ##########################################################################
    t0 = time.time()
    _, cut = mipLazy(test, name, timeLimit, CWD)
    t1 = time.time()
    print(t1-t0)
    print("Number of cuts: ", cut)
    

if __name__ == '__main__':
    main()
