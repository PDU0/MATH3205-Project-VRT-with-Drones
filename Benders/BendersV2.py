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
# CWD = r'C:\Users\Loong\OneDrive\Year3-Semester2\MATH3205\math3205project'
# testPath = CWD + r'\test'
# sys.path.insert(1, testPath)
# from load_test import load_test

# Global vars
OBJBND = False
CUTGCS = 0
CUTFEAS = 0
CUTOPTI = 0
 

def BendersV2(test, name, time_limit, CWD, param='new'):
    # Benders approch
    global CUTGCS, CUTFEAS, CUTOPTI
    
    t = {'ph': None,
         'tr': None,
         'w': None,
         'Benders': None}
    
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
     
    # Helper functions
    def adjacencyMatrix(V, E):
        # Construct an adjacency matrix for graph G(V, E)
        # V is a list of vertices (nodes) and E is a list of edges (arcs)
        M = np.zeros(shape=(len(V),len(V)))
        for (i,j) in E:
            M[V.index(i)][V.index(j)] = 1
        return M
        
    def generateComponent(label):
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
    
    # Preprocessing
    def TSP(Zp):
        # Travelling salesman problem
        def TSPCallback(model,where):
            # GCS Separation (Algorithm 1 from the paper)
            if where==GRB.Callback.MIPSOL:
                XV = model.cbGetSolution(X)
                AA = [(i, j) for (i, j) in Az if XV[i,j]>0]
                GG = csr_matrix(adjacencyMatrix(list(Nst), AA))
                # Find all SCC on G
                _, label = connected_components(GG,directed=True,connection='strong')
                S = generateComponent(label)
                for s in S:
                    if len(s) >= 2:
                        model.cbLazy(quicksum(X[i,j] for i in s for j in s if i!=j)
                                      <=len(s)-1)
        Zp['t'] = Zp['s']
        Az = [(i,j) for (i,j) in A if i in Zp and j in Zp]
        tsp = Model('TSP')
        
        X = {(i,j): tsp.addVar(vtype=GRB.BINARY) 
              for (i,j) in Az}
        
        # print(X)
        tsp.setObjective(quicksum(tv[i,j]*X[i,j] for (i,j) in Az), GRB.MINIMIZE)
        VisitOnce = {j: tsp.addConstr(
                        quicksum(X[i,j] for i in Zp if i!=j 
                                 if not (i=='s' and j=='t') if i!='t')==1)
                      for j in Zp if j!='s'}
        LeaveOnce = {i: tsp.addConstr(
                        quicksum(X[i,j] for j in Zp if i!=j 
                                 if not (i=='s' and j=='t') if j!='s')==1)
                      for i in Zp if i!='t'}
        LeaveStart = tsp.addConstr(quicksum(X['s',j] for j in Zp if j not in ['s','t'])==1)
        EnterEnd = tsp.addConstr(quicksum(X[i,'t'] for i in Zp if i not in ['s','t'])==1)
        
        tsp.setParam('OutputFlag', 0)
        tsp.setParam('LazyConstraints', 1)
        tsp.optimize(TSPCallback)
        # print(tsp.Status==GRB.OPTIMAL)
        # print(tsp.objVal)
        XV = {(i,j): X[i,j].x for (i,j) in Az}
        for arc in A:
            if arc not in XV:
                XV[arc] = 0
        return tsp.objVal, XV
    
    
    def primalHeuristic():
        # Returns an initial incumbent solu, and a lower bound on truck route
        print('Begin Primal Heuristic')
        P = [i for i in range(1, len(N))] # [1,2,...,|N|-1]
        for p in P:
            print('-Solving LW-p with p={}...'.format(p))
            # LW-p problem for primal heuristic
            lwp = Model('LW-p')
    
            Z = {i: lwp.addVar(vtype=GRB.BINARY) for i in Ns}
            Y = {(i,j): lwp.addVar(vtype=GRB.BINARY) for (i,j) in A}
            H = {(i,j,l): lwp.addVar(vtype=GRB.BINARY) for (i,j) in A for l in L}
            W = {i: lwp.addVar(vtype=GRB.CONTINUOUS) for i in Ns}
    
            lwp.setObjective(quicksum(W[i] for i in Ns), GRB.MINIMIZE)
    
            BatteryLimit = {l: 
                lwp.addConstr(quicksum(bd[i,j,l]*H[i,j,l] 
                                        for i in Ns for j in N if i!=j)<=BMax[l]) 
                              for l in L} 
    
            WaitForDrone = {(i,l): 
                lwp.addConstr(W[i]>=quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
                              for i in Ns for l in L}
                
            c3 = {(i,j): lwp.addConstr(Y[i,j]==quicksum(H[i,j,l] for l in L)) 
                  for (i,j) in A}
    
            c4 = {(i,j): lwp.addConstr(Z[i]>=Y[i,j]) for i in Ns for j in N if i!=j}
    
            c5 = lwp.addConstr(quicksum(Z[i] for i in N)==p)
    
            c6 = {j: lwp.addConstr(quicksum(Y[i,j] for i in Ns if i!=j)+Z[j]>=1) 
                  for j in N}
    
            c7 = Z['s']==1
            
            lwp.setParam('OutputFlag', 0)
            lwp.optimize()
            # if solution if feasible 
            if lwp.Status == GRB.OPTIMAL:
                ZV = {i: Z[i].x for i in Ns}
                HV = {(i,j,l): H[i,j,l].x for (i,j) in A for l in L}
                WV = {i: W[i].x for i in Ns}
                print('--Solution of LW-p is feasible')
                Wp = lwp.objVal
                Zp = {i: Ns[i] for i in Ns if Z[i].x>=0.8}
                Tp, XV = TSP(Zp)
                # print(Wp,Tp)
                ZHat = Wp + Tp
                pMin = p
                print('Result of Primal Heuristic: Z^={}, p_={}'.format(ZHat,pMin))
                break
            
            
        return ZHat, pMin, XV, ZV, HV, WV, Wp
         
    ph0 = time.time()
    ZHat, pMin, XV, ZV, HV, WV, Wp = primalHeuristic()
    ph1 = time.time()
    t['ph'] = ph1-ph0
    
    def RootRelaxPp(p):
        # Solve the root relaxation of the original MIP with restriction on 
        # truck route length
        
        m = Model('P-p')
        
        X = {(i, j): m.addVar(vtype=GRB.BINARY) for (i, j) in A}
        H = {(i, j, l): m.addVar(vtype=GRB.BINARY) for (i, j) in A for l in L}
        V = {i: m.addVar(vtype=GRB.CONTINUOUS) for i in Nst}
        W = {i: m.addVar(vtype=GRB.CONTINUOUS) for i in Nst}
    
        m.setObjective(quicksum(tv[i,j]*X[i,j] for (i, j) in A)+quicksum(W[i] for i in Ns), 
                        GRB.MINIMIZE)
    
        StartDepot = m.addConstr(quicksum(X['s', j] for j in N)==1)
    
        EndDepot = m.addConstr(quicksum(X[i, 't'] for i in N)==1)
    
        ConservationOfFlow = {i: 
            m.addConstr(quicksum(X[i,j] for j in Nt if i!=j)==
                        quicksum(X[j,i] for j in Ns if i!=j)) 
            for i in N}
    
        NoSubTour = {(i,j): 
            m.addConstr(V[i]-V[j]<=M*(1-X[i,j])-1) 
            for (i,j) in A}
    
        OnceVisitPerNode = {j: 
            m.addConstr(quicksum(X[i,j] for i in Ns if i!=j)+
                        quicksum(H[i,j,l] for i in Ns if i!=j for l in L)==1) 
            for j in N}
            
        DispatchDrone = {i: 
            m.addConstr(M*quicksum(X[i,j] for j in Nt if i!=j if not (i=='s' and j=='t'))>=
                        quicksum(H[i,j,l] for j in N if i!=j for l in L)) 
            for i in Ns}
            
        BatteryLimit = {l: 
            m.addConstr(quicksum(bd[i,j,l]*H[i,j,l] for i in Ns for j in N if i!=j)<=
                        BMax[l]) 
            for l in L}  
            
        WaitForDrone = {(i,l): 
            m.addConstr(W[i]>=quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
            for i in Ns for l in L}
            
        InitOrder = m.addConstr(V['s']==0)
        
        LengthBounding = m.addConstr(
                        quicksum(X[i,j] for i in N for j in Nt if i!=j)==p)
        
        def ppCallback(model, where):
            # Terminate if we move out of initial nodes
            global OBJBND
            if where == GRB.Callback.MIPNODE:
                nodecount = model.cbGet(GRB.Callback.MIPNODE_NODCNT)
                if nodecount > 0:
                    OBJBND = model.cbGet(GRB.Callback.MIPNODE_OBJBND)
                    model.terminate()
    
        m.setParam('OutputFlag', 0)
        m.optimize(ppCallback)
        return OBJBND
        
    
    def TruckBounding(ZHat, pMin):
        # Finds an upper bound of the truck route length 
        
        print('Begin Truck Route Length Bounding')
        p = pMin + round((len(N)-pMin)/3)
        print('--Initial Truck route length: {} | length of N: {}'.format(p, len(N)))
        while p <= len(N)-1:
            ZMin = RootRelaxPp(p)
            print('--Objval of P-{}: {}'.format(p,ZMin))
            if ZMin <= ZHat:
                break
            else:
                p = pMin + round((p-pMin+1)/3)
                print('Change p to ', p)
        pHat = p
        print('---')
        for p in [i for i in range(pHat+1, len(N))]:
            ZMin = RootRelaxPp(p)
            print('--Objval of P-{}: {}'.format(p,ZMin))
            if ZMin >= ZHat:
                pMax = p-1
                print('pMax found: ',pMax)
                break       
        return pMax
        
    tr0 = time.time()
    pMax = TruckBounding(ZHat, pMin)
    # pMax = len(N) # dummy for testing benders
    # pMin = 1
    tr1 = time.time()
    t['tr'] = tr1 - tr0
    
    w0 = time.time()
    WLow = {}
    print('-Generating Wp...')
    # for p in range(pMin, len(N)+1):
    for p in range(pMin, pMax+1):
        print('--Solving LW-{}'.format(p))
        lwp = Model('LW-p')
    
        Z = {i: lwp.addVar(vtype=GRB.BINARY) for i in Ns}
        Y = {(i,j): lwp.addVar(vtype=GRB.BINARY) for (i,j) in A}
        H = {(i,j,l): lwp.addVar(vtype=GRB.BINARY) for (i,j) in A for l in L}
        W = {i: lwp.addVar(vtype=GRB.CONTINUOUS) for i in Ns}
        
        lwp.setObjective(quicksum(W[i] for i in Ns), GRB.MINIMIZE)
    
        BatteryLimit = {l: 
            lwp.addConstr(quicksum(bd[i,j,l]*H[i,j,l] 
                                   for i in Ns for j in N if i!=j)<=BMax[l]) 
                          for l in L} 
    
        WaitForDrone = {(i,l): 
            lwp.addConstr(W[i]>=quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
                          for i in Ns for l in L}
            
        c3 = {(i,j): lwp.addConstr(Y[i,j]==quicksum(H[i,j,l] for l in L)) 
              for (i,j) in A}
    
        c4 = {(i,j): lwp.addConstr(Z[i]>=Y[i,j]) for i in Ns for j in N if i!=j}
    
        c5 = lwp.addConstr(quicksum(Z[i] for i in N)==p)
    
        c6 = {j: lwp.addConstr(quicksum(Y[i,j] for i in Ns if i!=j)+Z[j]>=1) 
              for j in N}
    
        c7 = Z['s']==1
        
        def lwpCallback(model, where):
            # Root relaxation
            global OBJBND
            if where == GRB.Callback.MIPNODE:
                nodecount = model.cbGet(GRB.Callback.MIPNODE_NODCNT)
                if nodecount > 0:
                    OBJBND = model.cbGet(GRB.Callback.MIPNODE_OBJBND)
                    model.terminate()
                    
        lwp.setParam('OutputFlag', 0)
        lwp.optimize(lwpCallback)
        
        # WLow[p] = lwp.objVal
        WLow[p] = OBJBND
        
    # print(WLow)
    w1 = time.time()
    t['w'] = w1-w0
    
    print('Preprocessing complete: pMax={}, pMin={}'.format(pMax, pMin))
    b0 = time.time()
    def WLB(p):
        # Finds the lower bound of the total waitin time
        
        lwp = Model('LW-p')
    
        Z = {i: lwp.addVar(vtype=GRB.BINARY) for i in Ns}
        Y = {(i,j): lwp.addVar(vtype=GRB.BINARY) for (i,j) in A}
        H = {(i,j,l): lwp.addVar(vtype=GRB.BINARY) for (i,j) in A for l in L}
        W = {i: lwp.addVar(vtype=GRB.CONTINUOUS) for i in Ns}
    
        lwp.setObjective(quicksum(W[i] for i in Ns), GRB.MINIMIZE)
    
        BatteryLimit = {l: 
            lwp.addConstr(quicksum(bd[i,j,l]*H[i,j,l] 
                                   for i in Ns for j in N if i!=j)<=BMax[l]) 
                          for l in L} 
    
        WaitForDrone = {(i,l): 
            lwp.addConstr(W[i]>=quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
                          for i in Ns for l in L}
            
        c3 = {(i,j): lwp.addConstr(Y[i,j]==quicksum(H[i,j,l] for l in L)) 
              for (i,j) in A}
    
        c4 = {(i,j): lwp.addConstr(Z[i]>=Y[i,j]) for i in Ns for j in N if i!=j}
    
        c5 = lwp.addConstr(quicksum(Z[i] for i in N)==p)
    
        c6 = {j: lwp.addConstr(quicksum(Y[i,j] for i in Ns if i!=j)+Z[j]>=1) 
              for j in N}
    
        c7 = Z['s']==1
        
        def lwpCallback(model, where):
            global OBJBND
            if where == GRB.Callback.MIPNODE:
                nodecount = model.cbGet(GRB.Callback.MIPNODE_NODCNT)
                if nodecount > 0:
                    OBJBND = model.cbGet(GRB.Callback.MIPNODE_OBJBND)
                    model.terminate()
                    
        lwp.setParam('OutputFlag', 0)
        lwp.optimize(lwpCallback)
        
        # WLow[p] = lwp.objVal
        WLow[p] = OBJBND
        return OBJBND
    
    ####### BMP ###############
    print('\nBegin Solving the Benders Master Problem...')
    BMP = Model('BMP')
    # Vars
    # 1 if truck gose from i to j, 0 otherwise
    X = {(i,j): BMP.addVar(vtype=GRB.BINARY) for (i,j) in A}
    # 1 if truck visits node i, 0 otherwose
    Z = {i: BMP.addVar(vtype=GRB.BINARY) for i in N}
    # Total waiting time
    Theta = BMP.addVar()
    
    BMP._x = list(X.values())
    BMP._z = list(Z.values())
    BMP._theta = Theta
    
    #Feed the incumbent solution
    for arc in X:
        X[arc].start = XV[arc]
    for node in N:
        Z[node].start = ZV[node]
    Theta.start = Wp
    
    # Objective
    BMP.setObjective(quicksum(tv[i,j]*X[i,j] for (i,j) in A)+Theta,
                      GRB.MINIMIZE)
    
    # Constraints
    StartDepot = BMP.addConstr(quicksum(X['s', j] for j in N)==1)
    
    EndDepot = BMP.addConstr(quicksum(X[i, 't'] for i in N)==1)
    
    ConservationOfFlow = {i: 
        BMP.addConstr(quicksum(X[i,j] for j in Nt if i!=j)==
                      quicksum(X[j,i] for j in Ns if i!=j)) 
        for i in N}
        
    Visit = {i: 
        BMP.addConstr(quicksum(X[i,j] for j in Nt if i!=j)==Z[i])
        for i in N}
        
    RouteLengthLB = BMP.addConstr(quicksum(X[i,j] 
                                           for i in N for j in Nt if i!=j)
                                  >=pMin)
    
    RouteLengthUB = BMP.addConstr(quicksum(X[i,j]
                                           for i in N for j in Nt if i!=j)
                                  <=pMax)
    
    NonNeg = BMP.addConstr(Theta >= 0)
    
    # Constraints from the BSP
    H = {(i,j,l): BMP.addVar(lb=0) for (i,j) in A for l in L}
    
    W = {i: BMP.addVar(vtype=GRB.CONTINUOUS, lb=0) for i in Ns}
    
    for (i,j) in A:
        for l in L:
            H[i,j,l].start = HV[i,j,l]
    for i in Ns:
        W[i].start = WV[i]
    
    DroneDelivery = {j:
        BMP.addConstr(quicksum(H[i,j,l] for i in Ns for l in L if i!=j)
                      ==1-Z[j]) 
        for j in N}
    
    VisitAndDeliver = {(i,j,l): 
        BMP.addConstr(Z[i]>=H[i,j,l]) 
        for i in N for j in Nt for l in L if i!=j}
        
    BatteryLimit = {l: 
        BMP.addConstr(quicksum(bd[i,j,l]*H[i,j,l] 
                                for i in Ns for j in N if i!=j)<=BMax[l]) 
        for l in L}  
    
    WaitTime = BMP.addConstr(Theta>=quicksum(W[i] for i in Ns))    
    
    WaitForDrone = {(i,l): 
        BMP.addConstr(W[i]>=
                      quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
        for i in Ns for l in L}
        
    StrengthenBound = {(i,j): 
        BMP.addConstr(W[i]>=min(td[i,j,l] for l in L)*quicksum(H[i,j,l] for l in L)) 
        for (i,j) in A if j!='t'}
    
    def Callback(model, where):
        global CUTFEAS, CUTOPTI, CUTGCS
        if where == GRB.Callback.MIPSOL:
            XV = model.cbGetSolution(X)
            ZV = model.cbGetSolution(Z)
            ThetaV = model.cbGetSolution(Theta)
            N0 = [i for i in N if ZV[i]<0.2]
            N1 = [i for i in N if ZV[i]>0.8]
            # print('Theta',ThetaV)
            
            # Sec4.1: GCS separation
            AA = [(i, j) for (i, j) in A if XV[i,j]>0]
            GG = csr_matrix(adjacencyMatrix(list(Nst), AA))
            # Find all SCC on G
            _, label = connected_components(GG,directed=True,connection='strong')
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
                        CUTGCS += 1
              
            ####### BSP ###############
            BSP = Model('BSP')
            # Vars
            H = {(i,j,l): BSP.addVar(vtype=GRB.BINARY) for (i,j) in A for l in L}
            W = {i: BSP.addVar() for i in Ns}
            # Objective
            BSP.setObjective(quicksum(W[i] for i in Ns), GRB.MINIMIZE)
    
            # BSP Constraints
            BatteryLimit = {l: 
                BSP.addConstr(quicksum(bd[i,j,l]*H[i,j,l] 
                                        for i in Ns for j in N if i!=j)<=BMax[l]) 
                for l in L}  
                
            WaitForDrone = {(i,l): 
                BSP.addConstr(W[i]>=
                              quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
                for i in Ns for l in L}
            # More BSP constraints
            DroneDelivery = {j:
                BSP.addConstr(quicksum(H[i,j,l] for i in Ns for l in L if i!=j)
                              >=1-ZV[j]) 
                for j in N}
            NoTruckNoDrone = {i:
                BSP.addConstr(quicksum(H[i,j,l] for j in N for l in L if i!=j)
                              <=len(N)*ZV[i])
                for i in N}
            NonNeg = {i: BSP.addConstr(W[i]>=0) for i in Ns}
            BSP.setParam('OutputFlag', 0)
            BSP.optimize()
            # for c in BSP.getConstrs():
            #     BSP.remove(c)
            
            if BSP.Status != GRB.Status.OPTIMAL:
                # Infeasible
                # print('BSP Infeasible')
                model.cbLazy(quicksum(1-Z[i] for i in N1)+ 
                              quicksum(Z[i] for i in N0)>=1)
                CUTFEAS += 1
            elif ThetaV <= BSP.objVal - 1e-4:
                # Optimality cut
                # print('BSP obj',BSP.objVal)
                # print('Optimal')
                pv = sum(ZV[i] for i in N)
                if pv < pMax:
                    Omega = max((BSP.objVal-WLow[round(q)])/(q-pv) 
                        for q in range(round(pv)+1, pMax+1))
                else:
                    print('Calculating LW-{}'.format(pv))
                    Omega = BSP.objVal-WLB(pv)
                BMP.cbLazy(Theta>=BSP.objVal-Omega*
                            quicksum(Z[i] for i in N0))
                CUTOPTI += 1
                
        elif where == GRB.Callback.MIPNODE:
            status = model.cbGet(GRB.Callback.MIPNODE_STATUS)
            if status == GRB.OPTIMAL:
                # GCS again
                XV = model.cbGetNodeRel(model._x)
                ZV = model.cbGetNodeRel(model._z)
                ThetaV = model.cbGetNodeRel(model._theta)
                N0 = [i for i in N if ZV[list(N).index(i)]<0.2]
                N1 = [i for i in N if ZV[list(N).index(i)]>0.8]
                # print('Theta',ThetaV)
                
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
                            CUTGCS += 1
     
    logName = CWD + r'\Benders\BendersTestLog\BendersV2-%s.txt' %(name)
    logfile = open(logName, 'w+')
    BMP.setParam('LogFile', logName)     
    BMP.setParam('TimeLimit', time_limit-sum(list(t.values())[:3]))
    BMP.setParam('LazyConstraints', 1) 
    BMP.optimize(Callback)
    b1 = time.time()
    t['Benders'] = b1 - b0
    logfile.close()
    
    ZV = {i: Z[i].x for i in Z}
    
    ####### Solve the BSP again to obtain the drone route ####################
    print('-Generating Drone Paths...')
    BSP = Model('BSP')
    # Vars
    H = {(i,j,l): BSP.addVar(vtype=GRB.BINARY) for (i,j) in A for l in L}
    W = {i: BSP.addVar() for i in Ns}
    # Objective
    BSP.setObjective(quicksum(W[i] for i in Ns), GRB.MINIMIZE)
    
    # BSP Constraints
    BatteryLimit = {l: 
        BSP.addConstr(quicksum(bd[i,j,l]*H[i,j,l] 
                                for i in Ns for j in N if i!=j)<=
                    BMax[l]) 
        for l in L}  
        
    WaitForDrone = {(i,l): 
        BSP.addConstr(W[i]>=
                      quicksum(td[i,j,l]*H[i,j,l] for j in N if i!=j)) 
        for i in Ns for l in L}
        
    DroneDelivery = {j:
        BSP.addConstr(quicksum(H[i,j,l] for i in Ns for l in L if i!=j)
                      >=1-ZV[j]) 
        for j in N}
    NoTruckNoDrone = {i:
        BSP.addConstr(quicksum(H[i,j,l] for j in N for l in L if i!=j)
                      <=len(N)*ZV[i])
        for i in N}
    NonNeg = {i: BSP.addConstr(W[i]>=0) for i in Ns}
    BSP.setParam('OutputFlag', 0)
    BSP.optimize()
    
    
    
    # Plot truck routs
    verts = []
    for (i,j) in A:
        if X[i,j].x > 0.9:
            print(i,j)
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
    plt.title('Benders-{}'.format(name))

    plotPath = CWD + r'\Benders\BendersTestLog\Benders-{}.png'.format(name)
    plt.savefig(plotPath)
    plt.show()
    
    for l in L:
        print('Drone ', l, ' Battery Consumption: ', 
              sum(bd[i,j,l]*H[i,j,l].x for i in Ns for j in N if i!=j))
    
    numGCS = CUTGCS
    numFeas = CUTFEAS
    numOpti = CUTOPTI
    CUTGCS = 0
    CUTFEAS = 0
    CUTOPTI = 0
    
    return t, BMP.objVal, numGCS, numFeas, numOpti

def main(): # If you want batch testing, go to test.py
    ######## Please change ###################################################
    CWD = r'YOUR/WORKING/DIR'
    ##########################################################################
    TestDir = CWD + r'\test\testCases'
    testPath = CWD + r'\test'
    sys.path.insert(1, testPath)
    from load_test import load_test
    
    #List of test sets ['P', 'Solomon', 'A', 'B']
    timeLimit = 3600
    ##### Find a test, and change me #########################################
    name = 'P-n16-k8'
    test = load_test('P', TestDir)[name]
    ##########################################################################
    t0 = time.time()
    t,_,_,_,_ = BendersV2(test, name, timeLimit, CWD)
    t1 = time.time()
    print('Total Time: {}s'.format(t1-t0))
    print(t)
        
    

if __name__ == '__main__':
    main()
