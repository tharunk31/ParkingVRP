import numpy as np
import cvxpy as cp
import gurobipy as gp
from gurobipy import GRB
import math
import pandas as pd

np.random.seed(25)
A = 5
E = int(A*(A-1)/2)
K = 3

ExpectedViolations = 5
NViolations = np.random.poisson(ExpectedViolations,(100,A))
NViolationsProbability = np.zeros((100,A))
for it in range(100):
    for i in range(A):
        NViolationsProbability[it,i] = np.exp(-1*ExpectedViolations) * (ExpectedViolations ** NViolations[it,i]) / math.factorial(NViolations[it,i])

c = np.random.uniform(1,5,(K,A,A))
Time = np.zeros((K,A,A))
for k in range(K):
    for i in range(A):
        c[k][i][i] = 0
        
Time = c * 2
# print(Time)
        
StartCost = 7
OverTimeCost = 30
Fine = 40

FineProcessingTime = 2
ShiftLength = 60

NIterations =  100        


def Problem(Type):
    if Type == "WaitAndSee":
        
        WaitAndSeeObj = [0]*NIterations
        for l in range(NIterations):
            print("\n\n\nIteration:", l)
            m1 = gp.Model("ParkingVRP")
            m1.params.LogToConsole = 0

            x = m1.addVars(K, A, A, name="X", vtype=GRB.BINARY)

            u = m1.addVars(K,A, name="U", vtype=GRB.INTEGER,lb=1,ub=A)
            
            # Z = 0

            RoutingCost = 0

            for k in range(K):
                for i in range(A):
                    for j in range(A):
                        RoutingCost += c[k,i,j] * x[k,i,j] 
                    RoutingCost += StartCost * x[k,0,i]
            InflowConstraints = m1.addConstrs(x.sum(k,0,'*') <= 1 for k in range(K) ) #for i in range(1,A))

            OutflowConstraints = m1.addConstrs(x.sum(k,'*',0) <= 1 for k in range(K)) #for j in range(A))

            # StartFlowConstraints = m1.addConstrs(x.sum(k,0,'*')  == 1 for k in range(K))

            FlowBalanceConstraints = m1.addConstrs(x.sum(k,'*',j) - x.sum(k,j,'*') == 0 for k in range(K) for j in range(1,A))

            # EndFlowConstraints = m1.addConstrs(x.sum(k,'*',0)  == 1 for k in range(K)) 

            StayProhibitionConstraint = m1.addConstrs(x[k,i,i]==0 for k in range(K) for i in range(1,A))

            for k in range(K):
                for i in range(A):
                    for j in range(A):
                        if i != j:
                            if i != 0:
                                if j != 0:
                                    m1.addConstr(u[k,j] >= u[k,i] + 1 - (A) * (1 - x[k,i,j]))
                               
            Cost = RoutingCost
            # Z = NViolations[l]
            y = m1.addVars(K,A,name='Y',vtype=GRB.CONTINUOUS)
            
            OverTime = m1.addVars(K, name = 'Tau', vtype=GRB.CONTINUOUS)
            SlackTime = m1.addVars(K, name = "TauPrime", vtype = GRB.CONTINUOUS)
            
            TravelTime = [0]*K
            TimeAtLot = [0]*K
            VisitCountConstraints = m1.addConstrs(y[k,i] == x.sum(k,i,'*') for k in range(K) for i in range(A))
            
            for k in range(K):
                for i in range(A):
                    for j in range(A):
                        TravelTime[k] += x[k,i,j] * Time[k,i,j]
            
            
                        
            for k in range(K):
                Cost += OverTimeCost * OverTime[k]
                for i in range(A):
                    
                    Cost -= Fine*NViolations[l][i]*y[k,i]
                    TimeAtLot[k] += y[k,i] * FineProcessingTime * NViolations[l][i]
                    
            for k in range(K):
                m1.addConstr(TravelTime[k] + TimeAtLot[k] == ShiftLength + OverTime[k] - SlackTime[k])
                    
        
        
            m1.setObjective(Cost, GRB.MINIMIZE)

            m1.write('C:\PhD IE\Optimization under Uncertainty\parkingVRP.lp')

            m1.optimize()
            if m1.Status == GRB.OPTIMAL:
                obj = m1.getObjective()
                solution = m1.getAttr('X',x)
                print("The objective is :",obj.getValue())
                WaitAndSeeObj[l] = obj.getValue()
                for k in range(K):
                    print("Order of routing for %s:" % k)
                    next = np.zeros(A)
                    Routing = [0]
                    for i in range(A):
                        for j in range(A):
                            if solution[k,i,j] > 0:
                                next[i] = j
                    current = 0
                    while next[current] != 0:
                        current = int(next[current])
                        Routing.append(int(next[current]))
            
                    print(Routing)                  
        
        return WaitAndSeeObj    
            
    if Type == "ExpectedValue":
        m1 = gp.Model("ParkingVRP")
        m1.params.LogToConsole = 0
        ExpectedValueObj = 0

        x = m1.addVars(K, A, A, name="X", vtype=GRB.BINARY)

        u = m1.addVars(K,A, name="U", vtype=GRB.INTEGER,lb=1,ub=A)
        
        # Z = 0

        RoutingCost = 0

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    RoutingCost += c[k,i,j] * x[k,i,j] 
                RoutingCost += StartCost * x[k,0,i]
        InflowConstraints = m1.addConstrs(x.sum(k,0,'*') <= 1 for k in range(K) ) #for i in range(1,A))

        OutflowConstraints = m1.addConstrs(x.sum(k,'*',0) <= 1 for k in range(K)) #for j in range(A))

        # StartFlowConstraints = m1.addConstrs(x.sum(k,0,'*')  == 1 for k in range(K))

        FlowBalanceConstraints = m1.addConstrs(x.sum(k,'*',j) - x.sum(k,j,'*') == 0 for k in range(K) for j in range(1,A))

        # EndFlowConstraints = m1.addConstrs(x.sum(k,'*',0)  == 1 for k in range(K)) 

        StayProhibitionConstraint = m1.addConstrs(x[k,i,i]==0 for k in range(K) for i in range(1,A))

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    if i != j:
                        if i != 0:
                            if j != 0:
                                m1.addConstr(u[k,j] >= u[k,i] + 1 - (A) * (1 - x[k,i,j]))
                            
        Cost = RoutingCost
        # Z = NViolations[l]
        y = m1.addVars(K,A,name='Y',vtype=GRB.CONTINUOUS)
        
        OverTime = m1.addVars(K, name = 'Tau', vtype=GRB.CONTINUOUS)
        SlackTime = m1.addVars(K, name = "TauPrime", vtype = GRB.CONTINUOUS)
        
        TravelTime = [0]*K
        TimeAtLot = [0]*K
        VisitCountConstraints = m1.addConstrs(y[k,i] == x.sum(k,i,'*') for k in range(K) for i in range(A))
        
        for k in range(K):
            for i in range(A):
                for j in range(A):
                    TravelTime[k] += x[k,i,j] * Time[k,i,j]
        
        
                    
        for k in range(K):
            Cost += OverTimeCost * OverTime[k]
            for i in range(A):
                
                Cost -= Fine*ExpectedViolations*y[k,i]
                TimeAtLot[k] += y[k,i] * FineProcessingTime * ExpectedViolations
                
        for k in range(K):
            m1.addConstr(TravelTime[k] + TimeAtLot[k] == ShiftLength + OverTime[k] - SlackTime[k])
                
    
    
        m1.setObjective(Cost, GRB.MINIMIZE)

        m1.write('C:\PhD IE\Optimization under Uncertainty\parkingVRP.lp')

        m1.optimize()
        if m1.Status == GRB.OPTIMAL:
            obj = m1.getObjective()
            solution = m1.getAttr('X',x)
            print("The objective is :",obj.getValue())
            ExpectedValueObj = obj.getValue()
            for k in range(K):
                print("Order of routing for %s:" % k)
                next = np.zeros(A)
                Routing = [0]
                for i in range(A):
                    for j in range(A):
                        if solution[k,i,j] > 0:
                            # print(i,j)
                            next[i] = j
                current = 0
                while next[current] != 0:
                    
                    Routing.append(int(next[current]))
                    current = int(next[current])
                Routing.append(0)
                print(Routing)
        
        return ExpectedValueObj
    
    if Type == "MonteCarlo":
        m1 = gp.Model("ParkingVRP")
        m1.params.LogToConsole = 0
        print("Number of samples:", len(NViolations[0]))
        print("\n\n\nMonte Carlo")
        MonteCarloObj = 0

        x = m1.addVars(K, A, A, name="X", vtype=GRB.BINARY)

        u = m1.addVars(K,A, name="U", vtype=GRB.INTEGER,lb=1,ub=A)
        
        # Z = 0

        RoutingCost = 0

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    RoutingCost += c[k,i,j] * x[k,i,j] 
                RoutingCost += StartCost * x[k,0,i]
        InflowConstraints = m1.addConstrs(x.sum(k,0,'*') <= 1 for k in range(K) ) #for i in range(1,A))

        OutflowConstraints = m1.addConstrs(x.sum(k,'*',0) <= 1 for k in range(K)) #for j in range(A))

        # StartFlowConstraints = m1.addConstrs(x.sum(k,0,'*')  == 1 for k in range(K))

        FlowBalanceConstraints = m1.addConstrs(x.sum(k,'*',j) - x.sum(k,j,'*') == 0 for k in range(K) for j in range(1,A))

        # EndFlowConstraints = m1.addConstrs(x.sum(k,'*',0)  == 1 for k in range(K)) 

        StayProhibitionConstraint = m1.addConstrs(x[k,i,i]==0 for k in range(K) for i in range(1,A))

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    if i != j:
                        if i != 0:
                            if j != 0:
                                m1.addConstr(u[k,j] >= u[k,i] + 1 - (A) * (1 - x[k,i,j]))
                            
        Cost = RoutingCost
        # Z = NViolations[l]
        y = m1.addVars(K,A,name='Y',vtype=GRB.CONTINUOUS)
        
        OverTime = m1.addVars(K, name = 'Tau', vtype=GRB.CONTINUOUS)
        SlackTime = m1.addVars(K, name = "TauPrime", vtype = GRB.CONTINUOUS)
        
        TravelTime = [0]*K
        TimeAtLot = [0]*K
        VisitCountConstraints = m1.addConstrs(y[k,i] == x.sum(k,i,'*') for k in range(K) for i in range(A))
        
        for k in range(K):
            for i in range(A):
                for j in range(A):
                    TravelTime[k] += x[k,i,j] * Time[k,i,j]
        
        
                    
        for k in range(K):
            Cost += OverTimeCost * OverTime[k]
            for i in range(A):
                
                Cost -= Fine*np.mean(NViolations[i])*y[k,i]
                TimeAtLot[k] += y[k,i] * FineProcessingTime * np.mean(NViolations[i])
                
        for k in range(K):
            m1.addConstr(TravelTime[k] + TimeAtLot[k] == ShiftLength + OverTime[k] - SlackTime[k])
                
    
    
        m1.setObjective(Cost, GRB.MINIMIZE)

        m1.write('C:\PhD IE\Optimization under Uncertainty\parkingVRP.lp')

        m1.optimize()
        if m1.Status == GRB.OPTIMAL:
            obj = m1.getObjective()
            solution = m1.getAttr('X',x)
            print("The objective is :",obj.getValue())
            MonteCarloObj = obj.getValue()
            for k in range(K):
                print("Order of routing for %s:" % k)
                next = np.zeros(A)
                Routing = [0]
                for i in range(A):
                    for j in range(A):
                        if solution[k,i,j] > 0:
                            # print(i,j)
                            next[i] = j
                current = 0
                while next[current] != 0:
                    
                    Routing.append(int(next[current]))
                    current = int(next[current])
                Routing.append(0)
                print(Routing)
        
        return MonteCarloObj, solution
    
def MonteCarloTest(X):
    print("\n\n\n Monte Carlo Test")
    print("Number of samples:", len(NViolations))
    
    MonteCarloObj = [0]*len(NViolations)
    for leng in range(len(NViolations)):
        m1 = gp.Model("ParkingVRP")
        m1.params.LogToConsole = 0
        
        

        x = m1.addVars(K, A, A, name="X", vtype=GRB.BINARY)

        u = m1.addVars(K,A, name="U", vtype=GRB.INTEGER,lb=1,ub=A)
        
        # Z = 0

        RoutingCost = 0

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    RoutingCost += c[k,i,j] * x[k,i,j] 
                RoutingCost += StartCost * x[k,0,i]
        InflowConstraints = m1.addConstrs(x.sum(k,0,'*') <= 1 for k in range(K) ) #for i in range(1,A))

        OutflowConstraints = m1.addConstrs(x.sum(k,'*',0) <= 1 for k in range(K)) #for j in range(A))

        # StartFlowConstraints = m1.addConstrs(x.sum(k,0,'*')  == 1 for k in range(K))

        FlowBalanceConstraints = m1.addConstrs(x.sum(k,'*',j) - x.sum(k,j,'*') == 0 for k in range(K) for j in range(1,A))

        # EndFlowConstraints = m1.addConstrs(x.sum(k,'*',0)  == 1 for k in range(K)) 

        StayProhibitionConstraint = m1.addConstrs(x[k,i,i]==0 for k in range(K) for i in range(1,A))

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    m1.addConstr(x[k,i,j] == X[k,i,j])
                    if i != j:
                        if i != 0:
                            if j != 0:
                                m1.addConstr(u[k,j] >= u[k,i] + 1 - (A) * (1 - x[k,i,j]))
                            
        Cost = RoutingCost
        # Z = NViolations[l]
        y = m1.addVars(K,A,name='Y',vtype=GRB.CONTINUOUS)
        
        OverTime = m1.addVars(K, name = 'Tau', vtype=GRB.CONTINUOUS)
        SlackTime = m1.addVars(K, name = "TauPrime", vtype = GRB.CONTINUOUS)
        
        TravelTime = [0]*K
        TimeAtLot = [0]*K
        VisitCountConstraints = m1.addConstrs(y[k,i] == x.sum(k,i,'*') for k in range(K) for i in range(A))
        
        for k in range(K):
            for i in range(A):
                for j in range(A):
                    TravelTime[k] += x[k,i,j] * Time[k,i,j]
        
        
                    
        for k in range(K):
            Cost += OverTimeCost * OverTime[k]
            for i in range(A):
                
                Cost -= Fine*NViolations[leng][i]*y[k,i]
                TimeAtLot[k] += (y[k,i] * (FineProcessingTime * NViolations[leng][i]))
                
        for k in range(K):
            m1.addConstr(TravelTime[k] + TimeAtLot[k] -ShiftLength - OverTime[k] + SlackTime[k] == 0)
                


        m1.setObjective(Cost, GRB.MINIMIZE)

        m1.write('C:\PhD IE\Optimization under Uncertainty\parkingVRP.lp')

        m1.optimize()
        if m1.Status == GRB.OPTIMAL:
            obj = m1.getObjective()
            solution = m1.getAttr('X',x)
            # print("The objective is :",obj.getValue())
            MonteCarloObj[leng] = obj.getValue()
            for k in range(K):
                # print("Order of routing for %s:" % k)
                next = np.zeros(A)
                Routing = [0]
                for i in range(A):
                    for j in range(A):
                        if solution[k,i,j] > 0:
                            # print(i,j)
                            next[i] = j
                current = 0
                while next[current] != 0:
                    
                    Routing.append(int(next[current]))
                    current = int(next[current])
                Routing.append(0)
                # print(Routing)
        
    return MonteCarloObj

def MonteCarloTest2(X):
    print("\n\n\n Monte Carlo Test")
    print("Number of samples:", len(NViolations))
    
    MonteCarloObj = [0]*len(NViolations)
    for leng in range(len(NViolations)):
        m1 = gp.Model("ParkingVRP")
        m1.params.LogToConsole = 0
        
        

        x = m1.addVars(K, A, A, name="X", vtype=GRB.BINARY)

        u = m1.addVars(K,A, name="U", vtype=GRB.INTEGER,lb=1,ub=A)
        
        # Z = 0

        RoutingCost = 0

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    RoutingCost += c[k,i,j] * x[k,i,j] 
                RoutingCost += StartCost * x[k,0,i]
        InflowConstraints = m1.addConstrs(x.sum(k,0,'*') <= 1 for k in range(K) ) #for i in range(1,A))

        OutflowConstraints = m1.addConstrs(x.sum(k,'*',0) <= 1 for k in range(K)) #for j in range(A))

        # StartFlowConstraints = m1.addConstrs(x.sum(k,0,'*')  == 1 for k in range(K))

        FlowBalanceConstraints = m1.addConstrs(x.sum(k,'*',j) - x.sum(k,j,'*') == 0 for k in range(K) for j in range(1,A))

        # EndFlowConstraints = m1.addConstrs(x.sum(k,'*',0)  == 1 for k in range(K)) 

        StayProhibitionConstraint = m1.addConstrs(x[k,i,i]==0 for k in range(K) for i in range(1,A))

        for k in range(K):
            for i in range(A):
                for j in range(A):
                    m1.addConstr(x[k,i,j] == X[k,i,j])
                    if i != j:
                        if i != 0:
                            if j != 0:
                                m1.addConstr(u[k,j] >= u[k,i] + 1 - (A) * (1 - x[k,i,j]))
                            
        Cost = RoutingCost
        # Z = NViolations[l]
        y = m1.addVars(K,A,name='Y',vtype=GRB.CONTINUOUS)
        
        OverTime = m1.addVars(K, name = 'Tau', vtype=GRB.CONTINUOUS)
        SlackTime = m1.addVars(K, name = "TauPrime", vtype = GRB.CONTINUOUS)
        
        TravelTime = [0]*K
        TimeAtLot = [0]*K
        VisitCountConstraints = m1.addConstrs(y[k,i] == x.sum(k,i,'*') for k in range(K) for i in range(A))
        
        for k in range(K):
            for i in range(A):
                for j in range(A):
                    TravelTime[k] += x[k,i,j] * Time[k,i,j]
        
        
                    
        for k in range(K):
            Cost += OverTimeCost * OverTime[k]
            for i in range(A):
                
                Cost -= Fine*NViolations[leng][i]*y[k,i]
                TimeAtLot[k] += (y[k,i] * (FineProcessingTime * NViolations[leng][i]))
                
        for k in range(K):
            m1.addConstr(TravelTime[k] + TimeAtLot[k] -ShiftLength - OverTime[k] + SlackTime[k] == 0)
                


        m1.setObjective(Cost, GRB.MINIMIZE)

        m1.write('C:\PhD IE\Optimization under Uncertainty\parkingVRP.lp')

        m1.optimize()
        if m1.Status == GRB.OPTIMAL:
            obj = m1.getObjective()
            solution = m1.getAttr('X',x)
            # print("The objective is :",obj.getValue())
            MonteCarloObj[leng] = obj.getValue()
            for k in range(K):
                # print("Order of routing for %s:" % k)
                next = np.zeros(A)
                Routing = [0]
                for i in range(A):
                    for j in range(A):
                        if solution[k,i,j] > 0:
                            # print(i,j)
                            next[i] = j
                current = 0
                while next[current] != 0:
                    
                    Routing.append(int(next[current]))
                    current = int(next[current])
                Routing.append(0)
                # print(Routing)
        
    return MonteCarloObj

# WaitAndSeeObj = Problem("WaitAndSee")
ExpectedValueObj = Problem("ExpectedValue")
# Mean_WAS = np.mean(WaitAndSeeObj)
# SD_WAS = np.std(WaitAndSeeObj)
# print("Mean Wait and see objective:",Mean_WAS)
# print("95% confidence interval of wait and see objective:",Mean_WAS-1.96*SD_WAS/NIterations**0.5,Mean_WAS+1.96*SD_WAS/NIterations**0.5)

print("Expected value objective:",ExpectedValueObj)

MonteCarloObj=[0]*10
X=[0]*10
MonteCarloTestObj = np.zeros((10,50,50))
NPoissonSamples = [50*i for i in range(1,10)]
for iter in range(len(NPoissonSamples)):
    NViolations = np.random.poisson(ExpectedViolations,(A,NPoissonSamples[iter]))
    MonteCarloObj[iter],X[iter] = Problem("MonteCarlo")


for j in range(2):
    NViolations = np.random.poisson(ExpectedViolations,(50,A))
    for i in range(len(NPoissonSamples)):
        print("\n\n\n Monte Carlo Test for ", i,"th iteration and",j,"th sample")
        MonteCarloTestObj[i][j] =  MonteCarloTest(X[i])

print(MonteCarloTestObj)
for i in range(9):
    Results = pd.DataFrame(MonteCarloTestObj[i])
    string = "C:\\PhD IE\\Optimization under Uncertainty\\MonteCarloResults"+str(i)+".csv"
    Results.to_csv(string)
    
    print("Mean objective for ",NPoissonSamples[i],":",np.mean(MonteCarloTestObj[i]))
    print("95% confidence interval for ", NPoissonSamples[i],":",np.mean(MonteCarloTestObj[i]) - 1.96 * np.std(MonteCarloTestObj[i])/50,np.mean(MonteCarloTestObj[i]) + 1.96 * np.std(MonteCarloTestObj[i])/50)                


# NPoissonSamples = [50*i for i in range(1,10)]

# MonteCarloTestObj2 = [0]*len(NPoissonSamples)
# for iter in range(len(NPoissonSamples)):
#     MonteCarloTestObj2
#     NViolations = np.random.poisson(ExpectedViolations,(NPoissonSamples[iter],A))
#     MonteCarloTestObj2[iter] = MonteCarloTest2[X[9]]

#     print("Mean for sample size: ",NPoissonSamples[iter],np.mean(MonteCarloTestObj2[iter]))
#     print("Confidence interval:")
#     print("5%:"np.mean(MonteCarloTestObj2[iter]) - 1.96 * np.std(MonteCarloTestObj2)/50)
           


