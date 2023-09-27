import sys
import pandas as pd
import time, numpy as np
import pyomo.environ as pyo
from pyomo.environ import *
import math 

#Parameters

  #Coordinates
X_Pos = np.array([50, 91.96597675, 4.429534833, 70.62278085, 36.3491225, 20.25312943, 65.01756495, 80.47269263, 44.25742111, 55.37454121, 90.76322566, 90.20079397, 30.9490321, 89.52208309, 81.66672711, 17, 87.27887171, 83.6627887, 96.89989373, 59.63776655, 71.83902707, 92.31240214, 73.08969445, 21.40806172, 11.83393172, 6.887087363, 43.32531433, 86.01225057, 26.93871206, 29.06518566, 7.4])
Y_Pos = np.array([50, 2.429480545, 86.08245321, 29.59853816, 35.03474281, 93.31407738, 94.51487113, 75.14517172, 8.050213559, 46.94397809, 52.4313392, 87.32686729, 69.92298919, 72.81256499, 32.13648462, 83.28247292, 57.62395476, 8.733175191, 45.53407853, 16.50682552, 45.48331256, 29.08620109, 63.56253827, 9.609263291, 34.78660679, 42.58023397, 52.04592848, 86.82712686, 91.63189677, 55.63919829, 24])

n = len(X_Pos)

range_i = range(1,n)
range_j = range(0,n)

dist = np.zeros((len(X_Pos),len(Y_Pos))) #matrix of distances

  #Compute distances
for i in range(len(X_Pos)):
    for j in range(len(Y_Pos)):
        dist[i][j] = math.sqrt(math.pow(X_Pos[j]- X_Pos[i],2) + math.pow(Y_Pos[j]- Y_Pos[i],2))

C = 90 # Capacity

d = np.array([0,17,4,5,10,2,13,19,17,5,12,7,14,7,9,10,17,20,18,14,8,20,12,8,19,17,8,13,17,6,16]) # Demands

Total_Demand = sum(d) # Total Demand

#Create Model
model = pyo.ConcreteModel()

#Define variables
model.f = pyo.Var(range(0,n), # index i
                  range(0,n), # index j
                  bounds = (0,None))

model.x = pyo.Var(range(0,n), # index i
                  range(0,n), # index j
                  within = Binary)

f = model.f
x = model.x

#Constraints 
model.C1 = pyo.ConstraintList() 
for i in range_i:
    model.C1.add(expr = sum(x[i,j] for j in range_j)  == 1)
    
model.C2 = pyo.ConstraintList() 
for i in range_i:
    model.C2.add(expr = sum(x[j,i] for j in range_j)  == 1)

model.C3 = pyo.ConstraintList() 
for i in range_i:
    model.C3.add(expr = sum(f[j,i] - f[i,j] for j in range_j)  == d[i])

model.C4 = pyo.ConstraintList() 
for i in range(0,n):
    for j in range_j:
       model.C4.add(expr = f[i,j] - x[i,j]*C <= 0)

# Define Objective Function
model.obj = pyo.Objective(expr = sum(dist[i,j]*x[i,j] for i in range(0,n) for j in range_j), 
                          sense = minimize)


begin = time.time()
opt = SolverFactory('cplex')
results = opt.solve(model)

deltaT = time.time() - begin # Compute Exection Duration

model.pprint()

sys.stdout = open("Vehicle_Routing_Problem_VRP_Problem_CLSP_Problem_Results.txt", "w") #Print Results on a .txt file

print('Time =', np.round(deltaT,2))

if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
    print('Total Cost (Obj value) =', round(pyo.value(model.obj),2))
    print('Solver Status is =', results.solver.status)
    print('Termination Condition is =', results.solver.termination_condition)
    print(" " )
    print("Value of Variables x[i,j]" )
    for i in range_i:
        for j in range_j:
            if  pyo.value(x[i,j]) != 0:
               print('x[' ,i+1, '][ ', j+1,']: ', round(pyo.value(x[i,j]),2))
    print(" " )
    print("Value of Variables f[i,j]" )
    for i in range_i:
        for j in range_j:
            if  pyo.value(f[i,j]) != 0:
               print('f[' ,i+1, '][ ', j+1,']: ', round(pyo.value(f[i,j]),2))
    print(" " )
    for i in range(0,n):
        print('In node ',i+1,': ')
        In_Flow = 0
        Out_Flow = 0
        for j in range_j:
            In_Flow += pyo.value(f[j, i])
            Out_Flow += pyo.value(f[i, j])
        print('---> In_Flow value is: ',In_Flow)
        print('---> Out_Flow value is: ',Out_Flow)
        print('---> In_Flow - Out_Flow is: ',In_Flow - Out_Flow)
        print('---> Demand value is: ',d[i])
        print(" " )
            
elif (results.solver.termination_condition == TerminationCondition.infeasible):
   print('Model is unfeasible')
  #print('Solver Status is =', results.solver.status)
   print('Termination Condition is =', results.solver.termination_condition)
else:
    # Something else is wrong
    print ('Solver Status: ',  result.solver.status)
    print('Termination Condition is =', results.solver.termination_condition)
    
sys.stdout.close()