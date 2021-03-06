import numpy as np
from control.matlab import *
from control import place

m = 0.067    # mass of pendulum in kg
M = 1.463    # mass of carriage in kg
l = 0.305    # COM of pendulum in m
g = 9.81     # gravity
b = 0.075    # coeff of friction
I = m*(l**2) # moment of inertia of pendulum

A = np.array([[0,1, 0, 0],
              [0, -b*(I+m*l**2)/((M+m)*I+M*m*l**2), (m**2)*(l**2)*g/(I*(M+m)+M*m*l**2), 0],
              [0,0,0,1],
              [0,-b*m*l/((M+m)*I+M*m*l**2),(m+M)*g*m*l/((M+m)*I+M*m*l**2),0]], dtype=float)

B = np.array([[0],[(I+m*l**2)/((M+m)*I+M*m*l**2)],
              [0],[m*l/((M+m)*I+M*m*l**2)]], dtype=float)

Q = np.array([[1, 0, 0  ,0   ],
              [0, 1, 0  ,0   ],
              [0, 0, 1000,0   ],
              [0, 0, 0  ,10000]], dtype=int)

# print(A,B)

R = 0.0030 

K = lqr(A,B,Q,R)[0]
k = np.array(K)
for i in k:
  x = np.round([*i],3)
  print(*x, sep=", ")
