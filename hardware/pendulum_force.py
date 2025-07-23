import numpy as np

m = 0.200 #kg
l = 0.250 #m
g = 9.81 #m/s^2
I_pend = m * l**2 # kg*m^2

a = 8/1000 #m  [distance from pendulum to front bearing]
b = 52/1000 #m  [distance from front bearing to rear bearing]

w = 25/1000 #m  [assumed force is evenly distributed across the width of bearing]
E = 68258e+6 #Pa [mod of elasticity]
Y = 179.26e+6 #Pa [Yield Strength]
N = 8

#Pendulum Forces
max_energy = m * 2*l * g
max_rot_velo = (2*max_energy / I_pend)**(1/2)#E = I * w^2 /2
max_centrip_acc = max_rot_velo**2 / l

max_force = m * (max_centrip_acc + g) #N

print(f"max force = {max_force}N")

#Bearing Forces
A = np.array([[1, 1],
              [a, a+b]])

B = np.array([-max_force, 0])
[f_front, f_rear] = np.linalg.solve(A,B)

print('')
print(f"front bearing force = {f_front}N")
print(f"rear bearing force = {f_rear}N")



#Plate Bending Forces
#Plate Bending Forces

t = (6 * f_rear * b / (w * (Y/N)))**(1/2)

I_beam = w * t**3 / 12 # m^4
deflection = f_rear * b**3 / (3*E*I_beam)
stress = t * f_rear * b / (2*I_beam)

tip_deflection = l*deflection/b 

print('')
print(f"plate thickness {t*1e+3}mm ")
print(f"pendulum tip deflection {tip_deflection*1e+3}mm ")
print(f"maximum stress {stress*1e-6}MPa ")

