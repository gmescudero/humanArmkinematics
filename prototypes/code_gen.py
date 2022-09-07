from numpy import sin,cos
from sympy import Symbol

omega_x = Symbol('omega_x')
rho_1 = Symbol('rho_1')
theta_1 = Symbol('theta_1')
theta_2 = Symbol('theta_2')
rho_2 = Symbol('rho_2')
omega_y = Symbol('omega_y')
omega_z = Symbol('omega_z')
e = omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1))