from math import pi
import numpy as np
'Input given value'
rho=1.225
U=0
r=0.12
RPM=5000
T = 1.3*9.81/4

'Disk Theory'
u = np.sqrt(T/(pi*r**2*2))
print('Incremental flow speed', u)

'Propeller'
n = RPM / 60
om = n*2*pi
D = 2*r

J_UI = U/(n*D)
CT_UI = 0.0944-0.0954*J_UI-0.0897*J_UI**2
CP_UI = 0.0339+0.0251*J_UI-0.1073*J_UI**2
CQ_UI = CP_UI/(2*pi)

Tp = CT_UI*rho*n**2*D**4
Pp = CP_UI*rho*n**3*D**5
Qp = CQ_UI*rho*n**2*D**5
etap = (Tp*U)/(Qp*om)
CT = Tp/(0.5*rho*(om*r)**2*pi*r**2)
CQ = Qp/(0.5*rho*(om*r)**2*pi*r**3)
CP = CQ
J = U/(om*r)
FOM = CT**(3/2)/(2*CQ)
print(Pp,Qp,CT,CQ,CP,FOM)


P = (np.sqrt((T)**3/(2*rho*(pi*r**2))))/FOM
Q = P/om
CT = (T)/(0.5*rho*(om*r)**2*pi*r**2)
Cp = P/(0.5*rho*om**3*pi*r**2*r**3)
CQ = Q/(0.5*rho*(om*r)**2*pi*r**3)
FOM = CT**(3/2)/(2*CQ)
print('propeller power is %.3f'%P,'[N*m/s]')
print('propeller torque is %.3f'%Q,'[N*m]')
print('CT value is %.3f'%CT)
print('Cp value is %.3f'%Cp)
print('CQ value is %.3f'%CQ)
print('Figure of merit is %.3f'%FOM)

'Motor'
I0 = 1.0 # Amp
R = 0.07 # Ohm
Kv = 920*2*pi/60 #rad/sec/Volt
V = 9 # Applied effective voltage

I = V/R - om/(R*Kv)
Qm = (V/(R*Kv)-I0/Kv) + (-1/(R*Kv**2))*om
etam = Qm*om / (V*I)
print(etam)
etatotal = etap*etam


'Matching'
am = V/(R*Kv)-I0/Kv
bm = -1/(R*Kv**2)
ap = rho * 4/pi**3 * r**3 * (-0.1073*U**2*pi**2)
bp = rho * 4/pi**3 * r**3 * (0.0251*U*r*pi)
cp = rho * 4/pi**3 * r**3 * (0.0339*r**2)
om_res = (-(bp-bm) + np.sqrt((bp-bm)**2 - 4*cp*(ap-am)))/(2*cp)
RPM_res = om_res * 60/(2*pi)
print('Matched rotation rate is %.3f [rad/s]'%om_res)
print('Matched RPM is %.3f'%RPM_res)