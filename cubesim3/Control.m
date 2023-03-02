function current = Control(BfieldNav,pqrNav)

k = 67200;
%%%Bfield is in teslas - 40000 nT = 4e-5 ~= 1e-5
%%%pqr is in rad/s -- 0.1 rad/s = 1e-1
%%%pqr*Bfield = 1e-6
%%%pqr*Bfield / (n*A) = 6e-7
%%% muB = n*I*A
magtorquer_params

current = k*cross(pqrNav,BfieldNav)/(n*A);
%%%current to be in amps ~= 40mA = 4e-2
