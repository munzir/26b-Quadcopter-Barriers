function [xdd, ydd, zdd] = linearAccel(RotZYX, g, m, Ftotal)
    gz = [0 0 g];
    Fz = [0 0 -Ftotal];
    lin_accel = gz' + (1/m)*RotZYX*Fz';
    xdd = lin_accel(1) ;
    ydd = lin_accel(2);
    zdd = lin_accel(3);
end