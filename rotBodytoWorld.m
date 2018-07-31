function RotZYX = rotBodytoWorld(phi, theta, psi)
Rx_phi  = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry_theta= [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz_psi  = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

RotZYX = Rz_psi*Ry_theta*Rx_phi;
end