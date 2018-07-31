function [Vr1, Vr2] = getRotVertex(V1, V2, phi, theta, psi)
    V_center1 = mean(V1,1);                   %Centre of line
    V_center2 = mean(V2,1);                   %Centre of line
    
    V_shift1  = ones(size(V1,1),1)*V_center1;
    V_shift2  = ones(size(V2,1),1)*V_center2;
    
    Vc1 = V1 - V_shift1;                       %Centering coordinates
    Vc2 = V2 - V_shift2;                       %Centering coordinates

    R = rotBodytoWorld(phi, theta, psi);
    
    Vrc1 = (R*Vc1')';         %Rotating centred coordinates
    Vrc2 = (R*Vc2')';         %Rotating centred coordinates
    
    Vr1 = Vrc1 + V_shift1;     %Shifting back to original location
    Vr2 = Vrc2 + V_shift2;     %Shifting back to original location
    
end