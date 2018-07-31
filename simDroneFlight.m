% clear; clc; 
% close all;

pause on;
% close all;
%Example coordinates
L  = 0.5;   % rotor-to-rotor distance
l  = L/(2*sqrt(2));
load stateHistory.mat
load refTrajectory.mat

%%
x_ref = refTrajectory(:,1)';
y_ref = refTrajectory(:,2)';
z_ref = refTrajectory(:,3)';

% legend('Reference');
%%
% subplot(2,2,1);
% figure;
% pause(5);
for i=1:10:length(stateHistory)
    x = stateHistory(i,1);
    y = stateHistory(i,2);
    z = stateHistory(i,3);
    
    phi   = stateHistory(i,4);
    theta = stateHistory(i,5);
    psi   = stateHistory(i,6);
    
    x1 = [x-l  x+l];
    y1 = [y-l  y+l];
    
    x2 = [x-l  x+l];
    y2 = [y+l  y-l];
    
    z1 = [z z];
    
    % Vertices matrix
    V1 = [x1(:) y1(:) z1(:)];
    V2 = [x2(:) y2(:) z1(:)];
    [Vr1, Vr2] = getRotVertex(V1, V2, phi, theta, psi);
    
    plot3(x_ref, y_ref, z_ref, 'b--');
    grid on;
    hold on;
    
    plot3(Vr1(:,1), Vr1(:,2), Vr1(:,3), 'r.-', 'MarkerSize', 10);   %Rotated around centre of line
    plot3(Vr2(:,1), Vr2(:,2), Vr2(:,3), 'r.-', 'MarkerSize', 10);   %Rotated around centre of line
    
    title('Flight Path','Interpreter','Latex');
    xlabel('X [meters]','Interpreter','Latex');
    ylabel('Y [meters]','Interpreter','Latex');
    zlabel('Z [meters]','Interpreter','Latex');
    limit = 4;
    axis([  min(x_ref)-limit/4    max(x_ref)+limit/4 ...
        min(y_ref)-limit/4    max(y_ref)+limit/4 ...
        min(z_ref)-limit    max(z_ref)+limit   ]);
    
    view(30,30)
    
    if mod(i,10)==0 || mod (i,11)==0 || mod(i,12)==0 || mod(i,13)==0
        pause(0.06);
    else
        pause(0.03);
    end
%     pause(0.035);
    hold off;
end
disp('Drone flight completed');