clc; clear; 
close all;
x1 = linspace(-2,2,100);
x2 = linspace(-2,2,100);

% h = 1 - 0.4254*x1 - 0.3248*x2 -0.7549*x2.^2 - 0.8616*x1.^2 -0.2846*x1'*x2;
mu = [5 1; 5 1];
% for i=1:length(x1)
for i=1:length(x1)
    for j=1:length(x2)
        value = 1 - [x1(i) x2(j)]*mu*[x1(i) x2(j)]';
        h(i,j) = value;
        if value >= 0
            h_(i,j) = value;
        else
            h_(i,j) = 0;
        end
    end
end

figure
surf(x1,x2,h);
colormap summer
xlabel('x1'); ylabel('x2');
grid on;
limit = 1;
axis( [ min(x1)-limit max(x1)+limit ...
        min(x2)-limit max(x2)+limit ...
        min(min(h_))-limit max(max(h_))+limit 
     ] );

 % x = linspace(-0.5,0.5,100);
% a1 = 1;
% a2 = 2;
% a3 = 3;
%
% f1 = 1 - a1*x.^2;
% f2 = 1 - a2*x.^2;
% f3 = 1 - a3*x.^2;
%
%
% plot(x,f1, 'r', x, f2, 'b', x, f3, 'k');
% grid on;
% legend('a1', 'a2', 'a3');
