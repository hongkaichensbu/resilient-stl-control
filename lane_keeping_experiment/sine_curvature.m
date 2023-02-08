% This file computes the y coordinates of the boundaries of a
% sinewave-shaped track
function [y_upperbounds, y_lowerbounds] = sine_curvature(yl, yu, H, track_lambda, delta_x)
% delta_x: constant. The x-distance between two points
% track_lambda: the wavelength of the track
% H: number of points. 

if nargin == 0
    yl = -0.5; yu = 0.5;
    H = 50; track_lambda = 50;
    delta_x = 2;
end

time = 0: delta_x: delta_x*(H-1);
% time = 0: 0.1:100;
k = 2*pi/track_lambda;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
centerline = sin(k*time);
y_upperbounds = centerline + yu;
y_lowerbounds = centerline + yl;

% figure; 
% plot(centerline); hold on
% plot(y_lowerbounds); 
% plot(y_upperbounds);
% ylabel('y coordinates (m)');
% xlabel('time points');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% options = sdpsettings('debug', 0,...
% 'solver', 'fmincon', ...
%     'verbose',0);
% for i = 1:H
%     sdpvar x e f
%     constraints = [e == time(i); f>=sin(k*time(i)); (e-x)^2 + (f-sin(k*x))^2 - yu^2 >=0];
%     sol=optimize(constraints, (e-x)^2 + (f-sin(k*x))^2,options);
%     y_upperbounds(i) = value(f);
% end
% for i = 1:H
%     sdpvar x e f
%     constraints = [e == time(i); f<=sin(k*time(i)); (e-x)^2 + (f-sin(k*x))^2 - yu^2 >=0];
%     sol=optimize(constraints, (e-x)^2 + (f-sin(k*x))^2,options);
%     y_lowerbounds(i) = value(f);
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% the path that a center of a circle rolling along a sinewave
% rho = 0.5;
% k = 2*pi/track_lambda;
% for i = 1:H
%     cx(i) = time(i) + rho*k*cos(k*time(i))/...
%         sqrt(1+k^2*cos(k*time(i))^2);
%     cy(i) = sin(k*time(i)) - rho/sqrt(1+k^2*cos(k*time(i))^2);
% end
% 
% scatter(cx/2,cy)

end

