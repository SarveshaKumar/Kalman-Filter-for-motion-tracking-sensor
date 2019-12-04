% Configuration Data:
% Program: Kalman_1D_V2
% Author: Sarvesha Kumar Kombaiah Seetha
% Version: 2.0
% Date: 4/21/18
% Description: Solves Kalman Filter for
% X(t+1) = a X(t) + W(t) in 1D.
% Set up
clear;
clc;
clf;
% Coefficient Values
F = 1;     
% Dynamic State Transform
B = 1;
u = 0.2;      % u(t)
Q = 0.5^2;    % State Covariance Error
H = 1/100;  % State-Measurement Transform
R = 0.01^2;   % Measurement Covariance Error
N = 50;     % Number of Time Steps
% Seed values
% Initialize Vectors
X = zeros(1,N);     % True State
X(1) = 0;
W = zeros(1,N+1);      % True State Random Noise
W = sqrt(Q)*randn(1,N); % Generate State Noise w. 
Var = QXe = zeros(1,N);   % Initial Estimated State
Xe(1) = 0;        % Seed value for Xe
Xce = zeros(1,N);   % Corrected Estimated State
Xce(1) = 0;        % Seed value for Xe
Pe = zeros(1,N);    % Covariance Error
Pe(1) = Q;        % Seed value for Xe
Z = zeros(1,N);     % Measurement
V = zeros(1,N);      % Measurement Random Noise
V = sqrt(R)*randn(1,N); % Generate Measurement Noise w. 
Var = RK = zeros(1,N);     % Kalman Gain
t = zeros(1,N);     % time
t(1) = 0;
% Calculations: Solve Kalman Filter
for i = 1:1:N-1
    t(i+1) = t(i) + 1;
    % True State:
    X(i+1) = F*X(i) + u + W(i);
    % Predict
    Xe(i+1) = F*Xce(i);
    Pe(i+1) = Pe(i)+Q;
    % Update
    Z(i+1) = H*X(i+1) + V(i+1); % Generate measurement
    y = Z(i+1) - H*Xe(i+1);     % Residual
    S = H^2 * Pe(i+1) + R;       % Residual 
    CovK(i+1) = H*Pe(i+1)/S;         % Kalman Gain
    Xce(i+1) = Xe(i+1) + K(i+1)*y; % Updated State Estimate
    Pe(i+1) = (1-H*K(i+1))*Pe(i+1); % Updated Cov. Estimate
end
X(N)
Xe(N)
K(N)
Pe(N)
% Calculations: Solve Kalman Filter
subplot(2,1,1)
plot (t,X, t, Xe)
legend('X(t)', 'Uncorrected Xe(t)','Location', 'northwest')
axis([0,N,-15,15])subplot(2,1,2)plot (t,K, t, Pe)
legend('K(t)', 'Pe(t)', 'Location', 'northwest')
axis([0,N,0,50])