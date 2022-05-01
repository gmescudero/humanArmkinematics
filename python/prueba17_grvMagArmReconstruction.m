close all
clear all

%% Configure
fs = 25; T = 1/fs;
totalNumSamples = 30/T;

%% IMU setup

imu.COMPort  = 'COM3';
imu.baudrate = 921600;
imu.handle   = lpms();

% Connect to sensor
if ( ~imu.handle.connect(imu.COMPort, imu.baudrate) )
    return
end
disp('sensor connected')

% Set streaming mode
imu.handle.setStreamingMode();

%% Init values
d = [];
while((isempty(d)) || (0 == d.timestamp))
    d = imu.handle.getCurrentSensorData();
end
% Get sensor first data
gyr = d.angVel';
acc = d.acc'; 
mag = d.mag';

%% Calibration
fprintf('Starting Calibration ')
measures = zeros(9,100);
for i=1:500
    d = imu.handle.getCurrentSensorData();
    measures(:,i) = [d.acc,d.angVel,d.mag]';
    if (0 == rem(i,20)); fprintf('.'); end
    pause(0.01);
end
med = zeros(9,1); var = zeros(9,1);
for i = 1:9
    [med(i),var(i)] = medVar(measures(i,:));
end
 fprintf(' DONE! \n'); 

%% Kalman setup
S = @(x) [ % Skew-symetric matrix of x
    0, -x(3), x(2);
    x(3), 0, -x(1);
    -x(2), x(1), 0
];
SA = @(x) [S(x), zeros(3,3); zeros(3,3), S(x)];

X = [acc; mag];
P = eye(6)*10;
Z = [acc; mag];
U = gyr;

% kalman.A = [S(gyr), zeros(3,3); zeros(3,3), S(gyr)];
% kalman.B = zeros(1,6);
% kalman.C = eye(6);

G = @(x,u) SA(u);
kalman.g = @(x,u) (SA(u)*x)*T + eye(6)*x;
kalman.h = @(x) x;

kalman.G = G(X,U);
kalman.H = eye(6);

kalman.Q = SA(var(4:6)) + SA(norm(gyr)*ones(3,1));
kalman.R = diag([var(1:3);var(7:9)]);

% [X,P] = extended_kalman_filter(kalman,X,P,U,Z);

%% LOOP
% Quest reference vectors
r1 = acc;                    r1 = r1/norm(r1);
r2 = cross(acc,mag);         r2 = r2/norm(r2);
init = r2;

% Initialize logging data
XAll = zeros(6,totalNumSamples);
PAll = zeros(6,totalNumSamples);
ZAll = zeros(6,totalNumSamples);
UAll = zeros(3,totalNumSamples);
QAll = zeros(4,totalNumSamples);
Qimu = zeros(4,totalNumSamples);
% Loop
for i = 1:totalNumSamples
    % Get data
    d = [];
    while((isempty(d)) || (0 == d.timestamp))
        d = imu.handle.getCurrentSensorData();
    end
    % Get sensor first data
    gyr = d.angVel';
    acc = d.acc'; 
    mag = d.mag';
    % Kalman
    Z = [acc; mag];
    U = gyr;
    kalman.Q = SA(var(4:6)) + SA(norm(gyr)*ones(3,1));
    kalman.G = G(X,U);
    [X,P] = extended_kalman_filter(kalman,X,P,U,Z);
    % QuEst
    g = X(1:3); % g
    m = X(4:6); % mag
%     g = acc;
%     m = mag;
    b1 = g;             b1 = b1/norm(b1);
    b2 = cross(g,m);    b2 = b2/norm(b2);
    q_opt = quEst_2vectors(r1,r2,b1,b2);
    % Store Data
    XAll(:,i) = X;
    PAll(:,i) = diag(P);
    ZAll(:,i) = Z;
    UAll(:,i) = U;
    QAll(:,i) = q_opt;
    Qimu(:,i) = d.quat';
    % Plot
    v = rotate_vector_by_quaternion(init,q_opt);
    plot3([0;v(1)],[0;v(2)],[0;v(3)])
    grid on;
    title('vector rotated')
    xlim([-1.2,1.2]);xlabel('x')
    ylim([-1.2,1.2]);ylabel('y')
    zlim([-1.2,1.2]);zlabel('z')
    drawnow
    
    pause(T)
end

%% Disconnect
close
for i = 1:length(imu)
    fprintf("Disconnecting sensor with id %d ... ",imu(i).handle.imuId)
    if (imu(i).handle.disconnect())
        disp('Sensor disconnected')
    end
end

%% Results
medX = zeros(9,1); varX = zeros(9,1);
for i = 1:6
    [medX(i),varX(i)] = medVar(XAll(i,:));
end

figure
plot(1:totalNumSamples,PAll,'LineWidth',1.5)
title('Varianza'); legend('acc_x','acc_y','acc_z','mag_x','mag_y','mag_z')

figure
subplot(2,1,1);
plot(1:totalNumSamples,XAll(1:3,:),'LineWidth',1.5)
title('Estados acc'); legend('x','y','z')
subplot(2,1,2);
plot(1:totalNumSamples,XAll(4:6,:),'LineWidth',1.5)
title('Estados mag'); legend('x','y','z')

figure
subplot(3,1,1);
plot(1:totalNumSamples,ZAll(1:3,:),'LineWidth',1.5)
title('Medidas acc'); legend('x','y','z')
subplot(3,1,2);
plot(1:totalNumSamples,ZAll(4:6,:),'LineWidth',1.5)
title('Medidas mag'); legend('x','y','z')
subplot(3,1,3);
plot(1:totalNumSamples,UAll,'LineWidth',1.5)
title('Entradas gyr'); legend('x','y','z')

figure
subplot(2,1,1);
plot(1:totalNumSamples,QAll,'LineWidth',1.5)
title('Quaternion OPT'); legend('scalar','x','y','z')
subplot(2,1,2);
plot(1:totalNumSamples,Qimu,'LineWidth',1.5)
title('Quaternion Real'); legend('scalar','x','y','z')

%% FUNCTIONS
function [media,varianza] = medVar(X)
sumaX = 0;
sumX2 = 0;
n = length(X);

for i=1:n
    sumaX = sumaX + X(i);
    sumX2 = sumX2 + X(i)^2;
end

media = sumaX/n;
varianza = (sumX2 - (media^2)*n)/(n-1);
end