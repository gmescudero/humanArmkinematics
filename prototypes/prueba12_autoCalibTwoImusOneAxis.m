close all
clear all

%% DESCRIPTION
% This script uses the data given from the LPMS-URS2 IMU sensors to
% calibrate a rotation angle from the angular velocity difference between
% the two of them

%% USAGE
% 0 - Take a look at the Configuration section and set up according to your
% case
% 1 - Connect a pair of two imus in the configured COM ports
% 2 - Place both imus in some surface with the same orientation
% 3 - Execute the script which will lead to the following set of steps:
%   3.1 - IMU setup: wait for it to finish 
%   3.2 - Calibration: wait for it to finish with the IMUs in the position
%   described earlier
%   3.3 - Loop: while looping perform movements of one IMU over one 
%   rotation axis of your preference keeping the other untouched.

%% Configuration
iterations = 2000;
rate = 0.01;

config.gradientWindow   = 500;
config.gradientStepSize = 10;

initGuess = [rand();rand();rand()];
initGuess = initGuess/norm(initGuess);
aA = initGuess; 

% imu data
imu(1).COMport  = 'COM4';          % COM port where the IMU goes
imu(1).baudrate = 921600;          % Baud rate to retrieve data

imu(2).COMport  = 'COM3';          % COM port where the IMU goes
imu(2).baudrate = 921600;          % Baud rate to retrieve data

%% Imu initialization

% Connect to sensor
for i = 1:length(imu)
    imu(i).handle   = lpms();   % IMU handler
    imu(i).calibrationOri = []; % Calibration procedure orientation
    if (~imu(i).handle.connect(imu(i).COMport, imu(i).baudrate))
        return
    end
    fprintf("sensor with id %d connected\n",imu(i).handle.imuId)
    
    % Set streaming mode
    imu(i).handle.setStreamingMode();
end

%% Calibration
fprintf("Calibration \n");
% Calibrate shoulder orientation
d1 = [];
while((isempty(d1)) || (0 == d1.timestamp))
    d1 = imu(1).handle.getCurrentSensorData();
    imu(1).calibrationOri = d1.quat;
end

% Calibrate elbow orientation
d2 = [];
while((isempty(d2)) || (0 == d2.timestamp))
    d2 = imu(2).handle.getCurrentSensorData();
    imu(2).calibrationOri = d2.quat;
end
fprintf("Calibration finished\n");

%% Bucle
E = zeros(2,iterations);
A = zeros(3,iterations);
G = zeros(3,iterations);
J = zeros(1,iterations);

fprintf("Loop for %f seconds\n",iterations*rate);
for i = 1:iterations
    d1 = imu(1).handle.getCurrentSensorData();
    d2 = imu(2).handle.getCurrentSensorData();
% Conversion from 2 to 1
    calibQ1 = quaternion_multiply(d1.quat,quaternion_conjugate(imu(1).calibrationOri));
    calibQ2 = quaternion_multiply(d2.quat,quaternion_conjugate(imu(2).calibrationOri));
    q21 = quaternion_multiply(quaternion_conjugate(calibQ1),calibQ2);
% Calcular velocidad angular relativa
    omegaR  = -d1.gyr' + rotate_vector_by_quaternion(d2.gyr,q21)';
    [aA,se,j] = calibrateOneRotationAxis(config,aA,omegaR);
% Recogida de datos
    E(1,i) = se;
    sz     = min(i,100);
    E(2,i) = (1/sz)*sum(E(1,i-sz+1:i),2);
    A(:,i) = aA;
    G(:,i) = omegaR;
    J(1,i) = j;
% rate
    pause(rate)
end

set(gcf,'WindowStyle','normal');
close
if (imu(1).handle.disconnect())
    disp('sensor disconnected')
end

%% Resultados
fprintf("aA: [%f, %f, %f]\n",aA(1),aA(2),aA(3));
media = mean(G,2);
desv  = sqrt(var(G,0,2));
fprintf("omegaR med: [%f, %f, %f]\n",media(1),media(2),media(3));
fprintf("omegaR dev: [%f, %f, %f]\n",desv(1), desv(2), desv(3));

figure
plot(J)
title('Cost function')
figure
plot(1:iterations,E(1,:),1:iterations,E(2,:))
legend('raw','filtered')
title('Error se')
figure
plot(1:iterations,A(1,:),1:iterations,A(2,:),1:iterations,A(3,:))
legend('x','y','z')
title('Vector A')
figure
plot(1:iterations,G(1,:),1:iterations,G(2,:),1:iterations,G(3,:))
legend('x','y','z')
title('Gyro')