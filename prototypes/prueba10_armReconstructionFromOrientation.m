close
clear all

%% DESCRIPTION
% This script uses the quaternion data given from the LPMS-URS2 IMU sensors
% to reconstruct the motion of a human arm

%% USAGE
% 0 - Take a look at the Configuration section and set up according to your
% case
% 1 - Connect a pair of two imus in the configured COM ports
% 2 - Place the first IMU in the arm close to the elbow 
% 3 - Place the second IMU in the forearm close to the wrist (for testing
% you may just hold it in you hand and try not to move the wrist)
% 4 - Execute the script which will lead to the following set of steps:
%   4.1 - IMU setup: wait for it to finish 
%   4.2 - Calibration: for this phase stand with your arm in T-pose and
%   thumbs pointing forward
%   4.3 - Loop: while looping the image of the arm will show and the
%   position of the wrist will be printed throug console. 
%
% NOTE: For low performance PCs it is recomended to comment out the 
% plotting functionality as it is quite hungry for resources.



%% Configuration

% Arm config
shoulder2elbowLength = 34; % Length from shoulder to elbow
elbow2wristLength    = 32; % Length from elbow to wrist

% Time alive config
iterations = 20000;

% imu config
imu(1).baudrate = 921600;          % Baud rate to retrieve data
imu(1).COMport  = 'COM4';          % COM port where the imu(1) goes

imu(2).baudrate = 921600;          % Baud rate to retrieve data
imu(2).COMport  = 'COM3';          % COM port where the imu(2) goes

%% Initial data build

% Arm data
arm.shoulder.calibrationPos = [0,0,0];  % Position when calibrating
arm.shoulder.calibrationOri = [1,0,0,0];% Orientation when calibrating (T-pose)
arm.shoulder.pos = arm.shoulder.calibrationPos; % Shoulder joint position. Initial value equal to calibration
arm.shoulder.ori = [];  % Shoulder orientation. Unknown until calibration
arm.shoulder.sensorToJoint = [];    % Sensor to Joint conversion. Unknown until calibration
arm.shoulder.toNextJoint = [-shoulder2elbowLength,0,0]; % Vector from the shoulder joint to the elbow joint

arm.elbow.calibrationPos = arm.shoulder.calibrationPos + ...
    rotate_vector_by_quaternion(arm.shoulder.toNextJoint, arm.shoulder.calibrationOri); % Position when calibrating
arm.elbow.calibrationOri = [1,0,0,0];   % Orientation when calibrating (T-pose)
arm.elbow.pos = arm.shoulder.calibrationPos;    % Elbow joint position. Initial value equal to calibration
arm.elbow.ori = []; % Elbow orientation. Unknown until calibration
arm.elbow.sensorToJoint = [];   % Sensor to Joint conversion. Unknown until calibration
arm.elbow.toNextJoint = [-elbow2wristLength,0,0];   % Vector from the elbow joint to the wrist joint

arm.wrist.calibrationPos = arm.elbow.calibrationPos + ...
    rotate_vector_by_quaternion(arm.elbow.toNextJoint, arm.elbow.calibrationOri);   % Position when calibrating
arm.wrist.pos = arm.wrist.calibrationPos;    % Orientation when calibrating


positiveLimit = shoulder2elbowLength + elbow2wristLength + 3;
limits = [-positiveLimit,positiveLimit];

%% IMU initialization

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

fprintf("Stand in T-pose\n");
pause(2);
% Calibrate shoulder orientation
d1 = [];
while((isempty(d1)) || (0 == d1.timestamp))
    d1 = imu(1).handle.getCurrentSensorData();
    imu(1).calibrationOri = d1.quat;
end
arm.shoulder.sensorToJoint = quaternion_multiply(quaternion_conjugate(imu(1).calibrationOri),arm.shoulder.calibrationOri);
% Calibrate elbow orientation
d2 = [];
while((isempty(d2)) || (0 == d2.timestamp))
    d2 = imu(2).handle.getCurrentSensorData();
    imu(2).calibrationOri = d2.quat;
end
arm.elbow.sensorToJoint = quaternion_multiply(quaternion_conjugate(imu(2).calibrationOri),arm.elbow.calibrationOri);
fprintf("Calibration finished\n");

%% Data loop  

% Figure
figure('doublebuffer','on', ...
    'CurrentCharacter','a', ...
    'WindowStyle','modal');

while ((iterations>0) && (double(get(gcf,'CurrentCharacter'))~=27))
    %% Ploting
%     plot3(...
%         [arm.shoulder.pos(1);arm.elbow.pos(1);arm.wrist.pos(1)],...
%         [arm.shoulder.pos(2);arm.elbow.pos(2);arm.wrist.pos(2)],...
%         [arm.shoulder.pos(3);arm.elbow.pos(3);arm.wrist.pos(3)])
%    
%     grid on;
%     title(sprintf('ts = %fs', d1.timestamp))
%     xlim(limits);xlabel('x')
%     ylim(limits);ylabel('y')
%     zlim(limits);zlabel('z')
%     drawnow limitrate
    
    ax1 = subplot(2,3,[1,2,4,5]);
    ax2 = subplot(2,3,3);
    ax3 = subplot(2,3,6);
    
    plotObj = plot3(ax1,...
        [arm.shoulder.pos(1);arm.elbow.pos(1);arm.wrist.pos(1)],...
        [arm.shoulder.pos(2);arm.elbow.pos(2);arm.wrist.pos(2)],...
        [arm.shoulder.pos(3);arm.elbow.pos(3);arm.wrist.pos(3)]);
%     plotObj = plot3(ax1,elbowPos(:,1),elbowPos(:,2),elbowPos(:,3));
%     plotObj = quiver3(ax1,0,0,0,arm.elbowPos(1),arm.elbowPos(2),arm.elbowPos(3));
    copyobj(plotObj,ax2);
    copyobj(plotObj,ax3);

    view(ax1,[positiveLimit,positiveLimit,positiveLimit]); 
    title(ax1,sprintf('pos: ts = %fs', iterations));
    view(ax2,90,0); title(ax2,'view(90,0)');
    view(ax3,0,90); title(ax3,'view(0,90)');

    grid(ax1,'on')
    xlim(ax1,limits);xlabel(ax1,'x')
    ylim(ax1,limits);ylabel(ax1,'y')
    zlim(ax1,limits);zlabel(ax1,'z')
    grid(ax2,'on')
    xlim(ax2,limits);xlabel(ax2,'x')
    ylim(ax2,limits);ylabel(ax2,'y')
    zlim(ax2,limits);zlabel(ax2,'z')
    grid(ax3,'on')
    xlim(ax3,limits);xlabel(ax3,'x')
    ylim(ax3,limits);ylabel(ax3,'y')
    zlim(ax3,limits);zlabel(ax3,'z')
    drawnow limitrate
    
    %% Algorithm
    % Limit the rate
%     pause(0.1);
    iterations = iterations-0.1;
    % Extract IMU data and convert it to JOINT data
    d1 = imu(1).handle.getCurrentSensorData();
    arm.shoulder.ori = quaternion_multiply(d1.quat,arm.shoulder.sensorToJoint);
    d2 = imu(2).handle.getCurrentSensorData();
    arm.elbow.ori = quaternion_multiply(d2.quat,arm.elbow.sensorToJoint);
    % Compute each JOINT position
    arm.elbow.pos = arm.shoulder.pos + ...
        rotate_vector_by_quaternion(arm.shoulder.toNextJoint, quaternion_conjugate(arm.shoulder.ori));
    arm.wrist.pos = arm.elbow.pos + ...
        rotate_vector_by_quaternion(arm.elbow.toNextJoint, quaternion_conjugate(arm.elbow.ori));
    % Display last joint position
    disp(arm.wrist.pos)
end

%% Disconnect
close
for i = 1:length(imu)
    fprintf("Disconnecting sensor with id %d ... ",imu(i).handle.imuId)
    if (imu(i).handle.disconnect())
        disp('Sensor disconnected')
    end
end
