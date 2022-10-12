clc;
clear all;
close all;

% Robot parameters
robotPose = [2,3,deg2rad(45)];

% Populate wheel velocities vector
vl_mps = 0.7;
vr_radps = deg2rad(20);
% Linear and Angular Velocity Measurements
u = [vl_mps, vr_radps; vl_mps, vr_radps];
% time between iterations in seconds
dt_s = 0.5;

hold on;
drawRobot(robotPose(1), robotPose(2), rad2deg(robotPose(3)), 0.1);
xpath = [robotPose(1)]; ypath =[robotPose(2)];

for ii = 1:length(u(1,:))
    robotPose(1) = robotPose(1) + u(ii,1)*cos(robotPose(3))*dt_s;
    robotPose(2) = robotPose(2) + u(ii,1)*sin(robotPose(3))*dt_s;
    robotPose(3) = robotPose(3) + vr_radps*dt_s;
    
    drawRobot(robotPose(1), robotPose(2), rad2deg(robotPose(3)), 0.1);
    xpath = [xpath; robotPose(1)]; ypath =[ypath; robotPose(2)];
end

plot(xpath, ypath);
xlim([1.5 3.]); ylim([2.5 4.0]);
xlabel('meters'); ylabel('meters');