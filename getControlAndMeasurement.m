function [u, z, truePose] = getControlAndMeasurement(estimatedPose, v_mps, lastGoal, currentGoal)
    % Robot parameters
    persistent truePose;
    tireDiameter_m = 0.25;
    trackWidth_m = 0.5;
    model = 'icr'; % or 'linear'
    %v_mps = 0.5;

    %initialize truePose if first call
    if(isempty(truePose))
        truePose = estimatedPose;
    end
    
    % time between iterations in seconds
    dt_s = 0.1;
    lookaheaddist_m = 0.25;
%     persistent lastGoal;
% 
%     %initialize lastGoal
%     if(isempty(lastGoal))
%         lastGoal = estimatedPose:
%     else if (lastGoal == )
    
    %lastGoal = truePose(1:2);
    currentGoal = goalPoints(ii, :);

    [w_radps, gp] = purePursuit(estimatedPose, lastGoal, currentGoal, v_mps);

    % Update robot pose using kinematic model
    truePose = differentialDriveKinematics(robotPose, v_mps, w_radps, dt_s, model);
