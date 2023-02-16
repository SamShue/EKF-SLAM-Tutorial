clc;
clear all;
close all;

% SLAM parameters
% State Vector : x
x = [0,0,0];
% Covariance Matrix : P
P = eye(length(h.x)).*0.1;
P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
% Measurement Noise : Q


function [x, P] = predict(x,P,u)
    % Get noise covariance matrix for control signal
    W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
    Q = zeros(size(P));
    C = 0.2;    % Process Noise Constant
    W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
    Q(1:3,1:3) = W*C*W';
    
    % 
    [x,F] = f(x,u);
    P = F*P*F' + h.Q;
    
    % Safety first! Ensure orientation doesn't pass 360:
    x(3) = wrapTo360(x(3));
end

function [x_new,F] = f(x,u)
    x_new = x;
    x_new(1:3) = [x(1) + u(1)*cosd(x(3)+u(2)); ...
                  x(2) + u(1)*sind(x(3)+u(2)); ...
                  x(3) + u(2)];
    % Jacobian F
    F = eye(length(x));
    F(1,3) = -1*u(1)*sind(x(3));
    F(2,3) = u(1)*cosd(x(3));
end

function measure(laserData, u, landmark_list)
    % Search for landmarks
    [observed_LL] = landmark_list.getLandmark(laserData,h.x);
    h.observed=observed_LL; 
    % Apply measurement update in EKF if landmarks are observed
    if(~isempty(observed_LL))
        numOfObservedLandmarks = size(observed_LL,1);
        for ii = 1:numOfObservedLandmarks  % "8: For all observed features..."
            R = zeros(2,2); R(1,1) = observed_LL(ii,1)*h.Rc(1); R(2,2) = observed_LL(ii,2)*h.Rc(2);
            %check if state has no landmarks
            if(length(h.x)<4)
                h.append(u,R,landmark_list.landmarkObj.landmark(find([landmark_list.landmarkObj.landmark.index])).loc,1);
            else
                %estimate correspondence
                
                % Measurement vector (Range and relative orientation)
                z = observed_LL(ii,:);
                numOfLandmarks=(length(h.x)-3)/2;
                if(z(3)>numOfLandmarks)
                    %append new landmark
                    h.append(u,R,landmark_list.landmarkObj.landmark(find([landmark_list.landmarkObj.landmark.index]==z(3))).loc,z(3));
           
                else
                    idx=ii;
                    mu = x(1:2)' + z(1)*[cosd(z(2) + x(3));sind(z(2) + x(3))];
                    mu_k = [x(4+(idx-1)*2); x(4+(idx-1)*2+1)];
                    delta_k = mu_k - x(1:2)';
                    q_k = delta_k'*delta_k;
                    
                    %Line 13
                    z_k=[sqrt(q_k); wrapTo360(atan2d(delta_k(2),delta_k(1))-(h.x(3)))];
                    
                    %Line 14
                    numOfLandmarks=(length(h.x)-3)/2;
                    F_k = zeros(5,numOfLandmarks*2+3);F_k(1:3,1:3) = eye(3);F_k(4:5,(4+(idx-1)*2):(5+(idx-1)*2)) = eye(2);
                    
                    %Line 15
                    H_k = (1/q_k)*[-sqrt(q_k)*delta_k(1), -sqrt(q_k)*delta_k(2), 0, sqrt(q_k)*delta_k(1), sqrt(q_k)*delta_k(2); ...
                        delta_k(2), -delta_k(1), -q_k, -delta_k(2), delta_k(1)]*F_k;
                    
                    %Line 16
                    phi_k = H_k*h.P*H_k' + R;
                    
                    K = P*H_k'*phi_k^-1;
                    h.x = h.x + (K*(z(1:2)' - z_k))';
                    h.P = (eye(size(h.P)) - K*H_k)*h.P;
                end
                
            end
        end
    end
end
