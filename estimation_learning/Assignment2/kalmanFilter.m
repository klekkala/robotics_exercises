function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    P = eye(2);
    R = eye(1);

    % Check if the first time running this function
    if previous_t<0
        state = [x; y; 0; 0];
        param.sigma_m = diag([x, y, 0, 0]);
        param.sigma_o = diag([0, 0]);
        param.P = param.sigma_m;
        param.R = param.sigma_o;
        predictx = x;
        predicty = y;
        dt = 0;
        return;
    end


%% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    dt = t - previous_t;
    
    meas = [state(1); state(2)];

    % Predict 330ms into the future
    A = [1, dt; 0 1];
    C = [1, 0, 0, 0;0, 1, 0, 0];
    
    state = mvnrnd(A*state, A*param.P*A') + mvnrnd([0;0], param.sigma_m);
    meas = mvnrnd(C*state, C*param.P*C') + mvnrnd([0; 0], param.sigma_o);

    % Define Kalman gain and update the Covariance
    P = param.P;
    %R = param.R;
    %K = P*C'*inv(R + C*P*C');
    param.P = A*P*A' + param.sigma_m;
    
  
   
    % State is a four dimensional element
    predictx = state_x(1);
    predicty = state_y(2);
end
