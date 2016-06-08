function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    P = eye(2);
    R = eye(1);

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(2);
        param.R = 1*eye(1);
        predictx = x;
        predicty = y;
        dt = 0;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    dt = t - previous_t;
    
    state_x = [state(1); state(3)];
    state_y = [state(2); state(4)];
    
    % Predict 330ms into the future
    A = [1 dt; 0 1];
    B = [dt^2/2; dt];
    C = [1 0];
    
    % Update the Kalman gain
    K = param.P*C'*inv(C*P*C' + param.R);
    param.P = param.P - K*C*param.P;
    
    
    z = C*state_x;
    state_x = A*state_x + K*(z - C*A*state_x);
    
    z = C*state_y;
    state_y = A*state_y + K*(z - C*A*state_y);
  
   
    % State is a four dimensional element
    state = [state_x(1), state_y(1), state_x(2), state_y(2)];
    predictx = state_x(1);
    predicty = state_y(1);
end
