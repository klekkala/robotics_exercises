% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

    % Number of poses to calculate
    N = size(ranges, 2);
    % Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
    myPose = zeros(3, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
% myResolution = param.resol;
% % the origin of the map in pixels
% myOrigin = param.origin; 
% 
% for j = 1:N % for each time,
% 
%       
%     % Find grid cells hit by the rays (in the grid map coordinate frame)
%    
% 
%     % Calculate the correlation scores of the particles
%   
% 
%     % Update the particle weights
%     
% 
%     % Visualize the pose on the map as needed
%    
% 
% end

end

