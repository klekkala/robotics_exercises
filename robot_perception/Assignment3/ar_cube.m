function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course


R_prime = [H(:,1), H(:,2), cross(H(:,1), H(:,2))];
[U, S, V] = svd(R_prime);

S_prime = [1, 0, 0; 0, 1, 0; 0, 0, det(U*V')];
R = U*S_prime*V';

t = H(:,3)/norm(H(:,1));

proj_points = [];
for i=1:size(render_points, 1)
X_c = K*(R*render_points(i, :)' + t);
X_im = X_c/X_c(3);
X_im(3) = [];
proj_points = [proj_points;[X_im(1), X_im(2)]];
X_im
end
end
