function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

A = [];
for i = 1:4
     x=[-video_pts(i,1), video_pts(i,2), -1, 0, 0, 0, video_pts(i,1)*logo_pts(i,1), video_pts(i,2)*logo_pts(i,1), logo_pts(i,1)];
     y=[0, 0, 0, -video_pts(i,1), -video_pts(i,2), -1, video_pts(i,1)*logo_pts(i,2), video_pts(i,2)*logo_pts(i,2), logo_pts(i,2)];
     A = [A;x;y];
end

size(A)
[U, S, V] = svd(A);
V
Z = V(:,9);
H = reshape(Z, [3,3]);
H = H';
end

