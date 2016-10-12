function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 3) pose rotation


%converting to homogenious system
X = [X , ones(size(X,1),1)];
A = zeros(3 * size(X,1), 12);

for i = 1 : size(X,1)
    Xt = X(i,:);
    x_temp = [x(i,:),1];
    xx = K \ x_temp';    
    j = (i-1)*3 + 1;
    A(j:j+2,:) = [0, -1, xx(2); 1, 0, -xx(1); -xx(2), xx(1), 0] * ...
                 [Xt, zeros(1,4), zeros(1,4); zeros(1,4), Xt, zeros(1,4); ...
                  zeros(1,4), zeros(1,4), Xt];
end
[u,d,v] = svd(A);
P_temp = v(:,end);
P = reshape(P_temp, 4, 3)';


R = P(:,1:3);
[uu,dd,vv] = svd(R);
if det(uu * vv') > 0
    R = uu * vv';
    T = P(:,4) ./ dd(1,1);
else
    R = -uu * vv';
    T = -P(:,4) ./ dd(1,1);
end
%C = -R' * T;