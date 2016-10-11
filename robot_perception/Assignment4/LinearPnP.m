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


numpts = size(x,1);
if isempty(X) || isempty(x)
    disp('problem');
end


if size(x,2) == 2
    x = [x, ones(size(x,1),1)];
end


if size(X,2) == 3
    X = [X, ones(size(X,1),1)];
end

vec2skew = @(v) [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];

Xdiag = zeros(3,12);
A = [];
for i=1:numpts
    Xdiag(1, 1:4) = X(i,:);
    Xdiag(2, 5:8) = X(i,:);
    Xdiag(3, 9:12) = X(i,:);
    skew = vec2skew(inv(K)*x(i,:)');
    A = [A; skew*Xdiag];

end

%least squares to solve for P
[~,~,V] = svd(A);
Pvec = V(:,end);
P = [Pvec(1:4)';Pvec(5:8)';Pvec(9:12)'];

% check
check = zeros(3, numpts);
for i = 1:numpts
    check(:, i) = vec2skew(inv(K)*x(i,:)')*P*X(i,:)';
end

%reconditioning
RC = P;
[U,D,V] = svd(RC(:,1:3));
R = sign(det(U*V'))*U*V';
C = -R'*RC(:,4)/D(1,1);

end