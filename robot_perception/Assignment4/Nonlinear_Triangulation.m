function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix for both
%     cameras
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 
X = zeros(size(X0));
for i = 1 : size(X0,1)
    X(i,:) = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:));
end
end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
n_max = 1;
i = 1;
while i <= n_max  % result is correct when iterate once
    i = i + 1;
    J1 = Jacobian_Triangulation(C1, R1, K, X0);
    J2 = Jacobian_Triangulation(C2, R2, K, X0);
    J3 = Jacobian_Triangulation(C3, R3, K, X0);
    J = [J1; J2; J3];
    b = [x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
    x1 = K * R1 * (X0' - C1);
    u1 = x1(1);
    v1 = x1(2);
    w1 = x1(3);
    x2 = K * R2 * (X0' - C2);
    u2 = x2(1);
    v2 = x2(2);
    w2 = x2(3);
    x3 = K * R3 * (X0' - C3);
    u3 = x3(1);
    v3 = x3(2);
    w3 = x3(3);
    fx = [u1 / w1, v1 / w1, u2 / w2, v2 / w2, u3 / w3, v3 / w3]';
    delta_x = (J' * J) \ J' * (b - fx);
    X0 = X0 + delta_x';
end
X = X0;
end
function J = Jacobian_Triangulation(C, R, K, X)
x = K * R * (X' - C);
u = x(1);
v = x(2);
w = x(3);
f = K(1,1);
px = K(1,3);
py = K(2,3);
dudx = [f * R(1,1) + px * R(3,1), f * R(1,2) + px * R(3,2), f * R(1,3) + px * R(3,3)];
dvdx = [f * R(2,1) + py * R(3,1), f * R(2,2) + py * R(3,2), f * R(2,3) + py * R(3,3)];
dwdx = [R(3,1), R(3,2), R(3,3)];
dfdx = [(w .* dudx - u .* dwdx)./ w ^ 2 ;(w .* dvdx - v .* dwdx)./ w ^ 2 ];
J = dfdx;
end