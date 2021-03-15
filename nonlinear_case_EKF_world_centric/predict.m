function [P_pre,x_pre]=predict(op, u, Q, P, x, measurement_smooth, M, noise_odem)
theta = x(1);
R=[cos(theta), -sin(theta);
    sin(theta), cos(theta)];
motion = [u(1);R * u(2:3); measurement_smooth];
if op == 1  % actual pose
    x_pre = x + motion(1:3) + noise_odem;
    P_pre = [];
elseif op == 2  % predict vector
    x_pre = x + motion;
    
    J = [0 -1; 1 0];
    Fx = [1 0 0; R * J * u(2:3), eye(2)];
    F = blkdiag(Fx, eye(2*M));
    G = blkdiag(1, R, eye(2*M));
    P_pre=F*P*F' + G*Q*G';
end
% x_pre(1) = wrapToPi(x_pre(1));