function obj = objFun(V)
load data op op_noise range Q P x measurement_smooth i visble constraint_edge M
load sigma sigma_cons
[P_pre, x_pre] = predict(2, V, Q, P, x, measurement_smooth, M, 0);

% R=[cos(V_angle), -sin(V_angle);
%     sin(V_angle), cos(V_angle);];
% GX=blkdiag(R,eye(2),eye(2),eye(2));
% X_update_predict=GX*X_update+...
%     [V;0;...
%     measurement_smooth{1,i};measurement_smooth{2,i};
%     measurement_smooth{3,i}];%
% PX_predict=GX*PX*GX'+Q;
d = 0;
if op_noise == 2
    sigma_r0 = 0.01;
    sigma_r1 = 0.09;
    dr_sigma = (sigma_r1 - sigma_r0) / (range(2) - range(1));
    
    sigma_b0 = 0.01;
    sigma_b1 = 0.09;
    db_sigma = (sigma_b1 - sigma_b0) / (range(2) - range(1));
    
else
    sigma_r_ = 0.05;
    sigma_b_ = 0.04;
end
sigma_Feat  =[];
for j = 1:M
    y = so2_exp(x_pre(1))' * (x_pre(3+2*j-1:3+2*j) - x_pre(2:3));
    q = sqrt(y'*y);
    
    rot = so2_exp(x_pre(1));
    J = [0 -1; 1 0];
    dh = [y(1)/q y(2)/q; -y(2)/q^2 y(1)/q^2];
    Hij = [-J*y, -rot', rot'];
    H_feat{i, j} = dh * Hij;
    if q <= range(2) && q >= range(1)
        if op_noise == 2
            sigma_r{j, i} = sigma_r0 + dr_sigma * max(0, q - range(1));
            sigma_b{j, i} = sigma_b0 + db_sigma * max(0, q - range(1));
        else
            sigma_r{j, i} = sigma_r_;
            sigma_b{j, i} = sigma_b_;
        end
        sigma_Feat = [sigma_Feat; sigma_r{j, i}^2; sigma_b{j, i}^2];
        visble(j, i) = 1;
        d = d + q;
    else
        visble(j, i) = 0;
        d = d + 2;
    end
end
if op == 2 || op == 4
    N = diag(sigma_Feat);
    for j = 1:size(constraint_edge, 1)
        N = blkdiag(N, sigma_cons^2*eye(2));
    end
    [K, H] = KF_gain(P_pre, N, visble(:, i), constraint_edge, M, H_feat, i);
    PX=(eye(size(P_pre,1))-K*H)*P_pre;
    if op == 2
        obj = trace(PX);
    elseif op == 4
        obj1 = trace(PX);
        t= log10(obj1);
        scale = real(floor(t));
        % 10^scale * d
        obj = trace(PX) + 10^scale * d / 3;
        % trace(PX)
    end
elseif op == 3
    obj = d;
end
end

