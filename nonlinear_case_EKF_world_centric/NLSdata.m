%%
clc;
clear;
close all;

%% Initialization
op = 1;     % 1 predetermined; 2 trace(P); 3 distance; 4 trace(P)+d
op_trans = 2;   % 1 straight line; 2 circle
op_noise = 1;   % 1 load; 2 related to distance

N = 20;
M = 3;
d = 2;
dtheta = pi/10;

Pose = [0; 0; 0];
% V_angle = 1;

feature{1} = [0.5, 0.5; 0.9, 0.8; 0.3, 0.4]';
% feature{1} = [0.3, 0.4]';
translation = getTrans(op_trans, d, dtheta, N);
rotation_t = eye(2);
rotation_d = eye(2);
range = [0.1; 2];

if op == 2
    load result/EKF_N2_212 u
end
load sigma sigma_head sigma_odem sigma_smoo sigma_cons
load noise/noise2 noise_odem
load noise/noise2 noise_cons noise_smoo
if op_noise == 1
%     load sigma sigma_feat
    sigma_r_ = 0.05;
    sigma_b_ = 0.05;
    load noise/noise2 noise_feat
elseif op_noise == 2
    sigma_r0 = 0.01;
    sigma_r1 = 0.09;
    dr_sigma = (sigma_r1 - sigma_r0) / (range(2) - range(1));
    
    sigma_b0 = 0.01;
    sigma_b1 = 0.09;
    db_sigma = (sigma_b1 - sigma_b0) / (range(2) - range(1));
end
save data M d op range op_noise op_trans dtheta
%%
visble(:, 1) = ones(3, 1);

constraint_edge = [];
for i = 1:M
    for j = i+1:M
        constraint_edge = [constraint_edge; i, j];
    end
end
save data visble constraint_edge -append
%%
sigma_Odem = [];
sigma_Feat = [];
sigma_Cons = [];
sigma_Smoo = [];
for i = 1:N
    for j = 1:M
        feature_frame{j}(1:2, i) = feature{i}(1:2, j);
        feature{i + 1}(1:2, j) = feature{i}(1:2, j) + rotation_t * translation(1:2, i) + noise_smoo{j, i};
        d_local_feature{j}(1:2, i) = zeros(2,1);
        feature_frame{j}(1:2, i) = feature_frame{j}(1:2, i) + rotation_d * d_local_feature{j}(1:2, i) + noise_cons{j, i};
        local_cordinate_feature{j}(1:2, i) = feature_frame{j}(1:2, i) - feature_frame{1}(1:2, i);
        if i > 1
            m_smoo{j, i-1} = feature_frame{j}(1:2, i) - feature{i-1}(1:2, j);
        end
    end
    
    if i == 1
        x = Pose;
        for j = 1:M
            y = so2_exp(Pose(1))' * (feature_frame{j}(1:2, i) - Pose(2:3, i));
            q = sqrt(y' * y);
            zj = y;
            if op_noise == 2
                sigma_r{j, i} = sigma_r0 + dr_sigma * max(0, q - range(1));
                sigma_b{j, i} = sigma_b0 + db_sigma * max(0, q - range(1));
                noise_feat{j, i} = [normrnd(0, sigma_r{j, i}); normrnd(0, sigma_b{j, i})];
                while noise_feat{j, i}(1) > 2*sigma_r{j, i} || noise_feat{j, i}(1) < -2*sigma_r{j, i}
                    noise_feat{j, i}(1) = normrnd(0, sigma_r{j, i}, 1, 1);
                end
                while noise_feat{j, i}(2) > 2*sigma_b{j, i} || noise_feat{j, i}(2) < -2*sigma_b{j, i}
                    noise_feat{j, i}(2) = normrnd(0, sigma_b{j, i}, 1, 1);
                end 
            elseif op_noise == 1
                sigma_r{j, i} = sigma_r_;
                sigma_b{j, i} = sigma_b_;
            end
            zj = zj + noise_feat{j, i};
            m_feat{i, j} = zj;
            sigma_feat2 = [sigma_r{j, i}^2; sigma_b{j, i}^2];
            sigma_Feat = [sigma_Feat; sigma_feat2];
            
            xfj = x(2:3) + zj;
            x = [x; xfj];
        end
        xr = x(1:3);
        xf = x(4:end);
        xr_pre = xr;
        xf_pre = xf;
    else
        % smooth measurement
        for j = 1:M
            measurement_smoo{j, i-1} = rotation_t * translation(1:2, i-1) + d_local_feature{j}(1:2, i);
            sigma_Smoo = [sigma_Smoo; sigma_smoo^2 + sigma_cons^2; sigma_smoo^2 + sigma_cons^2];
            xf_pre(2*j-1:2*j) = xf_pre(2*j-1:2*j) + measurement_smoo{j, i-1};
        end
        xf = [xf; xf_pre];
        
        % prediction
        if op == 1
            if op_trans == 1
                u(:, i-1) = [0; d; 0];
            elseif op_trans == 2
                u(:, i-1) = [dtheta; d*cos(dtheta); d*sin(dtheta)];
            end
        end
        sigma_Odem = [sigma_Odem; sigma_head^2; sigma_odem^2; sigma_odem^2];
        
        theta = Pose(1, i-1);
        R=[cos(theta), -sin(theta);
            sin(theta), cos(theta)];
        motion = [u(1, i-1);R * u(2:3, i-1)];
        Pose(:, i) = Pose(:, i-1) + motion + noise_odem(:, i);
        
        theta = xr_pre(1);
        R=[cos(theta), -sin(theta);
            sin(theta), cos(theta)];
        motion = [u(1, i-1);R * u(2:3, i-1)];
        xr_pre = xr_pre + motion;
        xr = [xr; xr_pre];
        
        %feature measurement
        for j = 1:M
            y = so2_exp(Pose(1, i))' * (feature_frame{j}(1:2, i) - Pose(2:3, i));
            q = sqrt(y' * y);
%             zj = [q; wrapToPi(atan2(y(2),y(1)))];
            zj = y; 
            if q <= range(2) && q >= range(1)
                if op_noise == 2
                    sigma_r{j, i} = sigma_r0 + dr_sigma * max(0, q - range(1));
                    sigma_b{j, i} = sigma_b0 + db_sigma * max(0, q - range(1));
                    noise_feat{j, i} = [normrnd(0, sigma_r{j, i}); normrnd(0, sigma_b{j, i})];
                    while noise_feat{j, i}(1) > 2*sigma_r{j, i} || noise_feat{j, i}(1) < -2*sigma_r{j, i}
                        noise_feat{j, i}(1) = normrnd(0, sigma_r{j, i}, 1, 1);
                    end
                    while noise_feat{j, i}(2) > 2*sigma_b{j, i} || noise_feat{j, i}(2) < -2*sigma_b{j, i}
                        noise_feat{j, i}(2) = normrnd(0, sigma_b{j, i}, 1, 1);
                    end 
                else
                    sigma_r{j, i} = sigma_r_;
                    sigma_b{j, i} = sigma_b_;
                end
                zj = zj + noise_feat{j, i};
                m_feat{i, j} = zj;
                sigma_feat2 = [sigma_r{j, i}^2; sigma_b{j, i}^2];
                sigma_Feat = [sigma_Feat; sigma_feat2];
                
                visble(j, i) = 1;
            else
                visble(j, i) = 0;
                sigma_Feat = [sigma_Feat; 1; 1];
            end
        end
    end
        % construction measurement
        if size(constraint_edge, 1) ~= 0
            for j = 1:size(constraint_edge, 1)
                m_cons{i, j} = local_cordinate_feature{constraint_edge(j, 2)}(1:2, i) ...
                    - local_cordinate_feature{constraint_edge(j,1)}(1:2,i);
                sigma_Cons = [sigma_Cons; sigma_cons^2; sigma_cons^2];
            end
        else
            m_cons{i, j} = [];
        end
end
x0 = [xr; xf];
save NLSdata/N2_121 u m_feat m_cons m_smoo ...
    sigma_Odem sigma_Feat sigma_Cons sigma_Smoo ...
    x0 visble constraint_edge feature_frame Pose