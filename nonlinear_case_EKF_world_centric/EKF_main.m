%%
clc;
clear;
close all;
addpath('./Math/');

%% Initialization
op = 1;     % 1 predetermined; 2 active 
op_trans = 2;   % 1 straight path; 2 circle path
op_noise = 2;   % sigma_feat = 0.05; 2 sigma_feat is related to distance

record = 1;
if record == 1
    video = VideoWriter(['video/' num2str(op) num2str(op_trans) num2str(op_noise) '.avi']);
    video.FrameRate = 8;
    video.Quality = 100;
    open(video);
end

N = 40;
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

load sigma sigma_head sigma_odem sigma_smoo sigma_cons
load noise/noise2 noise_odem
load noise/noise2 noise_cons noise_smoo
if op_noise == 1
%     load sigma sigma_feat
    sigma_r_ = 0.05;
    sigma_b_ = 0.05;
    load noise/noise2 noise_feat
end
% sigma_odem = 0.02;
% sigma_smoo = 0.3;
% sigma_cons = 0.2;
% sigma_r = 0.05;
if op_noise == 2
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
num_unseen = 0;

constraint_edge = [];
for i = 1:M
    for j = i+1:M
        constraint_edge = [constraint_edge; i, j];
    end
end
save data visble constraint_edge -append
%%
er2 = 0;
feature_j = zeros(2 * i, M);
for i = 1:N
    f = figure(1);
    for j = 1:M
        feature_frame{j}(1:2, i) = feature{i}(1:2, j);
        feature{i + 1}(1:2, j) = feature{i}(1:2, j) + rotation_t * translation(1:2, i) + noise_smoo{j, i};
%         local_cordinate_feature{j}(1:2, i) = feature_frame{j}(1:2, i) - feature_frame{1}(1:2, i);
        d_local_feature{j}(1:2, i) = zeros(2,1);
        feature_frame{j}(1:2, i) = feature_frame{j}(1:2, i) + rotation_d * d_local_feature{j}(1:2, i) + noise_cons{j, i};
        local_cordinate_feature{j}(1:2, i) = feature_frame{j}(1:2, i) - feature_frame{1}(1:2, i);
        feature_i(:, j) = feature_frame{j}(1:2, i);
        feature_j(2*i-1:2*i, j) = feature_frame{j}(1:2, i);
    end
    edge = [feature_i, feature_frame{1}(1:2, i)];
    
    sigma_Feat{i} = [];
    vis_line = [];
    unvis_line = [];
    z = [];
    z_est = [];
    if i == 1
        x = Pose;
        cov_f = zeros(2 * M);
        for j = 1:M
            y = so2_exp(Pose(1))' * (feature_frame{j}(1:2, i) - Pose(2:3, i));
            q = sqrt(y' * y);
            zj = y;
%             zj = [q; wrapToPi(atan2(y(2),y(1)))];
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
            z = [z; zj];
            sigma_feat2 = [sigma_r{j, i}^2; sigma_b{j, i}^2];
            sigma_Feat{i} = [sigma_Feat{i}; sigma_feat2];
            
%             Cos = cos(x(1) + zj(2));
%             Sin = sin(x(1) + zj(2));
%             J_f = [Cos, -zj(1)*Sin; Sin, zj(1)*Cos];
%             cov_f(2*j-1:2*j, 2*j-1:2*j) = J_f * diag(sigma_feat2) * J_f';
%             xfj = x(2:3) + [zj(1) * cos(x(1) + zj(2)); zj(1) * sin(x(1) + zj(2))];
            xfj = x(2:3) + zj;
            x = [x; xfj];
            if visble(j, i) == 1
                vis_line = [vis_line, Pose(2:3, i), feature_i(:, j)];
            else
                unvis_line = [unvis_line, Pose(2:3, i), feature_i(:, j)];
            end
        end
        P = zeros(3 + 2*M, 3 + 2*M);
        P(4:end,4:end) = diag(sigma_Feat{i});
        X = x;
        x_pre = x;
        xr = X(2:3);
        
        %plot
%         for j = 1:M
%             plot(feature_frame{j}(1, i), feature_frame{j}(2, i), 'p', 'MarkerEdgeColor', [0 j/M j/M], 'MarkerFaceColor', [0 j/M j/M], 'MarkerSize', 10);
%             hold on;
%             plot(x(3 + 2 * j - 1), x(3 + 2 * j), '*', 'Color', [j/M, 0, j/M]);
%         end
        hfg = plot(feature_i(1, :), feature_i(2, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
        
        hfg1 = plot(feature_j(1,1), feature_j(2,1), '.--', 'color', [0 176 80]/250, 'linewidth',1.5, 'MarkerSize', 10);
        hfg2 = plot(feature_j(1,2), feature_j(2,2), '.--', 'color', [0 176 80]/250, 'linewidth',1.5, 'MarkerSize', 10);
        hfg3 = plot(feature_j(1,3), feature_j(2,3), '.--', 'color', [0 176 80]/250, 'linewidth',1.5, 'MarkerSize', 10);
        
        hedg = plot(edge(1, :), edge(2, :), '--k', 'linewidth',1);
        hrg = plot(Pose(2, :), Pose(3, :), 'k+-', 'MarkerSize', 10);
        hold on;

        if ~isempty(vis_line)
            hvis = plot(vis_line(1, :), vis_line(2, :),'-g','linewidth',1.5);
        end
        if ~isempty(unvis_line)
            hunvis = plot(unvis_line(1, :), unvis_line(2, :),'--y','linewidth',1.5);
        else
            hunvis = plot(0, 0,'--y','linewidth',1.5);
        end

        hr = plot(xr(1, :), xr(2, :), 'b.-', 'MarkerSize', 10);
        re = plot_elipse(xr(1:2), P(2:3, 2:3));
        hre = plot(re(1, :), re(2, :), '.b', 'MarkerSize',0.5);

%         hfp = plot(x_pre(4:2:end), x_pre(5:2:end), 'mx');
        hf = plot(x(4:2:end), x(5:2:end), 'r*', 'MarkerSize', 10);
        hold on;

        fe = [];
        for j = 1:M
            fej = plot_elipse(feature_frame{j}(1:2, i), P(3+2*j-1:2+2*j, 3+2*j-1:2+2*j));
            fe = [fe, fej];
        end
        hfe = plot(fe(1, :), fe(2, :), '.k', 'MarkerSize',0.5);
        
        set(gca,'FontSize',20);
        if op_trans == 1
            axis([0 80 -4 4]);
    %         axis([0 40 -2 2]);
        else
            axis([-8 8 -4 16]);
    %         axis([-8 8 -2 16]);
        end
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
    else
        % smooth measurement
        measurement_smooth = [];
        for j = 1:M
            measurement_smooth = [measurement_smooth; rotation_t * translation(1:2, i-1) + d_local_feature{j}(1:2, i)];
        end
        Q = blkdiag(sigma_head^2, sigma_odem^2 * eye(2), (sigma_smoo^2 + sigma_cons^2) * eye(2*M));
        
        % prediction
        save data Q P x measurement_smooth i -append
        u(:, i - 1) = getV(op);
        [~, Pose(:, i)] = predict(1, u(:,i-1), Q, P, Pose(:, i-1), measurement_smooth, M, noise_odem(:, i));
        [P_pre, x_pre] = predict(2, u(:,i-1), Q, P, x, measurement_smooth, M, noise_odem(:, i));
        %feature measurement
        for j = 1:M
            y = so2_exp(Pose(1, i))' * (feature_frame{j}(1:2, i) - Pose(2:3, i));
            q = sqrt(y' * y);
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
                z = [z; zj];
                sigma_feat2 = [sigma_r{j, i}^2; sigma_b{j, i}^2];
                sigma_Feat{i} = [sigma_Feat{i}; sigma_feat2];
                
                visble(j, i) = 1;
                vis_line = [vis_line, Pose(2:3, i), feature_i(:, j)];
                
                rot = so2_exp(x_pre(1));
                delta = x_pre(3+2*j-1:3+2*j) - x_pre(2:3);
                y = rot' * delta;
                z_est = [z_est; y];
                
                J = [0 -1; 1 0];
                Hij = [-J*y, -rot', rot'];
                H_feat{i, j} = Hij;
            else
                visble(j, i) = 0;
                unvis_line = [unvis_line, Pose(2:3, i), feature_i(:, j)];
                num_unseen = num_unseen + 1;
            end
        end
        % construction measurement
        if size(constraint_edge, 1) ~= 0
            for j = 1:size(constraint_edge, 1)
                measurement_construction{i, j} = local_cordinate_feature{constraint_edge(j, 2)}(1:2, i) ...
                    - local_cordinate_feature{constraint_edge(j,1)}(1:2,i);
            end
        else
            measurement_construction{i, j} = [];
        end
        
        %KF Gain
        N = diag(sigma_Feat{i});
        for j = 1:size(constraint_edge, 1)
            N = blkdiag(N, sigma_cons^2*eye(2));
        end
        [K, H] = KF_gain(P_pre, N, visble(:, i), constraint_edge, M, H_feat, i);
        
        % update
        [x, P] = KF_update(x_pre, K, measurement_construction, z, H, P_pre, i, constraint_edge, z_est);
        X = [X, x];
        xr = [xr, x(2:3)];
        
        % plot
        op_fig = 2;
        if op_fig == 1
%             for j = 1:M
%                 plot(feature_frame{j}(1, i), feature_frame{j}(2, i), 'p', 'MarkerEdgeColor', [0 j/M j/M], 'MarkerFaceColor', [0 j/M j/M], 'MarkerSize', 10);
%                 plot(x(3 + 2 * j - 1), x(3 + 2 * j), '*', 'Color', [j/M, 0, j/M]);
%             end
            hfg = plot(feature_i(1, :), feature_i(2, :), 'gp', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
            hold on;
            hedg = plot(edge(1, :), edge(2, :), '--k', 'linewidth',1);
            hrg = plot(Pose(2, :), Pose(3, :), 'k+', 'MarkerSize', 10);
            hold on;

            if ~isempty(vis_line)
                hvis = plot(vis_line(1, :), vis_line(2, :),'-g','linewidth',1.5);
            end
            if ~isempty(unvis_line)
                hunvis = plot(unvis_line(1, :), unvis_line(2, :),'--y','linewidth',1.5);
            else
                hunvis = plot(0, 0,'--y','linewidth',1.5);
            end

            hr = plot(xr(1, :), xr(2, :), 'b.-', 'MarkerSize', 10);
            re = plot_elipse(xr(1:2, i), P(2:3, 2:3));
            hre = plot(re(1, :), re(2, :), '.b', 'MarkerSize',0.5);

%             hfp = plot(x_pre(4:2:end), x_pre(5:2:end), 'mx');
            hf = plot(x(4:2:end), x(5:2:end), 'r*', 'MarkerSize', 6);
            hold on;

            fe = [];
            for j = 1:M
                fej = plot_elipse(feature_frame{j}(1:2, i), P(3+2*j-1:3+2*j, 3+2*j-1:3+2*j));
                fe = [fe, fej];
            end
            hfe = plot(fe(1, :), fe(2, :), '.k', 'MarkerSize',0.5);
        elseif op_fig == 2
            %plot
            set(hfg, 'XData', feature_i(1, :), 'YData', feature_i(2, :));
            
            set(hfg1, 'XData', feature_j(1:2:end,1), 'YData', feature_j(2:2:end,1));
            set(hfg2, 'XData', feature_j(1:2:end,2), 'YData', feature_j(2:2:end,2));
            set(hfg3, 'XData', feature_j(1:2:end,3), 'YData', feature_j(2:2:end,3));
            
            set(hedg, 'XData', edge(1, :), 'YData', edge(2, :));
            set(hrg, 'XData', Pose(2, :), 'YData', Pose(3, :));

            if ~isempty(vis_line)
                set(hvis, 'XData', vis_line(1, :), 'YData', vis_line(2, :));
            else
                set(hvis, 'XData', 0, 'YData', 0);            
            end
            if ~isempty(unvis_line)
                set(hunvis, 'XData', unvis_line(1, :), 'YData', unvis_line(2, :));
            else
                set(hunvis, 'XData', 0, 'YData', 0);     
            end

            set(hr, 'XData', xr(1, :), 'YData', xr(2, :));
            re = plot_elipse(xr(1:2, i), P(2:3, 2:3));
            set(hre, 'XData', re(1, :), 'YData', re(2, :));

    %         set(hfp, 'XData', x_pre(4:2:end), 'YData', x_pre(5:2:end));
            set(hf, 'XData', x(4:2:end), 'YData', x(5:2:end));
            fe = [];
            for j = 1:M
                fej = plot_elipse(feature_frame{j}(1:2, i), P(3+2*j-1:3+2*j, 3+2*j-1:3+2*j));
                fe = [fe, fej];
            end
            set(hfe, 'XData', fe(1, :), 'YData', fe(2, :));
        end
    end
%     axis([0 5 0 15.5]);
%     pause(0.5);
if record == 1
    F(i) = getframe(f);
    writeVideo(video, F(i));
end
    er = x(1:3) - Pose(:, i);
    er2 = er2 + er' * er;
end
if record == 1
    close(video);
end
u
traceP = trace(P)
num_unseen
er2
% save result/EKF_N2_222 u traceP num_unseen
