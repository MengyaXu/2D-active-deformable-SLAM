%%
clc;
clear;
close all;
addpath('./Math/');

%% Initialization
op = 1;     % 1 predetermined 2 active 
op_trans = 1;   % 1 straight path; 2 circle path
op_noise = 2;   % sigma_feat = 0.05; 2 sigma_feat is related to distance

record = 1;
if record == 1
    video = VideoWriter(['video/feat' num2str(op) num2str(op_trans) num2str(op_noise) '.avi']);
    video.FrameRate = 8;
    video.Quality = 100;
    open(video);
end

Num_pose = 40;
Num_feature = 3;
d = 2;

Pose = [0; 0];
V_angle = 0;

feature{1} = [0.5, 0.5; 0.9, 0.8; 0.3, 0.4]';
translation = getTrans(op_trans, d, Num_pose);
rotation_t = eye(2);
rotation_d = eye(2);
range = [0.1; 2];

load sigma sigma_odem sigma_smoo sigma_cons
load noise/noise noise_odem 
load noise/noise noise_smoo noise_cons
if op_noise == 1
%     load sigma sigma_feat
    sigma_feat_ = 0.05;
    load noise/noise noise_feat
end
% sigma_odem = 0.02;
% sigma_smoo = 0.3;
% sigma_cons = 0.2;
% sigma_feat = 0.05;
if op_noise == 2
    sigma_feat0 = 0.01;
    sigma_feat1 = 0.09;
    d_sigma = (sigma_feat1 - sigma_feat0) / (range(2) - range(1));
end
save data Num_feature d op range op_noise op_trans
%%
visble(:, 1) = ones(3, 1);
num_unseen = 0;

% for i = 2:1:Num_pose
%     for j = 1:1:Num_feature
%         visble(j, i) = 1;
%     end
% end
% visble(2, 47:49) = zeros(1, 3);
% visble

constraint_edge = [];
for i = 1:Num_feature
    for j = i+1:Num_feature
        constraint_edge = [constraint_edge; i, j];
    end
end
save data visble constraint_edge -append
%%
feature_j = zeros(2 * i, Num_feature);
for i = 1:Num_pose
    f = figure(1);
    for j = 1:Num_feature
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
    
    sigma_Feat = [];
    vis_line = [];
    unvis_line = [];
    if i == 1
        Pose(1, i) = 0;
        Pose(2, i) = 0;
        V_angle(i) = 0;
        X_update = zeros(2, 1);
        for j = 1:Num_feature
            measurement_feature{i, j} = so2_exp(V_angle(i))' * (feature_frame{j}(1:2, i) - Pose(1:2, i));
            if op_noise == 2
                q = sqrt(measurement_feature{i, j}(1)^2 + measurement_feature{i, j}(2)^2);
                sigma_feat{j, i} = sigma_feat0 + d_sigma * max(0, q - range(1));
                noise_feat{j, i} = normrnd(0, sigma_feat{j, i}, 2, 1);
                while noise_feat{j, i}(1) > 2*sigma_feat{j, i}
                    noise_feat{j, i}(1) = normrnd(0, sigma_feat{j, i});
                end
                while noise_feat{j, i}(2) > 2*sigma_feat{j, i}
                    noise_feat{j, i}(2) = normrnd(0, sigma_feat{j, i});
                end 
            else
                sigma_feat{j, i} = sigma_feat_;
            end
            sigma_Feat = [sigma_Feat; sigma_feat{j, i}^2; sigma_feat{j, i}^2];
            X_update = [X_update; feature_frame{j}(1:2, 1) + noise_feat{j, i}];
            if visble(j, i) == 1
                vis_line = [vis_line, Pose(:, i), feature_i(:, j)];
            else
                unvis_line = [unvis_line, Pose(:, i), feature_i(:, j)];
            end
        end
        PX = zeros(2*(Num_feature+1),2*(Num_feature+1));
        PX(3:end,3:end) = diag(sigma_Feat);
        X = X_update;
        X_update_predict = X;
        xr = X(1:2);
        
        %plot
        hfg = plot(feature_i(1, :), feature_i(2, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
        
        hfg1 = plot(feature_j(1,1), feature_j(2,1), '.--', 'color', [0 176 80]/250, 'linewidth',1.5, 'MarkerSize', 10);
        hfg2 = plot(feature_j(1,2), feature_j(2,2), '.--', 'color', [0 176 80]/250, 'linewidth',1.5, 'MarkerSize', 10);
        hfg3 = plot(feature_j(1,3), feature_j(2,3), '.--', 'color', [0 176 80]/250, 'linewidth',1.5, 'MarkerSize', 10);
        
        hedg = plot(edge(1, :), edge(2, :), '--k', 'linewidth',1);
%         hrg = plot(Pose(1, :), Pose(2, :), 'k+', 'MarkerSize', 10);
%         hold on;
% 
%         if ~isempty(vis_line)
%             hvis = plot(vis_line(1, :), vis_line(2, :),'-g','linewidth',1.5);
%         end
%         if ~isempty(unvis_line)
%             hunvis = plot(unvis_line(1, :), unvis_line(2, :),'--y','linewidth',1.5);
%         else
%             hunvis = plot(0, 0,'--y','linewidth',1.5);
%         end
% 
%         hr = plot(xr(1, :), xr(2, :), 'b.-', 'MarkerSize', 10);
%         re = plot_elipse(X_update(1:2), PX(1:2, 1:2));
%         hre = plot(re(1, :), re(2, :), '.b', 'MarkerSize',0.5);
% 
%         hf = plot(X_update(3:2:end), X_update(4:2:end), 'r*', 'MarkerSize', 10);
%         hold on;
% 
%         fe = [];
%         for j = 1:Num_feature
%             fej = plot_elipse(feature_frame{j}(1:2, i), PX(2+2*j-1:2+2*j, 2+2*j-1:2+2*j));
%             fe = [fe, fej];
%         end
%         hfe = plot(fe(1, :), fe(2, :), '.k', 'MarkerSize',0.5);
%         
            set(gca,'FontSize',20);
        if op_trans == 1
            axis([0 85 -3 3]);
    %         axis([0 45 -1.5 2]);
        else
            axis([-10 10 -3 16]);
    %         axis([-10 8 -2 16]);
        end
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
    else
        % smooth measurement
        for j = 1:Num_feature
            measurement_smooth{j, i} = rotation_t * translation(1:2, i-1) + d_local_feature{j}(1:2, i);
        end
        Q = blkdiag(sigma_odem^2 * eye(2), (sigma_smoo^2 + sigma_cons^2) * eye(2),...
            (sigma_smoo^2 + sigma_cons^2)*eye(2),(sigma_smoo^2 + sigma_cons^2)*eye(2));
        
        % prediction
        save data Q PX X_update measurement_smooth i -append
        v(1:2, i - 1) = getV(op);
        
        Pose(:, i) = Pose(:, i-1) + v(:,i-1) + noise_odem(:, i);
        V_angle(i) = 0;
        
        [PX_predict, X_update_predict] = predict(v(:,i-1), V_angle(i), Q, PX, X_update, measurement_smooth, i);
        %feature measurement
        for j = 1:Num_feature
            measurement_feature{i, j} = so2_exp(V_angle(i))' * (feature_frame{j}(1:2, i) - Pose(1:2, i));
            q = sqrt(measurement_feature{i, j}(1)^2 + measurement_feature{i, j}(2)^2);
            if q <= range(2) && q >= range(1)
                if op_noise == 2
                    sigma_feat{j, i} = sigma_feat0 + d_sigma * max(0, q - range(1));
                    noise_feat{j, i} = normrnd(0, sigma_feat{j, i}, 2, 1);
                    while noise_feat{j, i}(1) > 2*sigma_feat{j, i}
                        noise_feat{j, i}(1) = normrnd(0, sigma_feat{j, i});
                    end
                    while noise_feat{j, i}(2) > 2*sigma_feat{j, i}
                        noise_feat{j, i}(2) = normrnd(0, sigma_feat{j, i});
                    end 
                else
                    sigma_feat{j, i} = sigma_feat_;
                end
                measurement_feature{i, j} = measurement_feature{i, j} + noise_feat{j, i};
                visble(j, i) = 1;
                vis_line = [vis_line, Pose(:, i), feature_i(:, j)];
                sigma_Feat = [sigma_Feat; sigma_feat{j, i}^2; sigma_feat{j, i}^2];
            else
                visble(j, i) = 0;
                unvis_line = [unvis_line, Pose(:, i), feature_i(:, j)];
                num_unseen = num_unseen + 1;
                measurement_feature{i, j} = [];
            end
        end
        % construction measurement
        for j = 1:size(constraint_edge, 1)
            measurement_construction{i, j} = local_cordinate_feature{constraint_edge(j, 2)}(1:2, i) + ...
                - local_cordinate_feature{constraint_edge(j,1)}(1:2,i);
        end
        
        %KF Gain
        N = diag(sigma_Feat);
        for j = 1:size(constraint_edge, 1)
            N = blkdiag(N, sigma_cons^2*eye(2));
        end
        [K, H] = KF_gain(PX_predict, V_angle(i), N, visble(:, i), constraint_edge, Num_feature);
        % update
%         z = so2_exp(0)' * (X_update_predict(2 + 2 *j - 1: 2 + 2 *j) - X_update_predict(1:2))
        [X_update, PX] = KF_update(X_update_predict, K, measurement_construction, measurement_feature, H, PX_predict, i, constraint_edge);
        X = [X, X_update];
        xr = [xr, X_update(1:2)];
        
        % plot
        %plot
        set(hfg, 'XData', feature_i(1, :), 'YData', feature_i(2, :));
        
        set(hfg1, 'XData', feature_j(1:2:end,1), 'YData', feature_j(2:2:end,1));
        set(hfg2, 'XData', feature_j(1:2:end,2), 'YData', feature_j(2:2:end,2));
        set(hfg3, 'XData', feature_j(1:2:end,3), 'YData', feature_j(2:2:end,3));
            
        set(hedg, 'XData', edge(1, :), 'YData', edge(2, :));
%         set(hrg, 'XData', Pose(1, :), 'YData', Pose(2, :));
% 
%         if ~isempty(vis_line)
%             set(hvis, 'XData', vis_line(1, :), 'YData', vis_line(2, :));
%         else
%             set(hvis, 'XData', 0, 'YData', 0);            
%         end
%         if ~isempty(unvis_line)
%             set(hunvis, 'XData', unvis_line(1, :), 'YData', unvis_line(2, :));
%         else
%             set(hunvis, 'XData', 0, 'YData', 0);     
%         end
% 
%         set(hr, 'XData', xr(1, :), 'YData', xr(2, :));
%         re = plot_elipse(X_update(1:2), PX(1:2, 1:2));
%         set(hre, 'XData', re(1, :), 'YData', re(2, :));
% 
%         set(hf, 'XData', X_update(3:2:end), 'YData', X_update(4:2:end));
%         fe = [];
%         for j = 1:Num_feature
%             fej = plot_elipse(feature_frame{j}(1:2, i), PX(2+2*j-1:2+2*j, 2+2*j-1:2+2*j));
%             fe = [fe, fej];
%         end
%         set(hfe, 'XData', fe(1, :), 'YData', fe(2, :));
    end
        
    if record == 1
        F(i) = getframe(f);
        writeVideo(video, F(i));
    end
end
if record == 1
    close(video);
end
v
trace(PX)
num_unseen