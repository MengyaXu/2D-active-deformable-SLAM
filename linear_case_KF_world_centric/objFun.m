function obj = objFun(V)
load data op op_noise range Q PX X_update measurement_smooth i Num_feature visble constraint_edge
load sigma sigma_cons sigma_feat
V_angle = 0;
[PX_predict, X_update_predict] = predict(V, V_angle, Q, PX, X_update, measurement_smooth, i);

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
    sigma_feat0 = 0.01;
    sigma_feat1 = 0.09;
    d_sigma = (sigma_feat1 - sigma_feat0) / (range(2) - range(1));
    sigma_feat = zeros(Num_feature, 1);
end
for j = 1:Num_feature
    measurement_feature{i, j} = so2_exp(V_angle)' * (X_update_predict(2+2*j-1:2+2*j) - X_update_predict(1:2));
    q = sqrt(measurement_feature{i, j}(1)^2 + measurement_feature{i, j}(2)^2);
    if q <= range(2) && q >= range(1)
        if op_noise == 2
            sigma_feat(j) = sigma_feat0 + d_sigma * max(0, q - range(1));
        end
        visble(j, i) = 1;
        d = d + q;
    else
        visble(j, i) = 0;
        d = d + 2;
    end
end
if op == 2 || op == 4
    N = [];
    for j = 1:Num_feature
        if visble(j, i) == 1    % predict visble
            if op_noise == 2
                N = blkdiag(N, sigma_feat(j)^2*eye(2));
            else
                N = blkdiag(N, sigma_feat^2*eye(2));
            end
        end
    end
    for j = 1:size(constraint_edge, 1)
        N = blkdiag(N, sigma_cons^2*eye(2));
    end
    [K, H] = KF_gain(PX_predict, V_angle, N, visble(:, i), constraint_edge, Num_feature);

    % 
    % rows=[];
    % columns=[];
    % values=[];
    % k=0;
    % for j=1:1:Num_feature
    %     if visble(j, i)==1
    %         k=k+1;
    %         rows=[rows,2*k-1,2*k-1,2*k,2*k,2*k-1,2*k-1,2*k,2*k];
    %         columns=[columns,1,2,1,2,2+2*j-1,2+2*j,2+2*j-1,2+2*j];
    %         values=[values,-cos(V_angle),sin(V_angle),-sin(V_angle),-cos(V_angle),...
    %             cos(V_angle),-sin(V_angle),sin(V_angle),cos(V_angle)];
    %     end
    % end
    % for j=1:1:size(constraint_edge,1)
    %     in=constraint_edge(j,1);
    %     out=constraint_edge(j,2);
    %     rows=[rows,2*k+2*j-1,2*k+2*j-1,2*k+2*j,2*k+2*j,2*k+2*j-1,2*k+2*j-1,2*k+2*j,2*k+2*j];
    %     columns=[columns,2+2*in-1,2+2*in,2+2*in-1,2+2*in,...
    %         2+2*out-1,2+2*out,2+2*out-1,2+2*out];
    %     values=[values,-1,0,0,-1,1,0,0,1];
    % end
    % H=sparse(rows,columns,values,2*k+size(constraint_edge,1)*2,size(PX_predict,1));
    % K=PX_predict*H'/(H*PX_predict*H'+N);
    PX=(eye(size(PX_predict,1))-K*H)*PX_predict;
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

