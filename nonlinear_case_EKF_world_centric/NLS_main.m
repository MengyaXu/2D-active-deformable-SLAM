clc;
clear;
close all;

load NLSdata/N2_121 x0 u  feature_frame Pose visble

N = 20;
M = 3;

c = 10^(-6);
[I, dx] = GN(N, M, x0);
dx
ep = 0;
while sum(abs(dx)) > c || sum(abs(dx)) < -c
    x0 = x0 + dx;
    for i = 1: N
        x0(3*N - 2) = wrapToPi(x0(3*N - 2));
    end
    [I, dx] = GN(N, M, x0);
    dx
    ep = ep+1;
    if ep > 30
        b = 1
        dx;
        break;
    end
end
ep
P = full(I(4:end, 4:end)^(-1));
P = blkdiag(zeros(3, 3), P);

% plot
er2 = 0;
for i = 1:N
    er = x0(3 * i - 2: 3 * i) - Pose(:, i);
    er(1) = wrapToPi(er(1));
    er2 = er2 + er' * er;
    vis_line = [];
    unvis_line = [];
    for j = 1:M
        feature_i(:, j) = feature_frame{j}(1:2, i);
        
        if visble(j, i) == 1
            vis_line = [vis_line, Pose(2:3, i), feature_i(:, j)];
        else
            unvis_line = [unvis_line, Pose(2:3, i), feature_i(:, j)];
        end
    end
    edge = [feature_i, feature_frame{1}(1:2, i)];
    
    hfg = plot(feature_i(1, :), feature_i(2, :), 'gp', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    hold on;
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

    re = plot_elipse(x0(3 * i - 1:3 * i), P(3 * i - 1:3 * i, 3 * i - 1:3 * i));
    hre = plot(re(1, :), re(2, :), '.b', 'MarkerSize',0.5);

    fe = [];
    for j = 1:M
        fej = plot_elipse(feature_frame{j}(1:2, i), P(3*N+2*(i-1)*M+2*j-1:3*N+2*(i-1)*M+2*j, 3*N+2*(i-1)*M+2*j-1:3*N+2*(i-1)*M+2*j));
        fe = [fe, fej];
    end
    hfe = plot(fe(1, :), fe(2, :), '.k', 'MarkerSize',0.5);
end
    hr = plot(x0(2:3:3*N), x0(3:3:3*N), 'b.-', 'MarkerSize', 10);
    hf = plot(x0(3*N+1:2:end), x0(3*N+2:2:end), 'r*', 'MarkerSize', 6);
    hold on;
    
    er2