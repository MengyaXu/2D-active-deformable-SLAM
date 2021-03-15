function [I, dx] = GN(N, M, x0)
load NLSdata/N2_121 u m_feat m_cons m_smoo ...
    sigma_Odem sigma_Feat sigma_Cons sigma_Smoo ...
    visble constraint_edge
load noise/noise2 noise_odem
J_const = [0 -1; 1 0];

row=[];
column=[];
value=[];
F=[];
row_Pw=[];
column_Pw=[];
value_Pw=[];

% odem
xr = [0;0;0; x0(4:3*N)];
for i = 1:N-1
    theta = xr(3*i - 2);
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    motion = [u(1, i);rot * u(2:3, i)];
    x_est(:, i+1) = xr(3*i - 2 : 3*i) + motion; %+ noise_odem(:, i);
    
    fxi = xr(3*i + 1 : 3*i + 3) - x_est(:, i+1);
    fxi(1) = wrapToPi(fxi(1));
    F = [F; fxi];
    
    Jxi = [[-1 0 0;
           -rot * J_const * u(2:3, i), -diag([1 1])] eye(3)];
    
    row=[row, (3*i-2)*ones(1, 6), (3*i-1)*ones(1, 6), (3*i)*ones(1, 6)];
    column=[column,3*i-2:3*i+3, 3*i-2:3*i+3, 3*i-2:3*i+3];
    value=[value, Jxi(1, :), Jxi(2, :), Jxi(3, :)];
end

% m_feat
for i = 1:N
    for j = 1:M
            theta = x0(3*i - 2);
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            delta = x0(3*N + 2*(i-1)*M + 2*j-1:3*N + 2*(i-1)*M + 2*j) - xr(3*i-1:3*i);
            z_est = rot' * delta;
        if visble(j, i) == 1
%             theta = x0(3*i - 2);
%             rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%             delta = x0(3*N + 2*(i-1)*M + 2*j-1:3*N + 2*(i-1)*M + 2*j) - xr(3*i-1:3*i);
%             z_est = rot' * delta;
            
            zj = m_feat{i, j};
            fz = zj - z_est;
            F=[F; fz];
        else
            F = [F; 0; 0];

%             Hij = -[-J_const*z_est, -rot', rot'];
%             row=[row,(3*(N-1) + sum(sum(visble(1:end,1:i-1)))*2 + 2*sum(sum(visble(1:j,i)))-1)*ones(1, 5), ...
%                 (3*(N-1) + sum(sum(visble(1:end,1:i-1)))*2 + 2*sum(sum(visble(1:j,i))))*ones(1, 5)];
%             column=[column, 3*i-2:3*i, 3*N+(i-1)*M*2+2*j-1, 3*N+(i-1)*M*2+2*j, ...
%                 3*i-2:3*i, 3*N+(i-1)*M*2+2*j-1, 3*N+(i-1)*M*2+2*j];
%             value=[value, Hij(1, :), Hij(2, :)];
        end

            Hij = -[-J_const*z_est, -rot', rot'];
            row=[row,(3*(N-1) + (i-1)*M*2 + 2*j-1)*ones(1, 5), ...
                (3*(N-1) + (i-1)*M*2 + 2*j)*ones(1, 5)];
            column=[column, 3*i-2:3*i, 3*N+(i-1)*M*2+2*j-1, 3*N+(i-1)*M*2+2*j, ...
                3*i-2:3*i, 3*N+(i-1)*M*2+2*j-1, 3*N+(i-1)*M*2+2*j];
            value=[value, Hij(1, :), Hij(2, :)];
    end
end

% m_cons
add_num = 3*(N - 1) + M * N * 2;
% add_num = 3*(N - 1) + sum(sum(visble)) * 2;
for i = 1:N
    for j = 1:M
        local_cordinate_feature{j}(1:2,i)=x0(3*N+(i-1)*M*2+2*j-1:3*N+(i-1)*M*2+2*j) - x0(3*N+(i-1)*M*2+1:3*N+(i-1)*M*2+2);
    end
    for j = 1:size(constraint_edge,1)
        est_cons{i, j} = local_cordinate_feature{constraint_edge(j, 2)}(1:2, i) ...
                    - local_cordinate_feature{constraint_edge(j,1)}(1:2,i);
        fcons = m_cons{i, j} - est_cons{i, j}
        F=[F; fcons];
        
        row=[row,(add_num+(i-1)*size(constraint_edge,1)*2+j*2-1)*ones(1, 4), ...
            (add_num+(i-1)*size(constraint_edge,1)*2+j*2)*ones(1, 4)];
        column=[column,3*N+(i-1)*M*2+constraint_edge(j,1)*2-1, ...
            3*N+(i-1)*M*2+constraint_edge(j,1)*2,...
            3*N+(i-1)*M*2+constraint_edge(j,2)*2-1, ...
            3*N+(i-1)*M*2+constraint_edge(j,2)*2, ...
            3*N+(i-1)*M*2+constraint_edge(j,1)*2-1, ...
            3*N+(i-1)*M*2+constraint_edge(j,1)*2,...
            3*N+(i-1)*M*2+constraint_edge(j,2)*2-1, ...
            3*N+(i-1)*M*2+constraint_edge(j,2)*2];
        value=[value, 1, 0, -1, 0, 0, 1, 0, -1];
    end
end

% m_smoo
% add_num = 3*(N-1) + sum(sum(visble))*2 + 2*N*size(constraint_edge,1);
add_num = 3*(N-1) + M*N*2 + 2*N*size(constraint_edge,1);
for i=1:M
    for j=1:N-1
        est_smoo = x0(3*N+j*M*2+i*2-1:3*N+j*M*2+i*2) - x0(3*N+(j-1)*M*2+i*2-1:3*N+(j-1)*M*2+i*2);
        fsmoo = m_smoo{i, j} - est_smoo;
        F = [F; fsmoo];
%         F = [F; 0; 0];
        
        row=[row,(add_num+(i-1)*(N-1)*2+j*2-1)*ones(1, 4), ...
            (add_num+(i-1)*(N-1)*2+j*2)*ones(1, 4)];
        column=[column,3*N+(j-1)*M*2+i*2-1, 3*N+(j-1)*M*2+i*2,...
            3*N+j*M*2+i*2-1, 3*N+j*M*2+i*2, ...
            3*N+(j-1)*M*2+i*2-1, 3*N+(j-1)*M*2+i*2,...
            3*N+j*M*2+i*2-1, 3*N+j*M*2+i*2];
        value=[value, 1, 0, -1, 0, 0, 1, 0, -1];
    end
end

J = sparse(row, column, value, add_num+M*(N-1)*2, M*N*2+3*N);
Pw = sparse(diag([sigma_Odem; sigma_Feat; sigma_Cons; sigma_Smoo]));
H = J' / Pw * J;
g = -J' / Pw * F;
dx = [zeros(3,1); H(4:end, 4:end) \ g(4:end)];
I = H;