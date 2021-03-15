sigma_head = 0.01;
sigma_odem = 0.02;
sigma_smoo = 0.3;
sigma_cons = 0.2;
sigma_r = 0.05;
sigma_b = 0.05;

% sigma_head = 0;
% sigma_odem = 0;
% sigma_smoo = 0;
% sigma_cons = 0;
% sigma_r = 0;
% sigma_b = 0;

save sigma sigma_head sigma_odem sigma_smoo sigma_cons sigma_r sigma_b
noise_odem = [];
% noise_smoo = [];
% noise_cons = [];
% noise_feat = [];
for i = 1:100
    noise_odem = [noise_odem, [normrnd(0, sigma_head); normrnd(0,sigma_odem,2,1)]];
    while noise_odem(1, i) > 2*sigma_head || noise_odem(1, i) < -2*sigma_head
        noise_odem(1, i) = normrnd(0, sigma_head, 1, 1);
    end
    while noise_odem(2, i) > 2*sigma_odem || noise_odem(1, i) < -2*sigma_odem
        noise_odem(2, i) = normrnd(0, sigma_odem, 1, 1);
    end
    while noise_odem(3, i) > 2*sigma_odem || noise_odem(3, i) < -2*sigma_odem
        noise_odem(3, i) = normrnd(0, sigma_odem, 1, 1);
    end
    for j = 1:3
        noise_smoo{j, i} = normrnd(0, sigma_smoo, 2, 1);
        while noise_smoo{j, i}(1) > 2*sigma_smoo || noise_smoo{j, i}(1) < -2*sigma_smoo
            noise_smoo{j, i}(1) = normrnd(0, sigma_smoo, 1, 1);
        end
        while noise_smoo{j, i}(2) > 2*sigma_smoo || noise_smoo{j, i}(1) < -2*sigma_smoo
            noise_smoo{j, i}(2) = normrnd(0, sigma_smoo, 1, 1);
        end 
        
        noise_cons{j, i} = normrnd(0, sigma_cons, 2, 1);
        while noise_cons{j, i}(1) > 2*sigma_cons || noise_cons{j, i}(1) < -2*sigma_cons
            noise_cons{j, i}(1) = normrnd(0, sigma_cons, 1, 1);
        end
        while noise_cons{j, i}(2) > 2*sigma_cons || noise_cons{j, i}(1) < -2*sigma_cons
            noise_cons{j, i}(2) = normrnd(0, sigma_cons, 1, 1);
        end 
        
        noise_feat{j, i} = [normrnd(0, sigma_r); normrnd(0, sigma_b)];
        while noise_feat{j, i}(1) > 2*sigma_r || noise_feat{j, i}(1) < -2*sigma_r
            noise_feat{j, i}(1) = normrnd(0, sigma_r, 1, 1);
        end
        while noise_feat{j, i}(2) > 2*sigma_b || noise_feat{j, i}(2) < -2*sigma_b
            noise_feat{j, i}(2) = normrnd(0, sigma_b, 1, 1);
        end 
    end
end
save noise/noise2 noise_odem noise_smoo noise_cons noise_feat