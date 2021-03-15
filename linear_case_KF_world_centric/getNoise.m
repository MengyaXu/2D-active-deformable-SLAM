sigma_odem = 0.02;
sigma_smoo = 0.3;
sigma_cons = 0.2;
sigma_feat = 0.05;

save sigma sigma_odem sigma_smoo sigma_cons sigma_feat
noise_odem = [];
% noise_smoo = [];
% noise_cons = [];
% noise_feat = [];
for i = 1:100
    noise_odem = [noise_odem, normrnd(0,sigma_odem,2,1)];
    while noise_odem(1, i) > 2*sigma_odem || noise_odem(1, i) < -2*sigma_odem
        noise_odem(1, i) = normrnd(0, sigma_odem, 1, 1);
    end
    while noise_odem(2, i) > 2*sigma_odem || noise_odem(1, i) < -2*sigma_odem
        noise_odem(2, i) = normrnd(0, sigma_odem, 1, 1);
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
        
        noise_feat{j, i} = normrnd(0, sigma_feat, 2, 1);
        while noise_feat{j, i}(1) > 2*sigma_feat || noise_feat{j, i}(1) < -2*sigma_feat
            noise_feat{j, i}(1) = normrnd(0, sigma_feat, 1, 1);
        end
        while noise_feat{j, i}(2) > 2*sigma_feat || noise_feat{j, i}(1) < -2*sigma_feat
            noise_feat{j, i}(2) = normrnd(0, sigma_feat, 1, 1);
        end 
    end
end
save noise/noise noise_odem noise_smoo noise_cons noise_feat