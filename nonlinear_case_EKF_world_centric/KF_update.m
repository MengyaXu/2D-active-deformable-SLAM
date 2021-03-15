function [x,P]=KF_update(x_pre,K,m_cons,z,H,P_pre,i,constraint_edge, z_est)
Z=z;
% for i=1:1:size(z,2)
%     if isempty(z{frame_i,i})==1
%     else
%         Z=[Z;z{frame_i,i}];
%     end
% end
for j=1:1:size(constraint_edge,1)
    Z=[Z;m_cons{i,j}];
end
% x=x_pre+K*(Z-H*x_pre);
ze = H*x_pre;
ze(1:length(z_est)) = z_est;
x=x_pre+K*(Z-ze);
% Z-H*x_pre
P=(eye(size(P_pre,1))-K*H)*P_pre;
end