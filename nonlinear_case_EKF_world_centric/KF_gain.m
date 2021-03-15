function [K,H]=KF_gain(P_pre, N, visble, constraint_edge, Num_feature, H_feat, i)
rows=[];
columns=[];
values=[];
k=0;

for j=1:1:Num_feature
    if visble(j)==1
        k=k+1;
        rows=[rows,2*k-1,2*k-1,2*k-1,2*k-1,2*k-1,2*k,2*k,2*k,2*k,2*k];
        columns=[columns,1,2,3,3+2*j-1,3+2*j,1,2,3,3+2*j-1,3+2*j];
        values=[values, H_feat{i, j}(1,:), H_feat{i, j}(2,:)];
    end
end
for j=1:1:size(constraint_edge,1)
    in=constraint_edge(j,1);
    out=constraint_edge(j,2);
    rows=[rows,2*k+2*j-1,2*k+2*j-1,2*k+2*j,2*k+2*j,2*k+2*j-1,2*k+2*j-1,2*k+2*j,2*k+2*j];
    columns=[columns,3+2*in-1,3+2*in,3+2*in-1,3+2*in,...
        3+2*out-1,3+2*out,3+2*out-1,3+2*out];
    values=[values,-1,0,0,-1,1,0,0,1];
end
H=sparse(rows,columns,values,2*k+size(constraint_edge,1)*2,size(P_pre,1));
K=P_pre*H'/(H*P_pre*H'+N);
end

