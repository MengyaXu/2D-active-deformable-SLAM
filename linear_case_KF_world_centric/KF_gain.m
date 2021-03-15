function [K,H]=KF_gain(PX,V_angle,N,visble,constraint_edge,Num_feature)
rows=[];
columns=[];
values=[];
k=0;
for j=1:1:Num_feature
    if visble(j)==1
        k=k+1;
        rows=[rows,2*k-1,2*k-1,2*k,2*k,2*k-1,2*k-1,2*k,2*k];
        columns=[columns,1,2,1,2,2+2*j-1,2+2*j,2+2*j-1,2+2*j];
        values=[values,-cos(V_angle),sin(V_angle),-sin(V_angle),-cos(V_angle),...
            cos(V_angle),-sin(V_angle),sin(V_angle),cos(V_angle)];
    end
end
for j=1:1:size(constraint_edge,1)
    in=constraint_edge(j,1);
    out=constraint_edge(j,2);
    rows=[rows,2*k+2*j-1,2*k+2*j-1,2*k+2*j,2*k+2*j,2*k+2*j-1,2*k+2*j-1,2*k+2*j,2*k+2*j];
    columns=[columns,2+2*in-1,2+2*in,2+2*in-1,2+2*in,...
        2+2*out-1,2+2*out,2+2*out-1,2+2*out];
    values=[values,-1,0,0,-1,1,0,0,1];
end
H=sparse(rows,columns,values,2*k+size(constraint_edge,1)*2,size(PX,1));
K=PX*H'/(H*PX*H'+N);
end

