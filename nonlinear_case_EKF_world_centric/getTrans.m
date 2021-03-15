function translation = getTrans(op, d, dtheta, Num_pose)
if  op == 1
    translation = repmat([d; 0], [1, Num_pose]);
elseif op == 2
    translation = zeros(2, Num_pose);
    for i = 1:Num_pose
        translation(:, i) = [d*cos(dtheta/2 + (i-1)*dtheta); d*sin(dtheta/2 + (i-1)*dtheta)];
%         rr = 5;
%         translation(:, i) = [rr*(sin(i*dtheta)-sin((i-1)*dtheta)); rr*(cos((i-1)*dtheta)-cos(i*dtheta))];
    end
end
end

