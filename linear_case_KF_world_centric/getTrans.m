function translation = getTrans(op, d, Num_pose)
if  op == 1
    translation = repmat([d; 0], [1, Num_pose]);
elseif op == 2
    dtheta = pi/10;
    translation = zeros(2, Num_pose);
    for i = 1:Num_pose
        translation(:, i) = [d*cos(dtheta/2 + (i-1)*dtheta); d*sin(dtheta/2 + (i-1)*dtheta)];
    end
end
end

