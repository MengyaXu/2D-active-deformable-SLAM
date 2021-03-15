function v = getV(op)
% save data V_angle Q PX sigma_odem X_update measurement_smooth i
% v0 = 0;
% [v,fmin] = fmincon('objFun', v0, [],[],[],[],0,10);
load data d i op_trans
if op == 1
    if op_trans == 1
        v = [d; 0];
    elseif op_trans == 2
        dtheta = pi/10;
        v = [d*cos(dtheta/2 + (i-1)*dtheta); d*sin(dtheta/2 + (i-1)*dtheta)];
    end
elseif op == 2 || op == 3 || op == 4
    min = Inf;
    vx = [-d:d/5:d]; 
    vy = [-d:d/5:d];
    if op_trans == 1
        for i = 1:length(vx)
            v0 = [vx(i); 0];
            obj = objFun(v0);
            if obj <= min
                min = obj;
                v = v0;
            end
        end
    elseif op_trans == 2
        for i = 1:length(vx)
            for j = 1:length(vy)
                v0 = [vx(i); vy(j)];
                obj = objFun(v0);
                if obj <= min
                    min = obj;
                    v = v0;
                end
            end
        end
    end
elseif op == 0
    load V v
    v0 = v;
    v = v0(i-1);
end

