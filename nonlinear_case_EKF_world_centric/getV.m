function v = getV(op)
% save data V_angle Q PX sigma_odem X_update measurement_smooth i
% v0 = 0;
% [v,fmin] = fmincon('objFun', v0, [],[],[],[],0,10);
load data d i op_trans dtheta
if op == 1
    if op_trans == 1
        v = [0; d; 0];
    elseif op_trans == 2
        v = [dtheta; d*cos(dtheta); d*sin(dtheta)];
%         rr = 5;
%         v = [dtheta; rr*(sin(i*dtheta)-sin((i-1)*dtheta)); rr*(cos((i-1)*dtheta)-cos(i*dtheta))];
    end
elseif op == 2
    min = Inf;
    vtheta = [-pi/4:pi/10:pi/4];
%     vx = [-2*d:2*d/5:2*d]; 
%     vy = [-2*d:2*d/5:2*d];
%     vx = [-d:d/5:d]; 
%     vy = [-d:d/5:d];
    dd = [-2*d:d/5:2*d];
    if op_trans == 1
        for i = 1:length(dd)
            v0 = [0; dd(i); 0];
            obj = objFun(v0);
            if obj <= min
                min = obj;
                v = v0;
            end
        end
    elseif op_trans == 2
        for i = 1:length(vtheta)
            for j = 1:length(dd)
%                 for k = 1:length(vy)
                    v0 = [vtheta(i); dd(j) * cos(vtheta(i)); dd(j) * sin(vtheta(i))];
                    obj = objFun(v0);
                    if obj <= min
                        min = obj;
                        v = v0;
                    end
%                 end
            end
        end
    end
elseif op == 0
    load V v
    v0 = v;
    v = v0(i-1);
end

