classdef observer 
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        prev_x1 = -0.19;
        prev_x2 = 0.1;
        prev_x3 = 0.1;
        prev_x4 = 0.1;
    end

    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [obs_pos, obs_vel, obs_theta, obs_dtheta, Pm] = process_meas(obj, q, A_sym, Pm, u_prev, p_ball, theta)      
            prev_x1=obj.prev_x1;
            prev_x2=obj.prev_x2;
            prev_x3=obj.prev_x3;
            prev_x4=obj.prev_x4;
            syms x1 x2 x3 x4 z1;
            xp = vpa(subs(q,{x1,x2,x3,x4,z1},{prev_x1, prev_x2, prev_x3, prev_x4,u_prev}));
            A = vpa(subs(A_sym, {x1,x2,x3,x4,z1},{prev_x1, prev_x2, prev_x3, prev_x4,u_prev}));
            L=eye(4);
            var_v = diag([0.1,0.1,0.1,0.1]);
            Pp=A*Pm*A' + L*var_v*L';

            H=[1.0,0.0,0.0,0.0;
                0.0,1.0,0.0,0.0];
            M=eye(2);
            var_w = diag([0.1,0.1]);
            K = Pp*H'*inv(H*Pp*H' + M*var_w*M');
            z=[p_ball;
                theta];
            h=[xp(1);
                xp(3)];
            xm=xp+K*(z-h);
            Pm=(eye(4) - K*H)*Pp;

            obj.prev_x1 = xm(1);
            obj.prev_x2 = xm(2);
            obj.prev_x3 = xm(3);
            obj.prev_x4 = xm(4);
            obs_pos = xm(1);
            obs_vel = xm(2);
            obs_theta = xm(3);
            obs_dtheta = xm(4);
            % obj.Pm=Pm;
        end
    end
    
end
