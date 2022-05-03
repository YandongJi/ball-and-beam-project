classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        extra_dummy1 = 0;
        extra_dummy2 = 0;

        alpha1 = 1.0;
        alpha2 = 0.1;
    end
    methods(Access = protected)
        function V_servo = stepImpl(obj, t, lglf3h, lf4h,p_ball,v_ball)
        % This is the main function called every iteration. You have to implement
        % the controller in this function, bu you are not allowed to
        % change the signature of this function. 
        % Input arguments:
        %   t: current time
        %   p_ball: position of the ball provided by the ball position sensor (m)
        %
        %   theta: servo motor angle provided by the encoder of the motor (rad)
        % Output:
        %   V_servo: voltage to the servo input.        
            %% Sample Controller: Simple Proportional Controller
            t
            t_prev = obj.t_prev;
            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            % Decide desired servo angle based on simple proportional feedback.
            k_p = 3;
            theta_d = - k_p * (p_ball - p_ball_ref);

            % Make sure that the desired servo angle does not exceed the physical
            % limit. This part of code is not necessary but highly recommended
            % because it addresses the actual physical limit of the servo motor.
            theta_saturation = 56 * pi / 180;    
            theta_d = min(theta_d, theta_saturation);
            theta_d = max(theta_d, -theta_saturation);

            % Simple position control to control servo angle to the desired
            % position.
%             p_ball_ref
%             p_ball
%             v_ball
%             v_ball_ref
%             1/lglf3h
%             lf4h
            V_servo = 1/lglf3h * (-lf4h + obj.alpha1*(p_ball-p_ball_ref)+obj.alpha2*(v_ball - v_ball_ref));
            V_servo
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.theta_d = theta_d;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t,  lglf3h, lf4h, p_ball, v_ball)        
            V_servo = stepImpl(obj, t,  lglf3h, lf4h, p_ball, v_ball);
            theta_d = obj.theta_d;
        end
    end
    
end
