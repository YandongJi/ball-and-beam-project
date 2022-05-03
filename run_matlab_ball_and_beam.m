close all
clear all

%% General Settings.
% Initial state.
x0 = [-0.19; 0.1; 0.1; 0.1];
t0 = 0;
% Simulation time.
T = 10;
% Sampling time of the controller
dt = 0.01;
% ode function to use.
ode_func = @ode45;
% print log for each timestep if true.
verbose = false;
% plot animation if true.
plot_animation = true;
% save animation to video if true.
save_video = false;

controller_handle = studentControllerInterface();
observer_handle = observer();
u_saturation = 10;

% Initialize traces.
xs = x0;
ts = t0;
us = [];
theta_ds = [];
[p_ball_ref, v_ball_ref] = get_ref_traj(t0);
ref_ps = p_ball_ref;
ref_vs = v_ball_ref;

% Initialize state & time.
x = x0;
t = t0;
end_simulation = false;
u=0.0;

% dynamics definition:
g = 9.81;
r_arm = 0.0254;
L = 0.4255;
a = 5 * g * r_arm / (7 * L);
b = (5 * L / 14) * (r_arm / L)^2;
c = (5 / 7) * (r_arm / L)^2;
K = 10;
tau = 0.1;

% system dynamics
% syms x1 x2 x3 x4 z1 z2;
% h=x1+x3;
% f = [x2;
%     a * sin(x3) - b * x4^2 * cos(x3)^2 + c * x1 * x4^2 * cos(x3)^2;
%     x4;
%     -x4/tau];
% g = [0.0;
%     0.0;
%     0.0;
%     K/tau];
% 
% lfh = [diff(h,x1); diff(h,x2); diff(h,x3); diff(h,x4)]' * f;
% lf2h = [diff(lfh,x1); diff(lfh,x2); diff(lfh,x3); diff(lfh,x4)]' * f;
% lf3h = [diff(lf2h,x1); diff(lf2h,x2); diff(lf2h,x3); diff(lf2h,x4)]' * f;
% lf4h = [diff(lf3h,x1); diff(lf3h,x2); diff(lf3h,x3); diff(lf3h,x4)]' * f;
% lglf3h = [diff(lf3h,x1); diff(lf3h,x2); diff(lf3h,x3); diff(lf3h,x4)]' * g;
% 
% lgh = [diff(h,x1); diff(h,x2); diff(h,x3); diff(h,x4)]' * g
% lglfh = [diff(lfh,x1); diff(lfh,x2); diff(lfh,x3); diff(lfh,x4)]' * g
% lglf2h = [diff(lf2h,x1); diff(lf2h,x2); diff(lf2h,x3); diff(lf2h,x4)]' * g
% lglf3h = [diff(lf3h,x1); diff(lf3h,x2); diff(lf3h,x3); diff(lf3h,x4)]' * g

syms x1 x2 x3 x4 z1 z2;
h=x1+L*sin(x3);
f = [x2;
    a * sin(x3) - b * x4^2 * cos(x3)^2 + c * x1 * x4^2 * cos(x3)^2;
    x4;
    -x4/tau + K/tau*z1;
    z2;
    0.0];
g = [0.0;
    0.0;
    0.0;
    0.0;
    0.0
    1.0];

lfh = [diff(h,x1); diff(h,x2); diff(h,x3); diff(h,x4); diff(h,z1); diff(h,z2)]' * f;
lf2h = [diff(lfh,x1); diff(lfh,x2); diff(lfh,x3); diff(lfh,x4); diff(lfh,z1); diff(lfh,z2)]' * f;
lf3h = [diff(lf2h,x1); diff(lf2h,x2); diff(lf2h,x3); diff(lf2h,x4); diff(lf2h,z1); diff(lf2h,z2)]' * f;
lf4h = [diff(lf3h,x1); diff(lf3h,x2); diff(lf3h,x3); diff(lf3h,x4); diff(lf3h,z1); diff(lf3h,z2)]' * f;
% lglf3h = [diff(lf3h,x1); diff(lf3h,x2); diff(lf3h,x3); diff(lf3h,x4); diff(lf3h,z1); diff(lf3h,z2)]' * g;

lgh = [diff(h,x1); diff(h,x2); diff(h,x3); diff(h,x4); diff(h,z1); diff(h,z2)]' * g;
lglfh = [diff(lfh,x1); diff(lfh,x2); diff(lfh,x3); diff(lfh,x4); diff(lfh,z1); diff(lfh,z2)]' * g;
lglf2h = [diff(lf2h,x1); diff(lf2h,x2); diff(lf2h,x3); diff(lf2h,x4); diff(lf2h,z1); diff(lf2h,z2)]' * g;
lglf3h = [diff(lf3h,x1); diff(lf3h,x2); diff(lf3h,x3); diff(lf3h,x4); diff(lf3h,z1); diff(lf3h,z2)]' * g;


% observer dynamics
q=[x1+x2*dt;
    x2+(a * sin(x3) - b * x4^2 * cos(x3)^2 + c * x1 * x4^2 * cos(x3)^2)*dt;
    x3 + x4 * dt;
    x4+(-x4/tau+K/tau*z1)*dt
    ];
A_sym = [diff(q,x1),diff(q,x2),diff(q,x3),diff(q,x4)];
Pm=[0.5,0.0,0.0,0.0;
    0.0,0.5,0.0,0.0;
    0.0,0.0,0.5,0.0;
    0.0,0.0,0.0,0.5];
%% Run simulation.
% _t indicates variables for the current loop.
tstart = tic;
last_t = tstart;
% dt=0.1;
while ~end_simulation
    %% Determine control input.
    tstart = tic; % DEBUG 
    % Update observer   
    [obs_pos, obs_vel, obs_theta, obs_dtheta, Pm] = observer_handle.process_meas(q,A_sym,Pm,u, x(1),x(3));

    % Controller step
%     dt = tstart - last_t;
%     dt
    lglf3h_subs = vpa(subs(lglf3h,{x1,x2,x3,x4,z1,z2},{obs_pos, obs_vel, obs_theta, obs_dtheta,u*dt,u}));
    lf4h_subs = vpa(subs(lf4h,{x1,x2,x3,x4,z1,z2},{obs_pos, obs_vel, obs_theta, obs_dtheta,u*dt,u}));
    [u, theta_d] = controller_handle.stepController(t, lglf3h_subs, lf4h_subs, obs_pos, obs_vel);
    u = min(u, u_saturation);
    u = max(u, -u_saturation);
    if verbose
        print_log(t, x, u);    
    end
    tend = toc(tstart);    
    us = [us, u];          
    theta_ds = [theta_ds, theta_d];
    %% Run simulation for one time step.
    t_end_t = min(t + dt, t0+T);
    ode_opt = odeset('Events', @event_ball_out_of_range);
%     x
%     t
%     u
%     t_end_t
%     ode_opt
    [ts_t, xs_t, t_event] = ode_func( ...
        @(t, x) ball_and_beam_dynamics(t, x, u), ...
        [t, t_end_t], x, ode_opt);
    end_simulation = abs(ts_t(end) - (t0 + T))<1e-10 || ~isempty(t_event);
    end_with_event = ~isempty(t_event); 
    t = ts_t(end);
    x = xs_t(end, :)';
    %% Record traces.
    last_t=t;
    xs = [xs, x];
    ts = [ts, t];
    [p_ball_ref, v_ball_ref] = get_ref_traj(t);
    ref_ps = [ref_ps, p_ball_ref];
    ref_vs = [ref_vs, v_ball_ref];    
end % end of the main while loop
%% Add control input for the final timestep.
lglf3h_subs = vpa(subs(lglf3h,{x1,x2,x3,x4,z1},{obs_pos, obs_vel, obs_theta, obs_dtheta,u}));
lf4h_subs = vpa(subs(lf4h,{x1,x2,x3,x4,z1},{obs_pos, obs_vel, obs_theta, obs_dtheta,u}));
[u, theta_d] = controller_handle.stepController(t, lglf3h_subs, lf4h_subs, obs_pos, obs_vel);

u = min(u, u_saturation);
u = max(u, -u_saturation);
us = [us, u];
theta_ds = [theta_ds, theta_d];
if verbose
    print_log(t, x, u);    
end
ps = xs(1, :);
thetas = xs(3, :);

% Evaluate the score of the controller.
score = get_controller_score(ts, ps, thetas, ref_ps, us);

%% Plots
% Plot states.
plot_states(ts, xs, ref_ps, ref_vs, theta_ds);
% Plot output errors.
plot_tracking_errors(ts, ps, ref_ps);        
% Plot control input history.
plot_controls(ts, us);

if plot_animation
    animate_ball_and_beam(ts, ps, thetas, ref_ps, save_video);
end

function print_log(t, x, u)
        fprintf('t: %.3f, \t x: ', t);
        fprintf('%.2g, ', x);
        fprintf('\t u: ');
        fprintf('%.2g, ', u);
        % Add custom log here.
        fprintf('\n');
end