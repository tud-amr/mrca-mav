%% one-horizon collision avoidance using chance-constrained MPC
clear model
clear all
clc

% The Forces Pro software with a valid license is required for running this
% code!


%% problem set up
p0 = [0; 0];                    % initial position of the robot
pg = [10; 0];                   % goal location of the robot
q0 = [5; -0.01];                % initial position of the obstacle center
Sigma_q = diag([0.4, 0.1]);     % position uncertainty of the obstacle 
r_box = [1; 0.5];               % size of the box, half length and half width
r_obs = sqrt(2)*r_box;          % [a;b] of the enclosed ellipse
Omega_obs = diag([1/r_obs(1)^2, 1/r_obs(2)^2]); % Omega of the obstable
T = 8;                          % horizon length, s
N = 40;                         % number of stages
dt = T/N;                       % time step
alpha = 0.01;                   % collision probabilisty threshold
z_alpha = erfinv(1-2.0*alpha);  % \erf^{-1}(1-2*alpha)


%% MPC setup
global model
% global information
model.pg        = pg;
model.q0        = q0;
model.r_obs     = r_obs;
model.Sigma_q   = Sigma_q;
model.z_alpha   = z_alpha;
% problem dimension
model.N     =   N;              % horizon length
model.dt    =   dt;             % time step
model.nvar  =   9;              % number of variables in z vector, including
                                % [u_x, u_y, u_yaw, px, py, vx, vy, yaw, yaw_rate]
model.neq   =   6;              % number of equality contraints (x)
model.nh    =   1;              % number of inequality constraints
model.nin   =   3;              % number of control inputs (u)
model.npar  =   2;              % number of paramters

% dynamics, i.e. equality constraints
model.eq = @(z) RK2(z((model.nin + 1) : model.nvar), ...
    z(1 : model.nin), @robot_dynamics, model.dt);
model.E = [zeros(model.neq, model.nin), eye(model.neq)];

% objective functions, penalize stage control input and res to the goal
model.objective = @(z, p) objective_chance(z, p);
    
% upper/lower bound of z vector
%           u_x, u_y, u_yaw, px, py, vx,  vy,  yaw, yaw_rate
model.lb = [-10, -10, -10, -10, -3, -10, -10, -10, -10];
model.ub = [+10, +10, +10, +10, +3, +10, +10, +10, +10];

% collision avoidance inequalities
model.ineq = @(z) nonlinIneq_chance(z);
model.hl = [0];
model.hu = [+Inf];


%% Initial conditions
model.xinit = [p0(1); p0(2); 0; 0; 0; 0];
model.xinitidx = 4:9;           % use this to specify on which variables 
                                % initial conditions are imposed

                                    
%% Define solver options
codeoptions             = getOptions('FORCESNLPsolver');
codeoptions.maxit       = 500; 	% Maximum number of iterations
codeoptions.printlevel  = 2;    % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel    = 0;    % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.overwrite   = 1;
codeoptions.cleanup     = 1;
codeoptions.timing      = 1;                                    
codeoptions.nlp.linear_solver   = 'symm_indefinite_fast'; 
                              	% linear system solver, better for long horizons

                                
%% Generate forces solver
FORCES_NLP(model, codeoptions);
                                

%% Call solver
% Set initial guess to start solver from:
x0i = model.lb+(model.ub-model.lb)/2;
x0  = repmat(x0i', model.N, 1);
problem.x0 = x0; 

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = model.xinit;

% Set parameters
w_pos = 1;
w_input = 2;
problem.all_parameters = repmat([w_pos w_input]',model.N,1);

% Time to solve the NLP!
[output, exitflag, info] = FORCESNLPsolver(problem);

% Make sure the solver has exited properly. 
fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n', ...
    info.it,info.solvetime);


%% Plot results
% main figure
fig_main = figure;              % main figure
hold on;
axis([0 10 -3 3]);
ax_main = fig_main.CurrentAxes;
box on;
grid on;
xlabel('x (m)');
ylabel('y (m)');
% plot static obstacle
fig_box_obs = plot_box_2D(ax_main, q0, 2*r_box, 0, ...
            'FaceColor', [0.4 0.4 0.4], 'FaceAlpha', 0.6, ...
            'EdgeColor', 'k', 'EdgeAlpha', 0.8);
fig_ell_obs = plot_ellipse_2D(ax_main, q0, r_obs, 0, ...
            'FaceColor', 'r', 'FaceAlpha', 0.4, ...
            'EdgeColor', 'r', 'EdgeAlpha', 0.2);
% plot robot trajecoty
mpc_plan = zeros(model.nvar, model.N);
for i = 1:model.N
    mpc_plan(:,i) = output.(['x',sprintf('%02d',i)]);
end
robot_traj = mpc_plan(4:5, :);
fig_robot_traj = plot(ax_main, robot_traj(1, :), robot_traj(2, :), ...
                'Color', 'g', ...
                'Marker', 'o', ... 
                'LineWidth', 1.5, 'LineStyle', '-');

%% objective function
function obj = objective_chance(z, p)

    global model
    u_x = z(1);
    u_y = z(2);
    u_yaw = z(3);
    px = z(4);
    py = z(5);
    w_pos = p(1);
    w_input = p(2);
    gx = model.pg(1);
    gy = model.pg(2);
    obj = w_pos * ((px - gx)^2+(py - gy)^2) + ...
        w_input * (u_x^2 + u_y^2 + u_yaw^2);

end

                             
%% chance-constraint inequality
function ineq = nonlinIneq_chance(z)

    global model                    % global model information  
    z_alpha = model.z_alpha;
    
    ego_pos = z(4:5);               % ego robot pos
    
    % collision avoidance constraints
    obs_pos = model.q0;             % obs pos
    obs_pos_cov = model.Sigma_q;    % obs pos cov
    a = model.r_obs(1);             % obs size
    b = model.r_obs(2);
    Omega_root = [ 1/a, 0; ...
                   0, 1/b];         % transformation matrix
    pos_ro = Omega_root*(ego_pos - obs_pos);
    cov_ro = transpose(Omega_root) * obs_pos_cov * Omega_root;
    pos_ro_norm = sqrt(transpose(pos_ro)*pos_ro);
    c_ro = z_alpha * sqrt(2*transpose(pos_ro)*cov_ro*pos_ro) / pos_ro_norm;
    cons_obs = pos_ro_norm - 1 - c_ro;
    
    % inequality constraints
    ineq = cons_obs;
end


%% robot dynamics
function xdot = robot_dynamics(x, u)
    % x = [px, py, vx, vy, yaw, yaw_rate]
    % u = [u_x, u_y, u_yaw]
    % px_dot = vx*cos(yaw) - vy*sin(yaw)
    % py_dot = vx*sin(yaw) + vy*cos(yaw)
    % vx_dot = -(vx + u_x) / tau_x
    % vy_dot = -(vy + u_y) / tau_y
    % yaw_dot = yaw_rate
    % yaw_rate_dot = -(yaw_rate + u_yaw) / tau_yaw

    tau = 1;
    
    % state
    px = x(1);
    py = x(2);
    vx = x(3);
    vy = x(4);
    yaw = x(5);
    yaw_rate = x(6);
    
    % control
    u_x = u(1);
    u_y = u(2);
    u_yaw = u(3);
    
    % dynamics
    px_dot = vx*cos(yaw) - vy*sin(yaw);
    py_dot = vx*sin(yaw) + vy*cos(yaw);
    vx_dot = -(vx + u_x) / tau;
    vy_dot = -(vy + u_y) / tau;
    yaw_dot = yaw_rate;
    yaw_rate_dot = -(yaw_rate + u_yaw) / tau;
    
    % gradient
    xdot = [px_dot; py_dot; vx_dot; vy_dot; yaw_dot; yaw_rate_dot];
    
end


%% plot a box represented in 2D
function h = plot_box_2D(ax, box_pos, box_size, box_yaw, varargin)
    % plot a box represented in 2D
    % 
    % inputs:
    %   - ax: figure handle
    %   - pt: box center, [dx1]
    %   - size: box size, [dx1]
    %   - yaw: box yaw, [1]
    %   - varargin: patch properties
    % 
    % outputs: 
    %   - h: plot handle
    % 
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    % 
    % convert to verts
    [c, ~] = box2PolyVertsCons_2D(box_pos, box_size, box_yaw);

    if isempty(c)
      return
    end
    
    k = convhull(c(1,:), c(2,:));
    X = reshape(c(1,k'), size(k'));
    Y = reshape(c(2,k'), size(k'));
    h = patch(ax, X,Y, 'k', varargin{:});
    
end


%% plot an ellipse with orientation and return the plot handle
function h = plot_ellipse_2D(ax, pos, ell, orient, varargin)
    % plot an ellipse with orientation and return the plot handle
    % 
    % inputs:
    %   - ax: figure handle
    %   - pos: ellipse center, [2x1]
    %   - ell: ellipse size, [2x1]
    %   - orient: ellipse orientation, 1
    %   - varargin: patch properties
    % 
    % outputs: 
    %   - h: plot handle
    % 
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    % 
    
    % generate plot data
    a = ell(1);
    b = ell(2);
    r = 0: 0.1: 2*pi+0.1;
    alpha = [ cos(orient) -sin(orient)
              sin(orient)  cos(orient)];
    p = [(a*cos(r))' (b*sin(r))'] * alpha;
    X = pos(1) + p(:,1);
    Y = pos(2) + p(:,2);

    % plot the ellipse
    h = patch(ax, 'XData', X, 'YData', Y, varargin{:});
    
end


%% transform a box to polytope verts and linear constraints
function [poly_vert, poly_Ab] = box2PolyVertsCons_2D(pos, size, orientation_xy)
    % transform a box to polytope verts and linear constraints
    % 
    % inputs:
    %   - pos: box center position, x and y, [2x1]
    %   - size: box size, length and width, [2x1]
    %   - orientation_xy: box orientation, [1]
    % 
    % outputs: 
    %   - poly_vert: vertice represented the box, 2*m
    %   - poly_Ab: linear constraints represented half planes of the box, 3*m
    % 
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    %
    
    poly_vert = zeros(2, 4);
    poly_Ab   = zeros(3, 4);
    
    %% vertices
    % before rotation and translation
    x_min = - 0.5*size(1);
    x_max = + 0.5*size(1);
    y_min = - 0.5*size(2);
    y_max = + 0.5*size(2);
    p1 = [x_min; y_min];
    p2 = [x_max; y_min];
    p3 = [x_max; y_max];
    p4 = [x_min; y_max];
    % only rotate in xy plane, then translation
    rot_mtx = rotz(rad2deg(orientation_xy));
    p1_r = rot_mtx(1:2, 1:2) * p1 + pos;
    p2_r = rot_mtx(1:2, 1:2) * p2 + pos;
    p3_r = rot_mtx(1:2, 1:2) * p3 + pos;
    p4_r = rot_mtx(1:2, 1:2) * p4 + pos;
    % polytope verts
    poly_vert = [p1_r, p2_r, p3_r, p4_r];

    %% half plane constraints
    % p1-p2 plane
    poly_Ab(1:2, 1) = [p2_r(2)-p1_r(2); -(p2_r(1)-p1_r(1))];
    poly_Ab(3, 1) = poly_Ab(1:2, 1)'*p1_r(1:2);
    % p2-p3 plane
    poly_Ab(1:2, 2) = [p3_r(2)-p2_r(2); -(p3_r(1)-p2_r(1))];
    poly_Ab(3, 2) = poly_Ab(1:2, 2)'*p2_r(1:2);
    % p3-p4 plane
    poly_Ab(1:2, 3) = -poly_Ab(1:2, 1);
    poly_Ab(3, 3) = poly_Ab(3, 1);
    % p4-p1 plane
    poly_Ab(1:2, 4) = -poly_Ab(1:2, 2);
    poly_Ab(3, 4) = poly_Ab(3, 2);

    %% normalization the A matrix
    for i = 1 : 4
        A = poly_Ab(1:2, i);
        poly_Ab(1:2, i) = poly_Ab(1:2, i) / norm(A);
        poly_Ab(3, i) = poly_Ab(3, i) / norm(A);
    end

end
