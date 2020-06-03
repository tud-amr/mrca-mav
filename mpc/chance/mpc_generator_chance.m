% Script for generating FORCES PRO NLP solvers for chance-constrained 
% mav collision avoidance
% 
% An optimization problem is formulated for a mav (Bebop 2) with the
% following nonlinear continuous-time dynamics
% 
%	\dot( x ) == vx
%	\dot( y ) == vy
%	\dot( z ) == vz
%	\dot( vx) == (cos(psi)*tan(theta)/cos(phi) + sin(psi)*tan(phi))*g - kD_x*vx
%	\dot( vy) == (sin(psi)*tan(theta)/cos(phi) - cos(psi)*tan(phi))*g - kD_y*vy
%	\dot( vz) == (k_vz*vz_c - vz) / tau_vz
%	\dot(phi) == (k_phi*phi_c - phi) / tau_phi
%	\dot(theta) == (k_theta*theta_c - theta) / tau_theta
%	\dot(psi) == psi_rate_c
% 
% where [x, y, z] are position, [vx, vy, vz] are velocity, [phi, theta,
% psi] are roll, pitch and yaw angles. The inputs are [phi_c, theta_c, 
% vz_c, psi_rate_c]. Others are model parameters.
% 
% Dynamics model function can be passed by C functions.
% 
% The mav (ellipsoid) is controlled to navigate to a given waypoint (with 
% zero yaw by default) while avoiding other moving objects (ellipsoids) in 
% the environments.
%  
% Collision avoidance constraints (approximated) between two ellipsoids 
% [a, b, c] at p and [a1, b1, c1] at p1:
% 
%	(x-x1)^2/(a+a1)^2 + (y-y1)^2/(b+b1)^2 + (z-z1)^2/(c+c1)^2 >= 1 + c
% 
% where c is an extra buffer computed from agent uncertainty covariance
% 
% Environment boundaries are considered.
% 
% Slack variable is used for environment boundary constraint
% 
% Cost function to be minimized include: way-point navigation, collision
% potential field cost, control input cost, slack cost. 
% 
% Variables are collected stage-wise into
%   z = [phi_c, theta_c, vz_c, psi_rate_c, slack_env, x, y, z, 
%       vx, vy, vz, phi, theta, psi].                       (14)
% where
%   s = [slack_env].                                        (1)
%   u = [phi_c, theta_c, vz_c, psi_rate_c].                 (4)
%   x = [x, y, z, vx, vy, vz, phi, theta, psi].             (9)
% 
% Paramters passing to the problem at each stage include
% 
%   - maximum x, y, z environment dimension [xdim, ydim, zdim]. (3)
%   - ego mav current start position [x0, y0, z0, psi0].    (4)
%   - ego mav waypoint position [xg, yg, zg, psig].         (4)
%   - ego mav size [a0, b0, c0].                            (3)
%   - cost weight [w_wp, w_input, w_coll, w_slack].         (4)
%   - ego mav position covariance.                          (6)
%   - other object position [x1, y1, z1].                   (3)
%   - other object size [a1, b1, c1].                       (3)
%   - other object potential function [lambda1, buffer1].   (2)
%   - othr object collision avoidance chance threshold.     (1)
%   - other object position covariance.                     (6)
%
% (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
% 


%% Dynamics, i.e. equlity constraints
% RK integrator is used to discretize the continuous-time dynamics
model.eq = @(z) RK2(z((model.nin + model.nslack + 1) : model.nvar), ...
    z(1 : model.nin), @bebop_dynamics, model.dt);
% model.continuous_dynamics = bebop_dynamics;
% Indices on LHS of dynamical constraint - for efficiency reasons, make
% sure the matrix E has structure [0 I] where I is the identity matrix
model.E = [zeros(model.neq, model.nin + model.nslack), eye(model.neq)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
model.lb = zeros(1, model.nvar);
model.ub = zeros(1, model.nvar);
% control input bound
model.lb(index.z.inputs) = [-pr.input.maxRoll, -pr.input.maxPitch, ...
                            -pr.input.maxVz, -pr.input.maxYawRate];
model.ub(index.z.inputs) = [ pr.input.maxRoll,  pr.input.maxPitch, ...
                             pr.input.maxVz,  pr.input.maxYawRate];
% slacks should be always larger than zero
model.lb(index.z.slack) = 0   ;
model.ub(index.z.slack) = +inf;
% position is not really hardly constrained (later softly constrained with slacks)
model.lb(index.z.pos) = [-inf, -inf, -inf];
model.ub(index.z.pos) = [+inf, +inf, +inf];
% velocity is hardly constrained
model.lb(index.z.vel) = [-pr.state.maxVx, -pr.state.maxVy, -pr.state.maxVz];
model.ub(index.z.vel) = [ pr.state.maxVx,  pr.state.maxVy,  pr.state.maxVz];
% euler angles are constrained to be within [-pi, pi]
model.lb(index.z.euler) = [-pi, -pi, -pi];
model.ub(index.z.euler) = [ pi,  pi,  pi];

% general nonlinear inequalities hl <= h(x) <=hu
model.ineq = @(z, p) mpc_nonlinIneq_chance(z, p, model.nObs);
% upper/lower bound for inequalities
model.hl = [-1, -1, 0.2,     zeros(1, model.nObs)]';
model.hu = [ 1,  1,   1, +inf*ones(1, model.nObs)]';

%% Objective function
model.objective  = @(z, p) mpc_objective_chance(z, p, model.nObs);
model.objectiveN = @(z, p) mpc_objectiveN_chance(z, p, model.nObs);

%% Initial and final conditions
% here we only need to spercify on which variables initial conditions are imposed
model.xinitidx = [index.z.pos, index.z.vel, index.z.euler];

%% Define solver options
solver_name = strcat('FORCESNLPsolver_chance_', num2str(model.nObs), '_', ...
    num2str(model.N), '_', num2str(1000*model.dt));
codeoptions = getOptions(solver_name);
% codeoptions.platform    = 'Generic';% target platform
codeoptions.maxit       = 1000;     % maximum number of iterations
codeoptions.printlevel  = 0;        % use printlevel = 2 to print progress
                                    % (but not for timings)
codeoptions.optlevel    = 3;        % 0: no optimization, 
                                    % 1: optimize for size, 
                                    % 2: optimize for speed, 
                                    % 3: optimize for size & speed
codeoptions.overwrite   = 1;
codeoptions.cleanup     = 1;
codeoptions.timing      = 1;
codeoptions.parallel    = 1;        % run prediction on multiple cores 
                                    % (better for longer horizons)
codeoptions.threadSafeStorage   = true; % the generated solver can be run
                                        % in parallel on different threads
codeoptions.BuildSimulinkBlock  = 0;% skipping builing of simulink S-function
codeoptions.nlp.linear_solver   = 'symm_indefinite_fast'; 
                                    % linear system solver, better for long horizons
% codeoptions.nlp.checkFunctions  = 0;% not check the output of the function evaluations
codeoptions.noVariableElimination = 1;
codeoptions.nlp.TolStat = 1E-3;     % infinity norm tolerance on stationarity
codeoptions.nlp.TolEq   = 1E-3;     % infinity norm of residual for equalities
codeoptions.nlp.TolIneq = 1E-3;     % infinity norm of residual for inequalities
codeoptions.nlp.TolComp = 1E-3;     % tolerance on complementarity conditions
% define integrator options
% codeoptions.nlp.integrator.type = 'ERK2'; % can also be 'ForwardEuler', 
%                                           % 'ERK2', 'ERK3', 'ERK4', 
%                                           % 'BackwardEuler', or 'IRK2'
% codeoptions.nlp.integrator.Ts   = model.dt;
% codeoptions.nlp.integrator.nodes= 4;
% change this to your server or leave uncommented for using the standard
% embotech server at https://www.embotech.com/codegen
% codeoptions.server = 'http://yourforcesserver.com:8114/v1.5'; 

%% Generate FORCES PRO solver
fprintf('[%s] Generating new FORCES solver...\n',datestr(now,'HH:MM:SS'));
FORCES_NLP(model, codeoptions);
fprintf('[%s] FORCES solver generated OK \n',datestr(now,'HH:MM:SS'));

%% Storing the solver
% create a folder
folder_name = ['./solver/chance', '/Chance_Forces_', num2str(model.nObs), '_', ...
    num2str(model.N), '_', num2str(1000*model.dt)];
mkdir(folder_name);
% delete it and create again to remove all files in the folder
rmdir(folder_name, 's');
mkdir(folder_name);
% move the generated solver to the folder
movefile(solver_name, folder_name);         % move the folder
movefile([solver_name, '*'], folder_name);  % move the files
movefile('*forces', folder_name);
% include path
setPath;
