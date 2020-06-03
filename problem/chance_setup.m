% Script of problem setup for chance-constrained mav collision avoidance
% 
% Used for generating FORCES PRO solvers and control node of the mav
% 
% Define system physical parameters, problem dimensions and indexing here
% 
% (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
% 

%% System physical parameters, not changable when running
global pr                           % the physical parameters are used globally
% speed limit, hard constraint
pr.state.maxVx      = 2.0;          % m/s
pr.state.maxVy      = 2.0;          % m/s
pr.state.maxVz      = 1.0;          % m/s
% control input 
pr.input.maxRoll    = deg2rad(12);  % rad, maximum roll, phi
pr.input.maxPitch   = deg2rad(12);  % rad, maximum pitch, theta
pr.input.maxVz      = 1.0;          % m/s, maximum vertical speed
pr.input.maxYawRate = deg2rad(90);  % rad/s

%% Problem dimensions, not changable when running
global model                        % the model setup is accessed globally
model.nDynObs       =   nDynObs;    % number of moving obstacles
model.nQuad         =   nQuad;      % number of mavs in the problem
model.nObs          =   model.nDynObs + (model.nQuad - 1);
                                    % number of other agents (including 
                                    % moving obstacles and other mavs) 
                                    % as obstalces
model.nParamPerObs  =   15  ;       % number of parameters each other agent
                                    % 3 + 3 + 3 + 6 (pos, size, coll, cov)
model.N             =   20  ;       % horizon length
model.dt            =   0.05;       % time step
model.nvar          =   14  ;       % number of stage variables (z)
model.neq           =   9   ;       % number of equality constraints (x)
model.nh            =   3 + model.nObs; % number of inequality constraints
model.nin           =   4   ;       % number of control inputs (u)
model.nslack        =   1   ;       % number of slacks (s)
model.npar          =   24 + model.nObs*model.nParamPerObs;
                                    % number of runtime parameters on each
                                    % stage

%% Indexing, not changable when running
global index                        % the index is used globally
% in stage vector, each stage
index.z.all         =   1:model.nvar;
index.z.inputs      =   1:4 ;       % control input, [phi_c, theta_c, vz_c, psi_rate_c]
index.z.slack       =   5   ;       % slack, [slack_env, slack_coll]
index.z.pos         =   6:8 ;       % position, [x, y, z]
index.z.vel         =   9:11;       % velocity, [vx, vy, vz]
index.z.euler       =   12:14;      % euler angles, [phi, theta, psi]
% in state vector, each stage
index.x.all         =   1:model.neq;
index.x.pos         =   1:3 ;
index.x.vel         =   4:6 ;
index.x.euler       =   7:9 ;
% in parameter vector, problem, each stage
index.p.all         =   1:model.npar;
index.p.envDim      =   1:3 ;       % [xdim, ydim, zdim]
index.p.startPos    =   4:7 ;       % [x0, y0, z0, psi0]
index.p.wayPoint    =   8:11;       % [xg, yg, zg, psig]
index.p.size        =   12:14;      % [a0, b0, c0]
index.p.posCov      =   15:20;      % position uncertainty covariance, 6 independent
index.p.weights     =   21:24;      % [w_wp, w_input, w_coll, w_slack]
% in parameter vector, obstacle, each stage
if model.nObs >= 1                  % parameter for obstalce
    idxBegin = index.p.weights(end) + 1;
    index.p.obsParam = reshape(idxBegin : ((idxBegin-1)+model.nParamPerObs*model.nObs),...
        [model.nParamPerObs,model.nObs]);
    % index inside for each moving obstacle    
    index.p.obs.pos     =   1:3 ;       % [x1, y1, z1]
    index.p.obs.size    =   4:6 ;       % [a1, b1, c1]
    index.p.obs.coll    =   7:9 ;       % [lambda1, buffer1, delta1]
    index.p.obs.posCov  =   10:15;      % position uncertainty covariance
end
