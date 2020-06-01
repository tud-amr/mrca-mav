%% initialization script
% common pre-run set up for simulation and experiment

%% Setup path and ROS 
setPath;                % include current path
setROS;                 % rosinit

%% Application and if get new Forces solver
application  = 'basic';
fprintf('[%s] Application: %s \n',datestr(now,'HH:MM:SS'), application);
getNewSolver = 0;
quad_ID = [4, 3, 2, 1, 5:100];  % ID of the real drones used for experiments

%% Load problem setup
nQuad   = 4;            % number of quadrotors
nDynObs = 2;            % number of moving obstacles
% three global variables are defined: pr, model, index
if strcmp(application, 'basic')
    basic_setup;        % basic multi-mav collision avoidance, slack is used
else
    error('No application is spercified!');
end


%% Load initial scenario
% including quad initial positions and goal
[quadStartPos, quadStartVel, quadEndPos] = scn_circle(model.nQuad, 3.2);
% [quadStartPos, quadStartVel, quadEndPos] = scn_random(model.nQuad, [-3.2, 3.2], [-3.2, 3.2], [0.6, 2.1]);

%% Runnning configuration, changble when running, should not be global variables
% running mode
cfg.application         =   application;
cfg.modeSim             =   1;              % 0 - experiment
                                            % 1 - simple simulation mode
cfg.modeCoor            =   0;              % -1 - sequential prioritized planning
                                            % 0 - sequential planning (centralized)
                                            % 1 - with path communication (distributed)
                                            % 2 - path prediction based on constant v (decentralized)

% environment boundary, [xmax, ymax, zmax]
% cfg.ws                  = [3.1; 1.4; 2.5];  % DCSC Lab, m
cfg.ws                  =   [4.8; 4.8; 2.4];% simulation, m

% quad goal
cfg.quad.goal           = quadEndPos;

% drone size, collision avoidance paramters
cfg.quad.size           = [0.3; 0.3; 0.4];  % [a, b, c], m
cfg.quad.coll           = [10; 1.2; 0.03];  % lambda, buffer, delta

% stage weights
wS.wp                   = 0.0;              % w_wp
wS.input                = 0.1;              % w_input
wS.coll                 = 1.0;              % w_coll
wS.slack                = 1E4;              % w_slack
cfg.weightStage         = [wS.wp; wS.input; wS.coll; wS.slack];

% terminal weights
wN.wp                   = 8;
wN.input                = 0.0;
wN.coll                 = 1.2;
wN.slack                = 1E4;
cfg.weightN             = [wN.wp; wN.input; wN.coll; wN.slack];

% moving obstalce
cfg.obs.size            = [0.4; 0.4; 0.9];  % [a, b, c], m
cfg.obs.coll            = [10; 1.4; 0.03];  % lambda, buffer, delta

% communication with gui
cfg.setParaGui          = 1;
cfg.ifShowQuadHead      = 1;
cfg.ifShowQuadSize      = 1;
cfg.ifShowQuadGoal      = 0;
cfg.ifShowQuadPath      = 1;
cfg.ifShowQuadCov       = 0;
cfg.ifShowQuadPathCov   = 0;
cfg.quadPathCovShowNum  = 5;

% noise simulation
cfg.quad.Mahalanobis    = sqrt(chi2inv(1-cfg.quad.coll(3),3));
cfg.obs.Mahalanobis     = sqrt(chi2inv(1-cfg.obs.coll(3),3));
% quad
cfg.addQuadStateNoise   = 0;
if strcmp(cfg.application, 'chance') || strcmp(cfg.application, 'chance_slack')
    cfg.addQuadStateNoise = 1;
end
cfg.quad.noise.pos      = diag([0.06, 0.06, 0.06].^2);
cfg.quad.noise.vel      = diag([0.01, 0.01, 0.01].^2);
cfg.quad.noise.euler    = diag(deg2rad([0.5, 0.5, 0.0]).^2);
% obs
cfg.addObsStateNoise    = 1;
cfg.obs.noise.pos       = 0*diag([0.04, 0.04, 0.04].^2);
cfg.obs.noise.vel       = 0*diag([0.01, 0.01, 0.01].^2);
