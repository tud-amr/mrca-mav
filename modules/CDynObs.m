classdef CDynObs < handle
    % Common base class for ellipsoid obstacles, used in simulation
    
    properties
        
        %% id, configuration
        id_             =   1;              % index
        cfg_
        
        %% shape, size
        size_           =   zeros(3, 1);
        
        %% state
        pos_real_       =   zeros(3, 1);    % real value
        vel_real_       =   zeros(3, 1);
        
        pos_est_        =   zeros(3, 1);    % estimation
        pos_est_cov_    =   eye(3);
        vel_est_        =   zeros(3, 1);
        vel_est_cov_    =   eye(3);
        
        %% predicted state/path
        dt_             =   0;              % time step
        N_              =   0;              % number of stage
        pred_path_      =   [];             % predicted path
        pred_pathcov_   =   [];             % corresponding covariance
        pred_state_     =   [];
        pred_statecov_  =   [];
        
        %% ROS publisher
        size_pub_
        pred_state_pub_
        pred_path_pub_
        pred_pathcov_pub_     
        
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CDynObs(obsID, cfg)
            % constructor
            
            global model
            
            obj.id_     =   obsID;
            obj.cfg_    =   cfg;
            
            obj.dt_     =   model.dt;
            obj.N_      =   model.N;
            
            obj.size_            =   cfg.obs.size;
            
            obj.pred_path_       =   zeros(3, obj.N_);
            obj.pred_pathcov_    =   zeros(6, obj.N_);
            obj.pred_state_      =   zeros(6, obj.N_);
            obj.pred_statecov_   =   zeros(12, obj.N_);
            
        end
        
        
        %%  =======================================================================
        function initializeROS(obj)
            % Initialize ROS publishers
            
            % publisher for predicted path
            obj.pred_path_pub_ = rospublisher(...
                ['/Target', num2str(obj.id_), '/path_prediction'], ...
                'std_msgs/Float64MultiArray', 'IsLatching', false);
            
            % publisher for obs size
            obj.size_pub_ = rospublisher(...
                ['/Target', num2str(obj.id_), '/size'], ...
                'std_msgs/Float64MultiArray', 'IsLatching', false);
            
            % TODO
            
        end
        
        
        %%  =======================================================================
        function initializeState(obj, pos, vel, pos_cov, vel_cov)
            % Initilize obs state
            
            obj.pos_real_    =   pos;
            obj.vel_real_    =   vel;
            obj.pos_est_cov_ =   pos_cov;
            obj.vel_est_cov_ =   vel_cov;
            
        end
        
        
        %%  =======================================================================
        function randomState(obj)
            % random generate obs state
%             pos_x = -obj.cfg_.ws(1) + 2*obj.cfg_.ws(1)*rand(1);
%             pos_y = -0.5*obj.cfg_.ws(2) + 1*obj.cfg_.ws(2)*rand(1);
%             pos_z = 0.6 + 0.6*rand(1);
            pos_x_1 = -obj.cfg_.ws(1) + 1*rand(1);
            pos_x_2 =  obj.cfg_.ws(1) - 1*rand(1);
            pos_y_1 = -obj.cfg_.ws(2) + 0.6*rand(1);
            pos_y_2 =  obj.cfg_.ws(2) - 0.6*rand(1);
            pos_z = 0.8 + 0.4*rand(1);
            
            rx = rand(1);
            if rx <= 0.5
                pos_x = pos_x_1;
            else
                pos_x = pos_x_2;
            end
            
            ry = rand(1);
            if ry <= 0.5
                pos_y = pos_y_1;
            else
                pos_y = pos_y_2;
            end            
            
            speed = 0.8 + 0.4*rand(1);
            angle = -pi + 2*pi*rand(1);
            vel_x = 1.5*speed * cos(angle);
            vel_y = 0.5*speed * sin(angle);
            vel_z = -0.04 + 0.08*rand(1);
            
            obj.pos_real_    =   [pos_x; pos_y; pos_z];
            obj.vel_real_    =   [vel_x; vel_y; vel_z];
            
        end
        
        
        %%  =======================================================================
        function randomSize(obj)
            % random shape the obstacle size
            size_min = [0.2; 0.2; 0.8];
            size_max = [0.6; 0.6; 1.2];
            
            for i = 1 : 3        
                obj.size_(i) = size_min(i) + (size_max(i)-size_min(i))*rand;
            end
            
        end
        
        
        %%  =======================================================================
        function getEstimatedObsState(obj)
            
            obj.pos_est_ = obj.pos_real_;
            obj.vel_est_ = obj.vel_real_;
            
            if obj.cfg_.addObsStateNoise == 1
                dpos = zeros(3, 1);
                dvel = zeros(3, 1);
                for i = 1 : 3
                    dpos(i) = random('Normal', 0, ...
                        sqrt(obj.pos_est_cov_(i,i)));
                    dvel(i) = random('Normal', 0, ...
                        sqrt(obj.vel_est_cov_(i,i)));
                end
                obj.pos_est_    =   obj.pos_est_ + dpos;
                obj.vel_est_    =   obj.vel_est_ + dvel;
            end
            
        end
        
        
        %%  =======================================================================
        function predictPathConstantV(obj)
            % perform path prediction
            
            obj.pred_path_(:, 1) = obj.pos_est_;
            obj.pred_pathcov_(:, 1) = [ obj.pos_est_cov_(1,1); ...
                                        obj.pos_est_cov_(2,2); ...
                                        obj.pos_est_cov_(3,3); ...
                                        obj.pos_est_cov_(1,2); ...
                                        obj.pos_est_cov_(2,3); ...
                                        obj.pos_est_cov_(1,3)];
            xpred = [obj.pos_est_; obj.vel_est_];
            Ppred = [obj.pos_est_cov_, zeros(3, 3); ...
                     zeros(3, 3), obj.vel_est_cov_];
            F = [1, 0, 0, obj.dt_, 0, 0; ...
                 0, 1, 0, 0, obj.dt_, 0; ...
                 0, 0, 1, 0, 0, obj.dt_; ...
                 0, 0, 0, 1, 0, 0; ...
                 0, 0, 0, 0, 1, 0; ...
                 0, 0, 0, 0, 0, 1];
            for iStage = 2 : obj.N_
                xpred = F * xpred;
                Ppred = F * Ppred * F';
                obj.pred_path_(:, iStage) = xpred(1:3);
                obj.pred_pathcov_(:, iStage) = [ Ppred(1,1); ...
                                                 Ppred(2,2); ...
                                                 Ppred(3,3); ...
                                                 Ppred(1,2); ...
                                                 Ppred(2,3); ...
                                                 Ppred(1,3)];
            end
            
        end
        
        
        %%  =======================================================================
        function sendPath(obj)
            % publish the path
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.pred_path_, [], 1);
            obj.pred_path_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function sendSize(obj)
            % publish the path
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.size_, [], 1);
            obj.size_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function step(obj, dt)
            % simulate one step
            
            obj.pos_real_ = obj.pos_real_ + dt*obj.vel_real_;  
            obj.vel_real_ = obj.vel_real_;
            
        end
        
        
        
        
    end
    
    
    
end