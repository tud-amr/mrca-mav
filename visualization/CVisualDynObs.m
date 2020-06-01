classdef CVisualDynObs < handle
    % CVisualDynObs Handles visualisation of dynamic obstacle
    
    properties
        
        %% ID
        id_             =   0;
        N_              =   0;
        
        %% Flag
        
        %% Color
        color_
        
        %% Physical propertices
        obs_size_       =   [];     % [a, b, c]
        
        %% State and path
        obs_state_      =   [];
        obs_path_       =   [];
        
        %% 3d plot handles
        % identify text
        h3d_text_
        % obs ellipse size
        h3d_size_
        
        %% Top-down/side X view handles
        % size ellipse boundaty
        h2d_td_size_
        h2d_sx_size_
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CVisualDynObs(obsID, N, obs_size, axesHandle, subaxesHandle, color)
            % Constructor
           
            obj.id_         =   obsID;
            obj.N_          =   N;
            obj.obs_size_   =   obs_size;
            obj.color_      =   color;
            
            obj.obs_state_  =   zeros(6, 1);
            obj.obs_path_   =   zeros(3, obj.N_);
            
            %% 3d view objects
            % identify text
            obs_text = char('Obs ' + string(obj.id_));
            obj.h3d_text_ = text(axesHandle, 0, 0, obj.obs_size_(3), ...
                obs_text, 'FontSize', 8);
            % obs ellipse size
            [data_size_x_, data_size_y_, data_size_z_] = ...
                ellipsoid(0, 0, 0, ... 
                obj.obs_size_(1), obj.obs_size_(2), obj.obs_size_(3));
            obj.h3d_size_ = surface(axesHandle, ...
                data_size_x_, data_size_y_, data_size_z_, ...
                'EdgeColor', [0.5 0.5 0.5], ...
                'FaceColor', [0.45 0.94 0.45], ...
                'FaceAlpha', 0.2);
            
            %% side view objects
            % obs size
            obs_size_points = pointsEllipse([0,0], ...
                obj.obs_size_(1), obj.obs_size_(2));
            obj.h2d_td_size_ = plot(subaxesHandle(1), ...
                obs_size_points(1,:), obs_size_points(2,:), '-', ...
                'color', [0.5 0.5 0.5], ...
                'linewidth', 1.5);
            obs_size_points = pointsEllipse([0,0], ...
                obj.obs_size_(1), obj.obs_size_(3));
            obj.h2d_sx_size_ = plot(subaxesHandle(2), ...
                obs_size_points(1,:), obs_size_points(2,:), '-', ...
                'color', [0.5 0.5 0.5], ...
                'linewidth', 1.5);
            
        end
        
        
        %%  =======================================================================
        function setPose(obj)
            % Update visulization of dynamic obstalce
            
            pos = obj.obs_state_(1:3);
            
            %% 3D view update
            % update text
            set(obj.h3d_text_, 'Position', ...
                [pos(1), pos(2), pos(3) + obj.obs_size_(3)]);
            % update size
            [data_size_x_, data_size_y_, data_size_z_] = ...
                ellipsoid(pos(1), pos(2), pos(3), ... 
                obj.obs_size_(1), obj.obs_size_(2), obj.obs_size_(3));
            set(obj.h3d_size_, 'XData', data_size_x_, ...
                'YData', data_size_y_, 'ZData', data_size_z_); 
            
            %% side view update
            % update size
            obs_size_points = pointsEllipse([pos(1), pos(2)], ...
                obj.obs_size_(1), obj.obs_size_(2));
            set(obj.h2d_td_size_, 'XData', obs_size_points(1,:), ...
                'YData', obs_size_points(2,:));
            obs_size_points = pointsEllipse([pos(1), pos(3)], ...
                obj.obs_size_(1), obj.obs_size_(3));
            set(obj.h2d_sx_size_, 'XData', obs_size_points(1,:), ...
                'YData', obs_size_points(2,:));
            
        end
        
        
    end
    
    
end