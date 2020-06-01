classdef CVisualQuad < handle
    % CVisualQuad Handles visualisation of quadrotor
    
    properties
        
        %% ID and N
        id_             =   0;
        N_              =   0;
        
        %% Running configuration
        cfg_
        
        %% Color, scale, line width
        color_
        scale_
        line_width_
        
        %% Physical propertices
        quad_size_      =   [];         % [a, b, c], m
        beam_dia_       =   0;          % beam diameter, m
        propeller_dia_  =   0;          % propeller diameter, m
        
        %% Goal, state and path
        quad_goal_      =   [];
        quad_state_     =   [];
        quad_state_cov_ =   [];
        quad_path_      =   [];
        quad_path_cov_  =   [];
        
        %% 3d plot handles
        % identify text
        h3d_text_
        % quad beam
        h3d_beam1_
        h3d_beam2_
        % quad propeller
        h3d_propeller_
        % quad ellipse size
        h3d_size_
        % quad head
        h3d_head_
        % quad goal
        h3d_goal_
        % quad mpc plan
        h3d_path_
        % quad enlarged cov ellipsoid
        h3d_cov_
        % quad mpc plan with enlarged cov ellipsoid
        h3d_path_cov_
        
        %% Top-down/side X view handles
        % quad as a point
        h2d_td_quad_
        h2d_sx_quad_
        % size ellipse boundaty
        h2d_td_size_
        h2d_sx_size_
        % quad goal
        h2d_td_goal_
        h2d_sx_goal_
        % path
        h2d_td_path_
        h2d_sx_path_
        % quad cov
        h2d_td_cov_
        h2d_sx_cov_
        % quad path cov
        h2d_td_path_cov_
        h2d_sx_path_cov_
        
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CVisualQuad(quadID, N, quad_size, cfg, axesHandle, subaxesHandle, color, scale)
            % Constructor
            
            %% parameters
            obj.id_     =   quadID;
            obj.N_      =   N;
            obj.color_  =   color;
            obj.scale_  =   scale;
            
            obj.cfg_    =   cfg;
            
            obj.quad_size_      =   quad_size;
            obj.line_width_     =   3.0 * obj.scale_;
            obj.beam_dia_       =   0.24 * obj.scale_;
            obj.propeller_dia_  =   0.15 * obj.scale_;
            
            %% initialization
            obj.quad_goal_  =   zeros(4, 1);
            obj.quad_state_ =   zeros(9, 1);
            obj.quad_path_  =   zeros(3, obj.N_);
            obj.quad_state_cov_ = eye(9);
            obj.quad_path_cov_  = zeros(3, 3, obj.N_);
            for i = 1 : obj.N_
                obj.quad_path_cov_(:, :, i) = eye(3);
            end
            
            %% 3d view objects
            % identify text
            quad_text = char('Quad ' + string(obj.id_));
            obj.h3d_text_ = text(axesHandle, 0, 0, 0.2*obj.scale_, ...
                quad_text, 'FontSize', 8);
            % quad crossing beams
            rotCenter(1,:) = obj.beam_dia_*[-1.0, 1.0, 0];
            rotCenter(2,:) = obj.beam_dia_*[ 1.0, 1.0, 0];
            rotCenter(3,:) = obj.beam_dia_*[ 1.0,-1.0, 0];
            rotCenter(4,:) = obj.beam_dia_*[-1.0,-1.0, 0];
            obj.h3d_beam1_ = line(axesHandle, ...
                [rotCenter(1,1), rotCenter(3,1)], ...
                [rotCenter(1,2), rotCenter(3,2)], ...
                [rotCenter(1,3), rotCenter(3,3)], ...
                'color', obj.color_, ...
                'linewidth', obj.line_width_);
            obj.h3d_beam2_ = line(axesHandle, ...
                [rotCenter(2,1), rotCenter(4,1)], ...
                [rotCenter(2,2), rotCenter(4,2)], ...
                [rotCenter(2,3), rotCenter(4,3)], ...
                'color', obj.color_, ...
                'linewidth', obj.line_width_);
            % quad propellers
            for i = 1:4
                rotor_points = pointsCircle3D(rotCenter(i,:), ...
                    [0, 0, 1], obj.propeller_dia_);
                obj.h3d_propeller_(i) = plot3(axesHandle, ...
                    rotor_points(1,:), rotor_points(2,:), rotor_points(3,:), ...
                    '-','color', obj.color_, ...
                    'linewidth', obj.line_width_);
            end
            % quad ellipse size 
            [data_size_x_, data_size_y_, data_size_z_] = ...
                ellipsoid(0, 0, 0, ... 
                obj.quad_size_(1), obj.quad_size_(2), obj.quad_size_(3));
            obj.h3d_size_ = surface(axesHandle, ...
                data_size_x_, data_size_y_, data_size_z_, ...
                'EdgeColor', obj.color_, ...
                'EdgeAlpha', 0.1, ...
                'FaceColor', obj.color_, ...
                'FaceAlpha', 0.1); 
            if obj.cfg_.ifShowQuadSize
                set(obj.h3d_size_, 'Visible', 'On');
            else
                set(obj.h3d_size_, 'Visible', 'Off');
            end
            % quad head
            obj.h3d_head_ = line(axesHandle, [0, obj.beam_dia_], ...
                [0, 0], [0, 0], 'color', obj.color_, ...
                'linewidth', obj.line_width_); 
            if obj.cfg_.ifShowQuadHead
                set(obj.h3d_head_, 'Visible', 'On');
            else
                set(obj.h3d_head_, 'Visible', 'Off');
            end
            % quad goal
            obj.h3d_goal_ = plot3(axesHandle, [0 0], [0 0], [0 0], 'd', ...
                'MarkerSize', 10, 'MarkerEdgeColor', obj.color_, ...
                'MarkerFaceColor', obj.color_);
            if obj.cfg_.ifShowQuadGoal
                set(obj.h3d_goal_, 'Visible', 'On');
            else
                set(obj.h3d_goal_, 'Visible', 'Off');
            end
            % quad mpc plan
            obj.h3d_path_ = plot3(axesHandle, [0 0], [0 0], [0 0], ...
                'color', obj.color_, 'LineWidth', 1.5);
            if obj.cfg_.ifShowQuadPath
                set(obj.h3d_path_, 'Visible', 'On');
            else
                set(obj.h3d_path_, 'Visible', 'Off');
            end
            % quad cov
            [data_cov_x_, data_cov_y_, data_cov_z_] = ...
                ellipsoid(0, 0, 0, ... 
                obj.quad_size_(1), obj.quad_size_(2), obj.quad_size_(3));
            obj.h3d_cov_ = surface(axesHandle, ...
                data_cov_x_, data_cov_y_, data_cov_z_, ...
                'EdgeColor', obj.color_, ...
                'EdgeAlpha', 0.1, ...
                'FaceColor', obj.color_, ...
                'FaceAlpha', 0.1); 
            if obj.cfg_.ifShowQuadCov
                set(obj.h3d_cov_, 'Visible', 'On');
            else
                set(obj.h3d_cov_, 'Visible', 'Off');
            end
            % quad path cov
            for iTemp = 1 : obj.cfg_.quadPathCovShowNum
                [data_cov_x_, data_cov_y_, data_cov_z_] = ...
                ellipsoid(0, 0, 0, ... 
                obj.quad_size_(1), obj.quad_size_(2), obj.quad_size_(3));
                obj.h3d_path_cov_(iTemp) = surface(axesHandle, ...
                    data_cov_x_, data_cov_y_, data_cov_z_, ...
                    'EdgeColor', obj.color_, ...
                    'EdgeAlpha', 0.1, ...
                    'FaceColor', obj.color_, ...
                    'FaceAlpha', 0.1);
                if obj.cfg_.ifShowQuadPathCov
                    set(obj.h3d_path_cov_(iTemp), 'Visible', 'On');
                else
                    set(obj.h3d_path_cov_(iTemp), 'Visible', 'Off');
                end                    
            end

            %% side view objects
            % quad as a point
            obj.h2d_td_quad_ = scatter(subaxesHandle(1), 0, 0, 'filled', 'o', ...
                'MarkerEdgeColor', obj.color_, ...
                'MarkerFaceColor', obj.color_, ...
                'LineWidth', 1.0);
            obj.h2d_sx_quad_ = scatter(subaxesHandle(2), 0, 0, 'filled', 'o', ...
                'MarkerEdgeColor', obj.color_, ...
                'MarkerFaceColor', obj.color_, ...
                'LineWidth', 1.0);
            % quad size
            quad_size_points = pointsEllipse([0,0], ...
                obj.quad_size_(1), obj.quad_size_(2));
            obj.h2d_td_size_ = plot(subaxesHandle(1), ...
                quad_size_points(1,:), quad_size_points(2,:), '-', ...
                'color', obj.color_, ...
                'linewidth', obj.line_width_);
            quad_size_points = pointsEllipse([0,0], ...
                obj.quad_size_(1), obj.quad_size_(3));
            obj.h2d_sx_size_ = plot(subaxesHandle(2), ...
                quad_size_points(1,:), quad_size_points(2,:), '-', ...
                'color', obj.color_, ...
                'linewidth', obj.line_width_);
            if obj.cfg_.ifShowQuadSize
                set(obj.h2d_td_size_, 'Visible', 'On');
                set(obj.h2d_sx_size_, 'Visible', 'On');
            else
                set(obj.h2d_td_size_, 'Visible', 'Off');
                set(obj.h2d_sx_size_, 'Visible', 'Off');
            end
            % quad goal
            obj.h2d_td_goal_ = plot(subaxesHandle(1), [0 0], [0 0], 'd', ...
                'MarkerSize', 10, 'MarkerEdgeColor', obj.color_, ...
                'MarkerFaceColor', obj.color_);
            obj.h2d_sx_goal_ = plot(subaxesHandle(2), [0 0], [0 0], 'd', ...
                'MarkerSize', 10, 'MarkerEdgeColor', obj.color_, ...
                'MarkerFaceColor', obj.color_);
            if obj.cfg_.ifShowQuadGoal
                set(obj.h2d_td_goal_, 'Visible', 'On');
                set(obj.h2d_sx_goal_, 'Visible', 'On');
            else
                set(obj.h2d_td_goal_, 'Visible', 'Off');
                set(obj.h2d_sx_goal_, 'Visible', 'Off');
            end
            % quad path
            obj.h2d_td_path_ = plot(subaxesHandle(1), [0 0], [0 0], ...
                'color', obj.color_, 'LineWidth', 1.5);
            obj.h2d_sx_path_ = plot(subaxesHandle(2), [0 0], [0 0], ...
                'color', obj.color_, 'LineWidth', 1.5);
            if obj.cfg_.ifShowQuadPath
                set(obj.h2d_td_path_, 'Visible', 'On');
                set(obj.h2d_sx_path_, 'Visible', 'On');
            else
                set(obj.h2d_td_path_, 'Visible', 'Off');
                set(obj.h2d_sx_path_, 'Visible', 'Off');
            end
            
        end
        
        
        %%  =======================================================================
        function setPose(obj)
            % Update visulization of quadrotor
            
            % position
            pos = obj.quad_state_(1:3);
            
            % rotation matrix
            r = obj.quad_state_(7);
            p = obj.quad_state_(8);
            y = obj.quad_state_(9);
            R = [cos(y)*cos(p), cos(y)*sin(p)*sin(r) - sin(y)*cos(r), cos(y)*sin(p)*cos(r) + sin(y)*sin(r); ...
                 sin(y)*cos(p), sin(y)*sin(p)*sin(r) + cos(y)*cos(r), sin(y)*sin(p)*cos(r) - cos(y)*sin(r); ...
                -sin(p), cos(p)*sin(r), cos(p)*cos(r)];
            
            %% 3D view update
            % update text
            text_pos = pos + 0.3*obj.scale_*R'*[1.5;1.5;1];
            set(obj.h3d_text_, 'Position', ...
                [text_pos(1), text_pos(2), text_pos(3)]);            
            % update beam
            rotCenter(1,:) = pos + R * obj.beam_dia_ * [-1, 1, 0]';
            rotCenter(2,:) = pos + R * obj.beam_dia_ * [ 1, 1, 0]';
            rotCenter(3,:) = pos + R * obj.beam_dia_ * [ 1,-1, 0]';
            rotCenter(4,:) = pos + R * obj.beam_dia_ * [-1,-1, 0]';
            set(obj.h3d_beam1_, 'XData', [rotCenter(1,1), rotCenter(3,1)], ...
                'YData', [rotCenter(1,2), rotCenter(3,2)], ...
                'ZData', [rotCenter(1,3), rotCenter(3,3)]);
            set(obj.h3d_beam2_, 'XData', [rotCenter(2,1), rotCenter(4,1)], ...
                'YData', [rotCenter(2,2), rotCenter(4,2)], ...
                'ZData', [rotCenter(2,3), rotCenter(4,3)]);
            % update propeller
            for i = 1:4
                rotor_points = pointsCircle3D(rotCenter(i,:), R(:,3).', obj.propeller_dia_);
                set(obj.h3d_propeller_(i), 'XData', rotor_points(1,:), ...
                    'YData', rotor_points(2,:), 'ZData',rotor_points(3,:));           
            end
            % update size  
            if obj.cfg_.ifShowQuadSize
                [data_size_x_, data_size_y_, data_size_z_] = ...
                    ellipsoid(pos(1), pos(2), pos(3), ... 
                    obj.quad_size_(1), obj.quad_size_(2), obj.quad_size_(3));
                set(obj.h3d_size_, 'XData', data_size_x_, ...
                    'YData', data_size_y_, 'ZData', data_size_z_);  
                set(obj.h3d_size_, 'Visible', 'On');
            else
                set(obj.h3d_size_, 'Visible', 'Off');
            end
            % update head
            if obj.cfg_.ifShowQuadHead
                new_head = pos + obj.beam_dia_*R(:,1);
                set(obj.h3d_head_, 'XData', [pos(1), new_head(1)], ...
                    'YData', [pos(2), new_head(2)], ...
                    'ZData', [pos(3), new_head(3)]);
                set(obj.h3d_head_, 'Visible', 'On');
            else
                set(obj.h3d_head_, 'Visible', 'Off');
            end
            % update goal
            if obj.cfg_.ifShowQuadGoal
                set(obj.h3d_goal_, 'XData', obj.quad_goal_(1), ...
                    'YData', obj.quad_goal_(2), 'ZData', obj.quad_goal_(3), ...
                    'Visible', 'On');
            else
                set(obj.h3d_goal_, 'Visible', 'Off');
            end
            % update mpc path
            if obj.cfg_.ifShowQuadPath
                set(obj.h3d_path_, 'XData', obj.quad_path_(1, :), ...
                    'YData', obj.quad_path_(2, :), ...
                    'ZData', obj.quad_path_(3, :), 'Visible', 'On');
            else
                set(obj.h3d_path_, 'XData', obj.quad_path_(1, :), ...
                    'YData', obj.quad_path_(2, :), ...
                    'ZData', obj.quad_path_(3, :), 'Visible', 'Off');
            end
            % update cov
            if obj.cfg_.ifShowQuadCov
                pos_cov = obj.quad_state_cov_(1:3, 1:3);
                [data_cov_x_, data_cov_y_, data_cov_z_] = ...
                    pointsErrorEllipsoid(pos, pos_cov, ...
                    obj.cfg_.quad.Mahalanobis, obj.quad_size_(1));
                set(obj.h3d_cov_, 'XData', data_cov_x_, ...
                    'YData', data_cov_y_, 'ZData', data_cov_z_);
                set(obj.h3d_cov_, 'Visible', 'On');
            else
                set(obj.h3d_cov_, 'Visible', 'Off');
            end
            % update path cov
            gapTemp = floor(obj.N_/obj.cfg_.quadPathCovShowNum);
            for iTemp = 1 : obj.cfg_.quadPathCovShowNum
                if obj.cfg_.ifShowQuadPathCov
                    pos_iTemp = obj.quad_path_(1:3, iTemp*gapTemp);
                    pos_cov_iTemp = obj.quad_path_cov_(1:3, 1:3, iTemp*gapTemp);
                    [data_cov_x_, data_cov_y_, data_cov_z_] = ...
                        pointsErrorEllipsoid(pos_iTemp, pos_cov_iTemp, ...
                        obj.cfg_.quad.Mahalanobis, obj.quad_size_(1));
                    set(obj.h3d_path_cov_(iTemp), 'XData', data_cov_x_, ...
                        'YData', data_cov_y_, 'ZData', data_cov_z_);
                    set(obj.h3d_path_cov_(iTemp), 'Visible', 'On');
                else
                    set(obj.h3d_path_cov_(iTemp), 'Visible', 'Off');
                end
            end
            
            
            %% side view update
            % update position
            set(obj.h2d_td_quad_, 'XData', pos(1), 'YData', pos(2));
            set(obj.h2d_sx_quad_, 'XData', pos(1), 'YData', pos(3));
            % update size
            if obj.cfg_.ifShowQuadSize
                quad_size_points = pointsEllipse([pos(1), pos(2)], ...
                    obj.quad_size_(1), obj.quad_size_(2));
                set(obj.h2d_td_size_, 'XData', quad_size_points(1,:), ...
                    'YData', quad_size_points(2,:), 'Visible', 'On');
                quad_size_points = pointsEllipse([pos(1), pos(3)], ...
                    obj.quad_size_(1), obj.quad_size_(3));
                set(obj.h2d_sx_size_, 'XData', quad_size_points(1,:), ...
                    'YData', quad_size_points(2,:), 'Visible', 'On');
            else
                set(obj.h2d_td_size_, 'Visible', 'Off');
                set(obj.h2d_sx_size_, 'Visible', 'Off');
            end
            % update goal
            if obj.cfg_.ifShowQuadGoal
                set(obj.h2d_td_goal_, 'XData', obj.quad_goal_(1), ...
                    'YData', obj.quad_goal_(2), 'Visible', 'On');
                set(obj.h2d_sx_goal_, 'XData', obj.quad_goal_(1), ...
                    'YData', obj.quad_goal_(3), 'Visible', 'On');
            else
                set(obj.h2d_td_goal_, 'Visible', 'Off');
                set(obj.h2d_sx_goal_, 'Visible', 'Off');
            end
            % update path
            if obj.cfg_.ifShowQuadPath
                set(obj.h2d_td_path_, 'XData', obj.quad_path_(1, :), ...
                    'YData', obj.quad_path_(2, :), 'Visible', 'On');
                set(obj.h2d_sx_path_, 'XData', obj.quad_path_(1, :), ...
                    'YData', obj.quad_path_(3, :), 'Visible', 'On');
            else
                set(obj.h2d_td_path_, 'Visible', 'Off');
                set(obj.h2d_sx_path_, 'Visible', 'Off');
            end
            
        end
        
        
    end
    
end
