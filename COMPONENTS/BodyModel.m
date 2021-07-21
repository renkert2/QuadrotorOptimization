classdef BodyModel < Model
    % Wraps equations to simulate 3D dynamics of the quadrotor.  Refer to weekly report 6/15/2021 for the development.
    properties (Constant)
        g double = 9.8067
    end
    
    properties (SetAccess = private)
        StateSyms (:,1) string
        InputSyms (:,1) string
        I struct % State and Input Indices
    end
    
    properties % Parameters for Body Model
        m compParam
        J compParam
        J_r compParam
        K_T compParam
        K_Q compParam
        l compParam
    end
    
    methods
        function obj = BodyModel(params)
            obj.Name = "QuadrotorBodyModel";
            obj.Nx = 12;
            obj.Nu = 4;
            obj.Nd = 3;
            obj.Ny = 12; % Full State Feedback
            
            obj.x0 = zeros(obj.Nx,1);
            
            obj.I = struct();
            obj.I.x.p = 1:3;
            obj.I.x.v = 4:6;
            obj.I.x.Theta = 7:9;
            obj.I.x.omega = 10:12;
            
            obj.I.u.omega_tilde = 1:4;
            
            obj.StateSyms = ["x","y","z","v_x","v_y","v_z","phi","theta","psi","omega_x","omega_y","omega_z"];
            obj.StateDescriptions = ["X Position", "Y Position", "Z Position", "X Velocity", "Y Velocity", "Z Velocity", "Roll", "Pitch", "Yaw", "X Angular Velocity", "Y Angular Velocity", "Z Angular Velocity"];
            obj.InputSyms = ["omega_tilde_1", "omega_tilde_2", "omega_tilde_3", "omega_tilde_4"];
            obj.InputDescriptions = ["Rotor Speed 1", "Rotor Speed 2", "Rotor Speed 3", "Rotor Speed 4"];
            obj.DisturbanceDescriptions = ["X Force", "Y Force", "Z Force"];
            obj.OutputDescriptions = obj.StateDescriptions;
            
            if nargin
                obj.m = get(params, "Mass", "QuadRotor"); % Must be set before calling genMatlabFunctions
                obj.J = get(params, "J", "QuadRotor");
                
                obj.J_r = get(params, "J_r", "MotorProp");

                obj.K_T = get(params, "K_T", "Propeller");
                obj.K_Q = get(params, "K_Q", "Propeller");
                
                obj.l = get(params, "l", "Frame");
                
                obj.Params = compParam.gatherObjectParams(obj);
                obj.Params.update();
                
                obj.init();
            end
        end
        
        function init(obj)
            for i = 1:numel(obj.Params)
                obj.Params(i).Tunable = true;
            end
            obj.setSymVars();
            setModelSymFunctions(obj)
            init@Model(obj);
        end
        
        function setModelSymFunctions(obj)
            x = obj.SymVars.x;
            d = obj.SymVars.d;
            p = x(obj.I.x.p);
            v = x(obj.I.x.v);
            
            Theta = x(obj.I.x.Theta);
            phi = Theta(1);
            theta = Theta(2);
            psi = Theta(3);
            
            omega = x(obj.I.x.omega);
            
            u = obj.SymVars.u;
            omega_tilde = u;
            
            % Params: J,m,J_r,K_T,K_Q,l
            J = obj.J.Sym_;
            m = obj.m.Sym_;
            J_r = obj.J_r.Sym_;
            K_T = obj.K_T.Sym_;
            K_Q = obj.K_Q.Sym_;
            l = obj.l.Sym_;
            
            e_3 = [0;0;1];
            W = obj.W(phi,theta);
            Rbe = obj.Rbe(phi,theta,psi);
            
            g = obj.g;
            
            [f,tau] = obj.controlEffectiveness(omega_tilde, K_T, K_Q, l);

            G_a = J_r*([-1 1 -1 1]*omega_tilde)*(cross(e_3, omega));

            obj.f_sym = [v;...
                g*e_3 + (1/m)*Rbe*(-f*e_3) + d./m;...
                W*omega;...
                inv(J)*(-cross(omega, J*omega) + tau + G_a)];   
            
            obj.g_sym = x;
        end
        
        function [ax] = plot(obj, x_arg, opts)
            arguments
                obj
                x_arg
                opts.Interval double = 10
                opts.ParentAxes matlab.graphics.axis.Axes = matlab.graphics.axis.Axes.empty()
                opts.RefTraj timeseries = timeseries.empty()
                opts.Annotate logical = true
                opts.TrajectoryNames string = string.empty()
            end
            
            if ~isempty(opts.ParentAxes)
                ax = opts.ParentAxes;
            else
                f = figure();
                f.Position = [300 425 1100 550];
                ax = axes(f);
            end
            
            if ~isempty(opts.RefTraj)
                ln_ref_flag = true;
                x = opts.RefTraj.Data;
                ln_ref = plot3(ax, x(:,1), -x(:,2), -x(:,3), '-b');
            else
                ln_ref_flag = false;
            end
            
            if isa(x_arg,'double')
                x = x_arg;
                trans = x(:,obj.I.x.p);
                rots = eul2quat([1 -1 -1].*fliplr(x(:,obj.I.x.Theta)),'ZYX');
                
                hold on
                traj_children = plot3(ax, trans(:,1), -trans(:,2), -trans(:,3), '-k');
                hold off
                
                traj_children.DisplayName = 'Trajectory';
                traj_names = "Trajectory";
            elseif isa(x_arg,'cell')
                x_primary = x_arg{1};
                rots = eul2quat([1 -1 -1].*fliplr(x_primary(:,obj.I.x.Theta)),'ZYX');
                trans_all = cellfun(@(x) x(:,obj.I.x.p), x_arg, 'UniformOutput', false);
                
                hold on
                N = numel(trans_all);
                traj_children = matlab.graphics.GraphicsPlaceholder.empty(0,N);
                traj_names = strings(1,N);
                for i = 1:numel(trans_all)
                    trans = trans_all{i};
                    traj_children(1,i) = plot3(ax, trans(:,1), -trans(:,2), -trans(:,3));
                    traj_names(1,i) = sprintf("Trajectory %d", i);
                end
                trans = trans_all{1};
            else
                error("x_arg must be a double of size [N_t,N_x] or a cell array of double arrays");
            end
            
            trans_paint = trans(1:opts.Interval:end, :);
            rots_paint = rots(1:opts.Interval:end, :);
            
            
            daspect([1 1 1])
            grid on
            view(ax, 3)
            axis padded
            if opts.Annotate
                title("QuadRotor Trajectory Plot")
                xlabel("$$x$$", 'Interpreter', 'latex')
                ylabel("$$-y$$", 'Interpreter', 'latex')
                zlabel("$$-z$$", 'Interpreter', 'latex')
            end

            meshPath = robotics.internal.validation.findFilePath('multirotor.stl', 'plotTransforms', 'MeshFilePath');
            inertialZDirection = 'down';
            painter = robotics.core.internal.visualization.TransformPainter(ax, meshPath, false);
            painter.Color = [1 0 0];
            painter.Size = 1;
            painter.InertialZDownward = strcmp(inertialZDirection, 'down');
            
            for i = 1:size(trans_paint, 1)
                painter.paintAt(trans_paint(i,:), rots_paint(i,:));
            end
            
            painter.HandleXAxis.DisplayName = 'Body X axis';
            painter.HandleYAxis.DisplayName = 'Body Y axis';
            painter.HandleZAxis.DisplayName = 'Body Z axis';
            
            if opts.Annotate
                lgnd_children = [traj_children, painter.HandleXAxis, painter.HandleYAxis, painter.HandleZAxis];
                if ~isempty(opts.TrajectoryNames)
                    traj_names = opts.TrajectoryNames;
                end
                lgnd_labels = [traj_names, "Body X axis", "Body Y axis", "Body Z axis"];
                if ln_ref_flag
                    lgnd_children = [ln_ref, lgnd_children];
                    lgnd_labels = ["Reference Trajectory", lgnd_labels];
                end
                legend(ax, lgnd_children, lgnd_labels)
            end
        end
        
        function ax = animate(obj, t, x, opts)
            arguments
                obj
                t double
                x double
                opts.ParentAxes matlab.graphics.axis.Axes = matlab.graphics.axis.Axes.empty()
                opts.RefTraj ReferenceTrajectory3D = ReferenceTrajectory3D.empty()
            end
            
            trans = x(:,obj.I.x.p);
            rots = eul2quat([1 -1 -1].*fliplr(x(:, obj.I.x.Theta)),'ZYX');
            t_diff = diff(t);
            
            if ~isempty(opts.ParentAxes)
                ax = opts.ParentAxes;
            else
                f = figure();
                f.Position = [300 425 1100 550];
                ax = axes(f);
            end
            
            if ~isempty(opts.RefTraj)
                ref_flag = true;
                ln_ref = opts.RefTraj.plot('ParentAxes', ax);
                final_ref_t =  opts.RefTraj.t(end);
            else
                ref_flag = false;
            end
            
            daspect([1 1 1])
            grid on
            view(ax, 3)
            axis padded
            title("QuadRotor Trajectory Animation")
            xlabel("$$x$$", 'Interpreter', 'latex')
            ylabel("$$-y$$", 'Interpreter', 'latex')
            zlabel("$$-z$$", 'Interpreter', 'latex')
            
            hold on
            ln = animatedline(ax);
            if ref_flag
                [~,x,y,z] = R_t(opts.RefTraj, t(1));
                ref_point = plot3(x,y,z,'.r','MarkerSize',20);
            end
            hold off
            
            meshPath = robotics.internal.validation.findFilePath('multirotor.stl', 'plotTransforms', 'MeshFilePath');
            inertialZDirection = 'down';
            painter = robotics.core.internal.visualization.TransformPainter(ax, meshPath, false);
            painter.Color = [1 0 0];
            painter.Size = 1;
            painter.InertialZDownward = strcmp(inertialZDirection, 'down');
            hMeshTransform = painter.paintAt(trans(1, :), rots(1, :));
            
            painter.HandleXAxis.DisplayName = 'Body X axis';
            painter.HandleYAxis.DisplayName = 'Body Y axis';
            painter.HandleZAxis.DisplayName = 'Body Z axis';
            ln.DisplayName = 'Trajectory';

            lgnd_children = [ln, painter.HandleXAxis, painter.HandleYAxis, painter.HandleZAxis];
            lgnd_labels = ["Trajectory", "Body X axis", "Body Y axis", "Body Z axis"];
            if ref_flag
                lgnd_children = [ln_ref, lgnd_children];
                lgnd_labels = ["Reference Trajectory", lgnd_labels];
            end
            legend(ax, lgnd_children, lgnd_labels)
            
            tic()
            for i = 2:numel(t)
                painter.move(hMeshTransform, trans(i, :), rots(i, :));
                addpoints(ln, trans(i,1), -trans(i,2), -trans(i,3));
                if ref_flag
                    [~,x,y,z] = R_t(opts.RefTraj, min(t(i), final_ref_t));
                    set(ref_point, 'XData', x, 'YData', -y, 'ZData', -z)
                end
                plt_time = toc();
                pause(t_diff(i-1) - plt_time)
                tic();
                drawnow
            end
        end
    end
    methods (Static)
        function w = W(phi,theta)
            w = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
                0 cos(phi) -sin(phi);...
                0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
        end
        
        function rz = Rz(psi)
            rz = [cos(psi) sin(psi) 0;...
                -sin(psi) cos(psi) 0;...
                0 0 1];
        end
        
        function ry = Ry(theta)
                ry = [cos(theta) 0 -sin(theta);...
                0 1 0;...
                sin(theta) 0 cos(theta)];
        end
        
        function rx = Rx(phi)
            rx = [1 0 0;...
                0 cos(phi) sin(phi);...
                0 -sin(phi) cos(phi)];
        end
        
        function rbe = Rbe(phi,theta,psi)
            rbe = BodyModel.Rx(phi)*BodyModel.Ry(theta)*BodyModel.Rz(psi);
        end
        
        function [f,tau] = controlEffectiveness(omega_tilde, K_T, K_Q, l)
            x = [K_T K_T K_T K_T;...
                l*K_T -l*K_T -l*K_T l*K_T;...
                l*K_T l*K_T -l*K_T -l*K_T;...
                K_Q -K_Q K_Q -K_Q]*omega_tilde.^2;
            f = x(1);
            tau = x(2:4);
        end
    end
end

