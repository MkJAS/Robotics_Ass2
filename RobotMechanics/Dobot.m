classdef Dobot < handle

    properties
        %> Robot model
        model;
        jointLimits;
        workspace = [-1 1 -1 1 -0.3 1];
        name = 'Dobot';
        qIntermediary = deg2rad([-15 40 60 12.5 0]);
        base;

    end

    methods %% Class for Dobot robot simulation

        function self = Dobot(base)

            if nargin < 1
                base = transl(0, 0, 0);
            end

            self.base = base;
            self.GetDobotRobot();
            self.PlotAndColourRobot();
            self.GetJointLimits();

        end

        %% GetDobotRobot
        % Given a name (optional), create and return a Dobot robot model
        function GetDobotRobot(self)
            %     if nargin < 1
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['Dobot', datestr(now, 'yyyymmddTHHMMSSFFF')];
            %     end
            L(1) = Link('d', 0.138, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', [-135 * pi / 180 135 * pi / 180]);
            L(2) = Link('d', 0, 'a', 0.135, 'alpha', 0, 'offset', -pi / 2, 'qlim', [5 * pi / 180 80 * pi / 180]);
            L(3) = Link('d', 0, 'a', 0.147, 'alpha', pi, 'offset', 0, 'qlim', [-5 * pi / 180 85 * pi / 180]);
            L(4) = Link('d', 0, 'a', 0.05, 'alpha', pi/2, 'offset', 0, 'qlim', [-pi / 2 pi / 2]);
            L(5) = Link('d', 0.09, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [-85 * pi / 180 85 * pi / 180]);
            self.model = SerialLink(L, 'name', name, 'base', self.base);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self) %robot,workspace)

            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['DobotLink', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);

            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight
            end

            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles, 'UserData');

                try
                    h.link(linkIndex + 1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red ...
                                                                    , plyData{linkIndex + 1}.vertex.green ...
                                                                    , plyData{linkIndex + 1}.vertex.blue] / 255;
                    h.link(linkIndex + 1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end

            end

        end

        function GetJointLimits(self)
            qlim = self.model.qlim;
            qlim(2, :) = [5 * pi / 180 80 * pi / 180];
            qlim(3, :) = [5 * pi / 180 85 * pi / 180];
            lowerLimit = [];
            upperLimit = [];

            for q2 = qlim(2, 1):0.01:qlim(2, 2)

                for theta3 = qlim(3, 1):0.01:qlim(3, 2) + 0.01
                    q3 = pi / 2 - q2 + theta3;

                    if theta3 <= qlim(3, 1)
                        lowerLimit = [lowerLimit; q2, q3]; %#ok<AGROW>
                    elseif qlim(3, 2) <= theta3
                        upperLimit = [upperLimit; q2, q3]; %#ok<AGROW>
                    end

                end

            end

            self.jointLimits = [lowerLimit upperLimit];

        end

        function Spin(self)
            %Function that animates a robot rotating to a desired joint state
            %Takes a robot and the desired joint configuration.
            % angle needs to be -135 -> 135 degrees
            while (1)
                % AnimateRobots(robot, robot.qIntermediary);

                steps = 50;

                qCurrent = self.model.getpos();
                qTarget = qCurrent;
                qTarget(1) = deg2rad(-135);

                %Find q trajectory
                qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

                for i = 1:steps
                    self.model.animate(qTrajectory(i, :));
                    drawnow();
                end

                qCurrent = self.model.getpos();
                qTarget = qCurrent;
                qTarget(1) = deg2rad(135);

                %Find q trajectory
                qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

                for i = 1:steps
                    self.model.animate(qTrajectory(i, :));
                    drawnow();
                end

            end

        end

    end

end
