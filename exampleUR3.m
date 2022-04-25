classdef UR3 < handle

    properties
        model;
        workspace = [-2 2 -2 2 -0.3 2]; %currently the same as UR5
        useGripper = false;
        base;
        name = 'UR3';
        qIntermediary;

    end

    methods %% Class for UR3 robot simulation

        function self = UR3(base, qIntermediary)

            if nargin < 1
                base = transl(0, 0, 0);
            end

            self.base = base;
            self.GetUR3Robot();
            self.PlotAndColourRobot();
            self.qIntermediary = qIntermediary;

        end

        %% ShowReach - minorly adapted from lab 3 solutions
        function ShowReach(self)

            stepRads = deg2rad(65);
            qlim = self.model.qlim;

            pointCloudSize = prod(floor((qlim(1:5, 2) - qlim(1:5, 1)) / stepRads + 1));
            pointCloud = zeros(pointCloudSize, 3);
            q6 = 0; % Assume q6 = 0
            counter = 1;
            tic;

            for q1 = qlim(1, 1):stepRads:qlim(1, 2)

                for q2 = qlim(2, 1):stepRads:qlim(2, 2)

                    for q3 = qlim(3, 1):stepRads:qlim(3, 2)

                        for q4 = qlim(4, 1):stepRads:qlim(4, 2)

                            for q5 = qlim(5, 1):stepRads:qlim(5, 2)
                                q = [q1, q2, q3, q4, q5, q6];
                                tr = self.model.fkine(q);
                                pointCloud(counter, :) = tr(1:3, 4)';
                                counter = counter + 1;

                                if mod(counter / pointCloudSize * 100, 1) == 0
                                    disp(['After ', num2str(toc), ' seconds, completed ', num2str((counter / pointCloudSize * 100)), '% of poses']);
                                end

                            end

                        end

                    end

                end

            end

            % Lab3, 2.6: Create a 3D model showing where the end effector can be over all these samples.
            pointCloudPlot = plot3(pointCloud(:, 1), pointCloud(:, 2), pointCloud(:, 3), 'r.');
            [~, volume] = convhull(pointCloud(:, 1), pointCloud(:, 2), pointCloud(:, 3));
            disp(["Approx Volume of UR3 in square metres: " num2str(volume)]);
            radius = nthroot((volume / (3/4) / pi), 3);
            disp(["Approx radius calculated by area in metres: " num2str(radius)])
            input("Press Enter to continue");
            try delete(pointCloudPlot); end
        end

        %% GetUR3Robot
        % Given a name, create and return a UR3 robot model
        function GetUR3Robot(self)
            pause(0.001);
            self.name = 'UR_3';

            % Create the UR3 model
            L(1) = Link('d', 0.1519, 'a', 0, 'alpha', pi / 2, 'offset', 0, 'qlim', [-2 * pi 2 * pi]);
            L(2) = Link('d', 0, 'a', -0.24365, 'alpha', 0, 'offset', 0, 'qlim', [-pi 0]);
            L(3) = Link('d', 0, 'a', -0.21325, 'alpha', 0, 'offset', 0, 'qlim', [-deg2rad(150) deg2rad(150)]);
            L(4) = Link('d', -0.11235, 'a', 0, 'alpha', pi / 2, 'offset', 0, 'qlim', [-2 * pi 2 * pi]); %! Positive 0.11235 from Universal Robots' Website
            L(5) = Link('d', 0.08535, 'a', 0, 'alpha', -pi / 2, 'offset', 0, 'qlim', [-2 * pi 2 * pi]);
            L(6) = Link('d', 0.0819, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [-2 * pi 2 * pi]);
            self.model = SerialLink(L, 'name', self.name, 'base', self.base);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)

            for linkIndex = 0:self.model.n

                if self.useGripper && linkIndex == self.model.n
                    [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['UR3Link', num2str(linkIndex), 'Gripper.ply'], 'tri'); %#ok<AGROW>
                else
                    [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['UR3Link', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
                end

                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);

            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight
            end

            self.model.delay = 0;

            % Correctly colour the arm (if colours are in ply file data)
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

    end

end
