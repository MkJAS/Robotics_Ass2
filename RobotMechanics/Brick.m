classdef Brick
    %Brick class which stores key brick attributes and places at brick at
    %the desired location upon instantiation. Destructor also clears the
    %Brick from the screen if the selfect is deleted.
    %Brick origin at Z zero in the middle of the brick.

    properties (Constant)
        length = 0.275;
        width = 0.1335;
        height = 0.0735;
        gapX = 0.175;
        gapY = 0.3;
    end

    properties
        locationInitial;
        locationFinal;
        brick;
        vertices;
    end

    methods
        %% Constructor
        function self = Brick(locationInitial, locationFinal, stackedOrGrid)

            switch (nargin)
                case 2
                    self.locationInitial = locationInitial;
                    self.locationFinal = locationFinal;
                    self.brick = PlaceObject('Brick.ply', self.locationInitial);

                case 3
                    self.locationInitial = locationInitial;
                    self.locationFinal = locationFinal;

                    if (stackedOrGrid == true)
                        self.brick = PlaceObject('BrickStacked.ply', self.locationFinal);
                    else
                        self.brick = PlaceObject('Brick.ply', self.locationInitial);
                    end

            end

            self.vertices = get(self.brick, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.brick); end
        end

    end

end
