classdef Fruit
    %!Brick class which stores key brick attributes and places at brick at

    properties (Constant)
        % length = 0.275;
        % width = 0.1335;
        height = 0.05;
    end

    properties
        location;
        fruit;
        vertices;
    end

    methods
        %% Constructor
        function self = Fruit(location)

            self.location = location;
            self.fruit = PlaceObject('Grape.ply', self.location);
            self.vertices = get(self.fruit, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.fruit); end
        end

    end

end
