classdef Grape < Fruit
    %!Grape class

    properties (Constant)
        diameter = 0.025;
    end

    properties
        type = "grape";
        grape;
        vertices;
    end

    methods
        %% Constructor
        function self = Grape(location)
            self = self@Fruit(location);
            self.grape = PlaceObject('Grape.ply', self.location);
            self.vertices = get(self.grape, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.grape); end
        end

    end

end
