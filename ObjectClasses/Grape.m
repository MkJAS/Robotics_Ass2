classdef Grape < Item
    %!Grape class

    properties (Constant)
        diameter = 0.025;
    end

    properties
        object;
        vertices;
    end

    methods
        %% Constructor
        function self = Grape(location)
            self = self@Item(location);
            self.object = PlaceObject('Grape.ply', self.location);
            self.vertices = get(self.object, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.object); end
        end

    end

end
