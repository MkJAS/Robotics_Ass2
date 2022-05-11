classdef Strawberry < Item
    %!Strawberry class

    properties (Constant)
        diameter = 0.025;
    end

    properties
        type = "strawberry";
        object;
        vertices;
    end

    methods
        %% Constructor
        function self = Strawberry(location)
            self = self@Item(location);
            self.object = PlaceObject('Strawberry.ply', self.location);
            self.vertices = get(self.object, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.object); end
        end

    end

end
