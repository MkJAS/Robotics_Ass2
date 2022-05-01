classdef Strawberry < Fruit
    %!Strawberry class

    properties (Constant)
        diameter = 0.05; % todo update
    end

    properties
        strawberry;
        vertices;
    end

    methods
        %% Constructor
        function self = Strawberry(location)
            self = self@Fruit(location);
            self.strawberry = PlaceObject('Strawberry.ply', self.location);
            self.vertices = get(self.strawberry, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.strawberry); end
        end

    end

end
