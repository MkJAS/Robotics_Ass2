classdef Lego < Item
    %!Grape class

    properties (Constant)
        diameter = 0.025;
    end

    properties
        type = "lego";
        object;
        vertices;
    end

    methods
        %% Constructor
        function self = Lego(location,colour)
            self = self@Item(location);
            switch colour
                case 'orange'
                    self.object = PlaceObject('LegoOrange.ply', self.location);
                    self.type = 'orangelego';
                case 'yellow'
                    self.object = PlaceObject('LegoYellow.ply', self.location);
                    self.type = 'yellowlego';
                case 'green'
                    self.object = PlaceObject('LegoGreen.ply', self.location);
                    self.type = 'greenlego';
                otherwise
                    self.object = PlaceObject('LegoBlue.ply', self.location);
            end
            self.vertices = get(self.object, 'Vertices');
        end

        %% Destructor
        function delete(self)
            try delete(self.object); end
        end

    end

end
