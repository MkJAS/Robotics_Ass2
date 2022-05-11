classdef Item
    %!Item class which stores key Fruit attributes

    properties (Constant)
        % height = 0.03;
    end

    properties
        location;
    end

    methods
        %% Constructor
        function self = Item(location)
            self.location = location;
        end

        %% Destructor
        function delete(self)
        end

        % %% Location setter
        % function self = set.location(self, newLocation)
        %     self.location = newLocation;
        % end

    end

end
