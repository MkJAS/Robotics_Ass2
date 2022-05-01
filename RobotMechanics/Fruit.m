classdef Fruit
    %!Fruit class which stores key Fruit attributes

    properties (Constant)
        height = 0.05;
    end

    properties
        location;
    end

    methods
        %% Constructor
        function self = Fruit(location)
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
