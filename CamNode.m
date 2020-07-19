classdef CamNode <handle
    properties
        imgmsg
        imgsubscriber
        formattedimg
    end
    methods
        function obj = CamNode(~)
            obj.imgmsg = [];
            obj.imgsubscriber = [];
            obj.formattedimg = [];
        end
    end
end