classdef Body < handle
    %BODY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=public)
        v
        q
        v_offset
    end
    
    methods
        function obj = Body(offset)
            %BODY Construct an instance of this class
            %   Detailed explanation goes here
            obj.v_offset = offset;
            obj.v = [0,0,0];
            obj.q = [0,0,0,0];
        end
        
        function curpos,quat = getPostion(obj);
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
           curpos = v+v_offset;
           quat = q;
        end
    end
end

