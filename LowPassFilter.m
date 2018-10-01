classdef LowPassFilter < handle
    %LOWPASSFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    properties(Access=private)
		cutOffFrequency
		samplePeriod
		lastState
		lastStateDerivative
		stateDerivative
        wT
        firstTime
    end
    
    methods(Access = public)
        % Constructor
        function obj = LowPassFilter(omega, T, dimensions)
            obj.cutOffFrequency = omega;
            obj.samplePeriod = T;
            obj.lastState = zeros(1,dimensions);
            obj.lastStateDerivative = zeros(1,dimensions);
            obj.stateDerivative = zeros(1,dimensions);
            obj.wT = 0;
            obj.setFilterParameters(omega,T);
            obj.firstTime = true;
        end
        
        function setFilterParameters(obj, omega, T)
            obj.cutOffFrequency = omega;
            obj.samplePeriod = T;
            obj.wT = obj.cutOffFrequency * obj.samplePeriod;
        end
        function stateDerivative = calculate(obj, currentState)
            if(obj.firstTime)
                obj.firstTime = false;
                obj.stateDerivative = 0*currentState;
            else 
            obj.stateDerivative = (2 - obj.wT) / (2 + obj.wT) * obj.lastStateDerivative...
                + (2*obj.cutOffFrequency) / (2 + obj.wT) * (currentState - obj.lastState);
            end
            obj.lastState = currentState;
            obj.lastStateDerivative = obj.stateDerivative;
            stateDerivative = obj.stateDerivative;
        end
        
        % Destructor
        function delete(obj)
           
        end
    end
    methods(Access = private)
        
        
    end
    
    methods(Static)
        
    end
    
end



