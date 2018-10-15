classdef HSA < handle
    %GELSIGHT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ser1
		device
		mins
		maxs
        channels
    end
    
    methods
        function obj = HSA(port,channels,mins,maxs)
            % Initialize
			obj.device = 12;
			
			obj.mins = mins;
			obj.maxs = maxs;
			obj.channels = channels;
			obj.ser1 = serial(port,...
				'Baudrate',9600, ...
				'DataBits',8, ...
				'Parity','none', ...
				'InputBufferSize',16384);
			%set(ser1, 'InputBufferSize', 2048);
			%set(ser1, 'BaudRate', 9600);
			%set(ser1, 'DataBits', 8);
			%set(ser1, 'Parity', 'none');
			%set(ser1, 'StopBits', 1);
        end
		
		function start(obj)
			fopen(obj.ser1);
		end 
		
		function write(obj, channel, servo_setting)
		    % Format servo command
			lower = bin2dec(regexprep(mat2str(fliplr(bitget(6120, 1:7))), '[^\w'']', ''));
			upper = bin2dec(regexprep(mat2str(fliplr(bitget(servo_setting*4, 8:14))), '[^\w'']', ''));
			% Advanced Serial Protocol
			% 0xAA = 170
			% 4 = action (set target)
			command = [170, obj.device, 4, channel, lower, upper];
			
			% Simple Serial Protocol
			% 0x84 = 132
			%command = [132, channel, lower, upper];
			
			% Send the command
			fwrite(obj.ser1, command);
		end
		
		function val = posToVal(obj, index, pos)
			minv = obj.mins(index);
			maxv = obj.maxs(index);
			val = floor((maxv-minv)*pos+minv);
		end
		
		function setPos(obj,pos)
			for i=1:length(obj.mins)
				val = obj.posToVal(i,pos);
				obj.write(obj.channels(i),val);
			end
		end
		
		function stop(obj)
			fclose(obj.ser1);
		end 
        % Destructor
        function delete(obj)
			delete(obj.ser1);
		end
		
	end
end