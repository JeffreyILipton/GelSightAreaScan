%echoudp('on',4012)
%host
clientip = '128.30.25.105';
%hostip = '128.30.25.104';
clientport = 8866;
hostport = 8844;
u1 = udp(clientip, 'RemotePort',clientport , 'LocalPort', hostport,'DatagramTerminateMode','on');
%u2 = udp(hostport, 'RemotePort',clientport , 'LocalPort', hostport);

%Connect the UDP object to the host.
fopen(u1);
%On host doetom.dhpc, open u2:
%fopen(u2)
%fprintf(u1, 'Ready for data transfer.')

%initialize
 disp('Move in sinusoid');
 amplitude = 600;
 timeTotal = 3; %s
 period = 0.01;
 steps = timeTotal/period;
t = linspace(0,timeTotal,steps);
arg = t*2*pi*1/timeTotal;
poses = amplitude*sin(arg)';
%plot(t,poses)
%positions = repmat(poses,1,numSegments);
positions = zeros(steps,6);
positions(:,3) = poses;
timeTaken = zeros(steps,1);
tstart = tic;
for k = 1:steps
just = tic;
fwrite(u1,positions(k,:),'int16');
timeTaken(k) = toc(just);
pause(period-timeTaken(k))
end
toc(tstart)
mean(timeTaken)
disp('Done Moving')
%res = fread(u1,6);
% end
%fscanf(u2)

fclose(u1);
delete(u1)
clear u1
%fclose(u2)
%delete(u2)
%clear u2
% 
% %Write to the host and read from the host.
% 
% fwrite(u,65:74)
% %A = fread(u,10);
% %A
% %Stop the echo server and disconnect the UDP object from the host.
% 
% echoudp('off')
% fclose(u)