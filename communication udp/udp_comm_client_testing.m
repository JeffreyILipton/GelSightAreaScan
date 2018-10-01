clear all
%echoudp('on',4012)
%host
%clientip = '128.30.25.104';
hostip = '128.30.25.105';
clientport = 8866;
hostport = 8844;
%u1 = udp(clientip, 'RemotePort',clientport , 'LocalPort', hostport);
u2 = udp(hostip, 'RemotePort',hostport , 'LocalPort', clientport);

%Connect the UDP object to the host.
%fopen(u1)
%On host doetom.dhpc, open u2:
fopen(u2);

fc = ForceController(6,true);
fc.start()
%fprintf(u1, 'Ready for data transfer.')
tic
while(toc<20)
%fscanf(u2)
command = fread(u2,6);
fc.
fwrite(u2,command+10);
end
%fclose(u1)
%delete(u1)
%clear u1
fclose(u2)
delete(u2)
clear u2
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