hudps = dsp.UDPSender('RemoteIPPort' ,1505); hudps. RemoteIPAddress = '127.0.0.1';

q = [0,0,pi,pi/3,0,pi/2,0]';

for t=1:0.1:30 disp(t); step(hudps ,[q;q]); pause (1); end