%%
% RBE3001 - Laboratory 1 end
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 
plotter = Plot();
try
    SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
    SERVER_ID_READ =1910;% ID of the read packet
    DEBUG   = true;          % enables/disables debug prints
    
    pp.interpolate_jp([45,0,0],1000);
    
    %You will need to quit manually
%     while true 
%     plotter.plot_arm(pp);
%             pause(0.05);
%     end
    
    goal = [45 0 0];% 45 45 0; 30 20 0; 30 20 30; 30 0 0; 0 0 0];
    
    %Will run on its own
    for i = 1:1
        disp("Interpolate to:");
        disp(goal(i,:));
        pp.interpolate_jp(goal(i,:),1500);
        t0 = clock;
        while etime(clock, t0) < 2
            plotter.plot_arm(pp);
            pause(0.05);
        end
        disp('The Transformation matrix is:');
        disp(pp.measured_cp());
    end
    pp.interpolate_jp([0,0,0],1000);


catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()