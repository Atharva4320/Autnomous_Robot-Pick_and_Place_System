%%
% RBE3001 - Laboratory 1 
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

%syms theta d a alpha

%disp(pp.dh2mat([theta,d,a,alpha]));

%disp(pp.dh2mat([0,0,0,0])); % With all the parameters as 0, the transformation
                            % matrix should return Identity matrix


%disp(pp.measured_cp());
% scatter3...cl

%plotter.plot_arm(pp);
%   Initializing variables:
%     posList = [];
%     sumPosX = 0;
%     sumPosY = 0;
%     sumPosZ = 0;
%     posX = 0;
%     posY =0;
%     posZ = 0;
%     diffPosX = 0;
%     diffPosY = 0;
%     diffPosZ = 0;
%     sumDiffX = 0;
%     sumDiffY = 0;
%     sumDiffZ = 0;
%     rms_X = 0;
%     rms_Y = 0;
%     rms_Z = 0;
%     rmsVals =[];
%     avgX = 0;
%     avgY = 0;
%     avgZ = 0;
%     avgPos = [];
%     
%     pp.interpolate_jp([0,0,0],2000); %Start at zero position
%     
%     pause(2);
%     
%     for i = 1:10
%         pp.interpolate_jp([30,20,50],2000);
%         pause(2);
%         pp.interpolate_jp([0,0,0], 2000);
%         pause(2);
%         transMatrix = pp.measured_cp();
% 
%         posX = transMatrix(1,4);
%         diffPosX = posX - 100;
%         sumDiffX = sumDiffX + (diffPosX)^2;
% 
%         
%         posY = transMatrix(2,4);
%         diffPosY = posY - 0;
%         sumDiffY = sumDiffY + (diffPosY)^2;
% 
%         
%         posZ = transMatrix(3,4);
%         diffPosZ = posZ - 195;
%         sumDiffZ = sumDiffZ + (diffPosZ)^2;
% 
%                 
%         sumPosX = sumPosX + transMatrix(1,4);
%         sumPosY = sumPosY + transMatrix (2,4);
%         sumPosZ = sumPosZ + transMatrix(3,4);
%         posList = [posList; [transMatrix(1,4) transMatrix(2,4) transMatrix(3,4)]]; 
%     end
%     
%     avgX = sumPosX/10;
%     avgY = sumPosY/10;
%     avgZ = sumPosZ/10;
%     
%     avgPos = [avgX, avgY, avgZ]; % Calculates the average tip position
%     % The actual calculated tip position is [100,0,195]
%     
%     disp('The average tip position is:')
%     disp(avgPos)
%     writematrix(posList, "lab2_position_data.csv");
%     
%     rms_X = sqrt(sumDiffX/10);
%     rms_Y = sqrt(sumDiffY/10);
%     rms_Z = sqrt(sumDiffZ/10);
%     
%     rmsVals = [rms_X, rms_Y, rms_Z];
%     disp('The rms value is:');
%     disp(rmsVals);

disp(pp.fk3001([25,-30,65]));
% disp(pp.measured_js(true,false));
% pp.interpolate_jp([0,0,0], 2000);
% pause(2);
% pp.interpolate_jp([-90,90,45], 2000);
    
%     % Add plot
%     Array = csvread('lab2_position_data.csv');
%     col1 = Array(:, 1);
%     col2 = Array(:, 2);
%     col3 = Array(:, 3);
%     scatter3(col1,col2,col3);
                            
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()