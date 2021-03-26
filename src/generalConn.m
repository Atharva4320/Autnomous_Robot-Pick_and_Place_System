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
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');


  
  % The following code executes functions servo_jp and interpolate_jp
  % respectively with a 2 second interval in between.
  
    pp.interpolate_jp([0,0,0],1000);

    pause(2);
    
    pp.interpolate_jp([60,0,0],3000);
    %pp.servo_jp([60,0,0]);
    
    t0 = clock;
    motor1 = [];
    motor2 = [];
    motor3 = [];
    posList = [];
    timeList = [];
    lastTime = clock;
    
    while etime(clock, t0) < 3
      data = pp.measured_js(true, false);
      motor1 = [motor1; data(1,1)];
      motor2 = [motor2; data(1,2)];
      motor3 = [motor3; data(1,3)];
      posList = [posList; [data(1,1) data(1,2) data(1,3)]];
      deltaT = round(etime(clock, lastTime) * 1000);
      timeList = [timeList; deltaT];
      lastTime = clock;
    end
    
    writematrix(timeList, "lab1_time_data.csv");
    writematrix(posList, "lab1_position_data.csv");
    
    disp("mean");
    disp(mean(timeList));
    disp("median");
    disp( median(timeList));
    disp("maximum");
    disp( max(timeList));
    disp("minimum");
    disp( min(timeList));
    

    %%Add plot
    figure
    plot(motor1);
    title('Line Plot of Motor 1');
    xlabel('time');
    ylabel('Motor 1 Values (degrees)');
    ylim([-90 90]);
    
    
    figure
    plot(motor2);
    title('Line Plot of Motor 2');
    xlabel('time');
    ylabel('Motor 2 Values (degrees)');
    ylim([-90 90]);
    
    
    figure
    plot(motor3);
    title('Line Plot of Motor 3');
    xlabel('time');
    ylabel('Motor 3 Values (degrees)');
    ylim([-90 90]);
    
    %%Add histogram
    figure
    histogram(timeList);
    title('Histogram of Time Step')
    xlabel('Time Step (Milliseconds)')
    ylabel('Occurances')
    
    
    %pp.read reads a returned 15 float backet from the micro controller.
    measuredValues = pp.measured_js(true, true);
    disp('Measured Pos/Vel:');
    disp(measuredValues);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()