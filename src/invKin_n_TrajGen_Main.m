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
  DEBUG   = false;          % enables/disables debug prints 
 
% Creating set points:
P1 = [90 -70 50];
P2 = [90 70 50];
P3 = [160 0 155];
pts = [P1;P2;P3;P1];% Looping back to P1

tp = Traj_Planner();
t_steps = 30;
syms t;
%t_step_int = 0.2;
int_time = 0:0.1:3;
DataP = [];
DataV = [];
DataA = [];
prevX = 0;
currX = 0;
prevY = 0;
currY = 0;
prevZ = 0;
currZ = 0;
prevVelX = 0;
currVelX = 0;
prevVelY = 0;
currVelY = 0;
prevVelZ = 0;
currVelZ = 0;
posList = [];
jointAngles = [];
goal_location = [90 -70 50; 90 70 50; 160 0 155];
goal_angles = [];
for i = 1:size(goal_location,1)
    goal_angles(i,:) = pp.ik3001(goal_location(i,:));
    disp(goal_angles(i,:));
end

init_angles = pp.ik3001(P1);
pp.interpolate_jp(init_angles,3);
for i=1:size(pts)-1
    
    x_coeff = tp.quintic_traj(0,3,0,0,pts(i,1),pts(i+1,1));
    y_coeff = tp.quintic_traj(0,3,0,0,pts(i,2),pts(i+1,2));
    z_coeff = tp.quintic_traj(0,3,0,0,pts(i,3),pts(i+1,3));
    
    
    %Calcuating position equations:
    qx = x_coeff(1) + x_coeff(2)*t + x_coeff(3)*(t^2) + x_coeff(4)*(t^3) + x_coeff(5)*(t^4) + x_coeff(6)*(t^5);
    qy = y_coeff(1) + y_coeff(2)*t + y_coeff(3)*(t^2) + y_coeff(4)*(t^3) + y_coeff(5)*(t^4) + y_coeff(6)*(t^5);
    qz = z_coeff(1) + z_coeff(2)*t + z_coeff(3)*(t^2) + z_coeff(4)*(t^3) + z_coeff(5)*(t^4) + z_coeff(6)*(t^5);
    
    %Calculating velocity equations:
    vx = x_coeff(2) + 2*x_coeff(3)*t + 3*x_coeff(4)*(t^2) + 4*x_coeff(5)*(t^3) + 5*x_coeff(6)*(t^4);
    vy = y_coeff(2) + 2*y_coeff(3)*t + 3*y_coeff(4)*(t^2) + 4*y_coeff(5)*(t^3) + 5*y_coeff(6)*(t^4);
    vz = z_coeff(2) + 2*z_coeff(3)*t + 3*z_coeff(4)*(t^2) + 4*z_coeff(5)*(t^3) + 5*z_coeff(6)*(t^4);
    
    %Calculating acceleration equations:
    ax = 2*x_coeff(3) + 6*x_coeff(4)*t + 12*x_coeff(5)*(t^2) + 20*x_coeff(6)*(t^3);
    ay = 2*y_coeff(3) + 6*y_coeff(4)*t + 12*y_coeff(5)*(t^2) + 20*y_coeff(6)*(t^3);
    az = 2*z_coeff(3) + 6*z_coeff(4)*t + 12*z_coeff(5)*(t^2) + 20*z_coeff(6)*(t^3);
    
    x_posTraj = subs(qx,t,int_time);
    y_posTraj = subs(qy,t,int_time);
    z_posTraj = subs(qz,t,int_time);
%     disp(x_posTraj);

    x_velTraj = subs(vx,t,int_time);
    y_velTraj = subs(vy,t,int_time);
    z_velTraj = subs(vz,t,int_time);
    
    x_accTraj = subs(ax,t,int_time);
    y_accTraj = subs(ay,t,int_time);
    z_accTraj = subs(az,t,int_time);
    
    sizeArr = size(x_posTraj);
%     disp(sizeArr(2));
%     disp(y_posTraj);
%     disp(z_posTraj);

    for j=1:(sizeArr(2))-1
        %disp(x_posTraj);
        packet = [x_posTraj(j),y_posTraj(j),z_posTraj(j)];
%         disp(packet);
        pose = pp.measured_js(true,false);
         x_pose = pose(1,1);
         y_pose = pose(1,2);
         z_pose = pose(1,3);
        DataP = [DataP; [x_pose y_pose z_pose]];
        
        currX = x_pose;
        currY = y_pose;
        currZ = z_pose;
        deltaX = currX - prevX;
        deltaY = currY - prevY;
        deltaZ = currZ - prevZ;
        velX = deltaX/0.1;
        velY = deltaY/0.1;
        velZ = deltaZ/0.1;
        DataV = [DataV; [velX velY velZ]];
        prevX = currX;
        prevY = currY;
        prevZ = currZ;
        
        currVelX = velX;
        currVelY = velY;
        currVelZ = velZ;
        deltaVx = currVelX - prevVelX;
        deltaVy = currVelY - prevVelY;
        deltaVz = currVelZ - prevVelZ;
        accX = deltaVx/0.1;
        accY = deltaVy/0.1;
        accZ = deltaVz/0.1;
        DataA = [DataA; [accX accY accZ]];
        prevVelX = currVelX;
        prevVelY = currVelY;
        prevVelZ = currVelZ;
        
%           disp(DataP);
%         y_DataP = pose(2);
%         z_DataP = pose(3);
%         DataV = [DataV; [vel(1) vel(2) vel(3)]];
%         y_DataV = vel(2);
%         z_DataV = vel(3);
        angles = pp.ik3001(packet);
        %disp(angles);
        
        pp.interpolate_jp(angles,5);
        
        
        plotter.plot_arm_triangle(pp, goal_angles);
        transMatrix = pp.measured_cp();
        posList = [posList; [transMatrix(1,4) transMatrix(2,4) transMatrix(3,4)]]; 
        angles = pp.measured_js(true, false);
        jointAngles = [jointAngles; [angles(1,1), angles(1,2), angles(1,3)]];
        
    end
    
%     
%     subplot(3,1,3)
%     plot(time,x_accTraj)
%     hold on
%     plot(time,y_accTraj)
%     hold on
%     plot(time,z_accTraj)
%     title("Acceleration vs Time")
%     hold off
    
    
%     disp(x_posTraj);
%     disp(y_posTraj);
%     disp(z_posTraj);
%     angles = pp.ik3001([x_posTraj, y_posTraj,z_posTraj]);
%     
%     pp.servo_jp(angles);
end

% % writematrix(DataP, "Lab3_quintic_position_data.csv");
% % writematrix(DataV, "Lab3_quintic_velocity_data.csv");
% % writematrix(DataA, "Lab3_quintic_acceleration_data.csv");
    
    figure(3)
    disp(DataP);
    time = 0:0.1:2;
    subplot (3,1,1)
    plot(DataP(:,1))
    hold on
    plot(DataP(:,2))
    hold on 
    plot(DataP(:,3))
    title("Position vs Time")
    hold off
    
    subplot(3,1,2)
    plot(DataV(:,1))
    hold on 
    plot(DataV(:,2))
    hold on
    plot(DataV(:,3))
    title("Velocity vs Time")
    hold off
    
    subplot(3,1,3)
    plot(DataA(:,1))
    hold on 
    plot(DataA(:,2))
    %%plots for part 3
    figure(5)
    title("Motion of arm in 3D task space")
    plot3(posList(:,1), posList(:,2), posList(:,3));
    
    figure(2)
    title("X.Y,Z during motion")
    plot(posList(:,1))
    hold on
    plot(posList(:,2))
    plot(posList(:,3))
    hold off
    legend('X', 'Y', 'Z');
    
    figure(4)
    title("Joint Angles during motion")
    plot(jointAngles(:,1))
    hold on
    plot(DataA(:,3))
    title("Acceleration vs Time")
    hold off
    
    figure
    plot3(DataP(:,1),DataP(:,2),DataP(:,3))
% 
% 
% %     disp(pp.fk3001([0,0,0]));
%     disp(pp.ik3001([100,0,195]));
% %     disp(pp.fk3001([-90,90,45]));
%     disp(pp.ik3001([0,-29,24]));
% %     disp(pp.fk3001([0,0,30]));
%     disp(pp.ik3001([86.6025,0,145])); 
% %     disp(pp.fk3001([25,-30,65]));
% %     disp(pp.ik3001([28.9250,13.4879,124.2449]));
%     %outside workspace
% %     disp(pp.ik3001([190, 100, 100])); 
%     %unreachable angle
% %     disp(pp.ik3001([0,0, 0]));
%     
%     %use this to debug location and estimates
%     while DEBUG
%        plotter.plot_arm(pp);
%        currentLocation = pp.measured_cp();
%        angles = pp.measured_js(true, false);
%        disp("Current angles");
%        disp(angles(1,:)); 
%        disp("Current location");
%        disp([currentLocation(1,4) currentLocation(2,4) currentLocation(3,4)]);
%        disp("Current IK angles");
%        ik = pp.ik3001([currentLocation(1,4) currentLocation(2,4) currentLocation(3,4)]);
%        disp(ik);
%        estimate = pp.fk3001(ik);
%        disp("IK Location Estimate");
%        disp([estimate(1,4) estimate(2,4) estimate(3,4)]);
%        pause(0.25);
%     end
%     
%     posList = [];
%     jointAngles = [];
%     goal_location = [90 -70 50; 90 70 50; 160 0 155];
%     goal_angles = [];
%     for i = 1:size(goal_location,1)
%         goal_angles(i,:) = pp.ik3001(goal_location(i,:));
%         disp(goal_angles(i,:));
%     end
%     
%     for i = 1:size(goal_location,1)
%         pp.interpolate_jp(goal_angles(i,:),2000);
%         t0 = clock;
%         while etime(clock, t0) < 2
%             plotter.plot_arm_triangle(pp, goal_angles);
%             transMatrix = pp.measured_cp();
%             posList = [posList; [transMatrix(1,4) transMatrix(2,4) transMatrix(3,4)]]; 
%             angles = pp.measured_js(true, false);
%             jointAngles = [jointAngles; [angles(1,1), angles(1,2), angles(1,3)]];
%             pause(0.05);
%         end
%         transMatrix = pp.measured_cp();
%         disp([transMatrix(1,4) transMatrix(2,4) transMatrix(3,4)]); 
%         pause(1);
%     end
%     
%     figure(3)
%     plot3(posList(:,1), posList(:,2), posList(:,3));
%     figure(4)
%     plot(jointAngles(:,1))
%     hold on
%     plot(jointAngles(:,2))
%     plot(jointAngles(:,3))
%     hold off
%     legend('Joint 1', 'Joint 2', 'Joint 3');
      
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

% Clear up mem