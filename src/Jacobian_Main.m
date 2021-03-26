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

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
  part2 = false;
  part3 = false;
  part5 = true;
  part6 = false;
  part7demo = false;
  part7motion = false;
  part8 = false;
  

  dJdata = [];

  if part2
      disp('implement part 2:');
      syms q1 q2 q3
      Jq = pp.jacob3001([q1,q2,q3]);
      disp(Jq);
      part2Done = true;
      disp('part 2 completed');
  end

  if part3
      disp('implement part 3:');
      disp('configuration 1:');
      Jq1 = pp.jacob3001([0,0,90]);%Along z0
      Jq1_3 = [Jq1(1,:); Jq1(2,:); Jq1(3,:)];
      dJ1 = det(Jq1_3); 
      disp(Jq1);
      disp(Jq1_3);
      disp(dJ1);
      disp('configuration 2:');
      Jq2 = pp.jacob3001([0,1,90]);
      Jq2_3 = [Jq2(1,:); Jq2(2,:); Jq2(3,:)];
      dJ2 = det(Jq2_3); 
      disp(Jq2);
      disp(Jq2_3);
      disp(round(dJ2));
      disp('part 3 completed');
      prt3Done = true;
  end

  if part7demo
      disp('Implement Part 7:');
      fprintf('\n')
      pt1 = [2,5,7]; 
      pt2 = [3,8,5]; 
      uv = pp.unit_vector(pt1,pt2);
      disp('Checking for unit_vactor(v1,v2) function:');
      y = ['The unit vector is: ', num2str(uv)];
      disp(y);
      fprintf('\n')
      disp('Checking for scale_vector(martix, scalar) function:');
      velVect = pp.scale_vector(uv, 10);
      disp(velVect);
      disp('Part 7 completed');
      fprintf('\n')
  end
  
  if part8
     measured = [0 0 0; 0 0 0];
     disp(pp.simulated_js(measured, [1 2 3], 0.1))
  end
  
  if part5
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
    vels = [];
    jointAngles = [];
    speed = [];

    for i = 1:size(pts,1)-1
        goal_angles(i,:) = pp.ik3001(pts(i,:));
    end

    
    for i=1:size(pts)-1
        x_coeff = tp.quintic_traj(0,3,0,0,pts(i,1),pts(i+1,1));
        y_coeff = tp.quintic_traj(0,3,0,0,pts(i,2),pts(i+1,2));
        z_coeff = tp.quintic_traj(0,3,0,0,pts(i,3),pts(i+1,3));


        %Calcuating position equations:
        qx = x_coeff(1) + x_coeff(2)*t + x_coeff(3)*(t^2) + x_coeff(4)*(t^3) + x_coeff(5)*(t^4) + x_coeff(6)*(t^5);
        qy = y_coeff(1) + y_coeff(2)*t + y_coeff(3)*(t^2) + y_coeff(4)*(t^3) + y_coeff(5)*(t^4) + y_coeff(6)*(t^5);
        qz = z_coeff(1) + z_coeff(2)*t + z_coeff(3)*(t^2) + z_coeff(4)*(t^3) + z_coeff(5)*(t^4) + z_coeff(6)*(t^5);
    
        x_posTraj = subs(qx,t,int_time);
    y_posTraj = subs(qy,t,int_time);
    z_posTraj = subs(qz,t,int_time);
    
        for j=1:(size(x_posTraj,2))-1
            %disp(x_posTraj);
            packet = [x_posTraj(j),y_posTraj(j),z_posTraj(j)];
    %         disp(packet);
            pose = pp.measured_js(true,false);
            x_pose = pose(1,1);
            y_pose = pose(1,2);
            z_pose = pose(1,3);
            DataP = [DataP; [x_pose y_pose z_pose]];
            angles = pp.ik3001(packet);
            pp.interpolate_jp(angles,5);
            
            plotter.plot_arm_velocity(pp, goal_angles);
            
            data = pp.measured_js(true, true);
            jointAngles = data(1,:);
            qdot = data(2,:);
            jVec = pp.fdk3001(jointAngles, qdot);
            vels = [vels; jVec.'];
            speed = [speed; sqrt(jVec(1,1)^2+jVec(2,1)^2+jVec(3,1)^2)];
            pause(0.1)
        end
    end
      figure(2)
      subplot(3,1,1)
      plot(vels(:,1));
      hold on
      plot(vels(:,2));
      plot(vels(:,3));
      title("Velocity vs Timesteps")
      hold off
      legend('vx', 'vy', 'vz');
      
      subplot(3,1,2)
      plot(vels(:,4));
      hold on
      plot(vels(:,5));
      plot(vels(:,6));
      title("Angular Velocity vs Timesteps")
      hold off
      legend('wx', 'wy', 'wz');
      
      subplot(3,1,3)
      plot(speed(:));
      title("Speed vs Timesteps")
    
    figure(6)
    plot3(DataP(:,1),DataP(:,2),DataP(:,3))
  end
  
  if part7motion || part8
    % Creating set points:
    P3 = [90 -70 50];
    P2 = [90 70 50];
    P1 = [160 0 155];
    pts = [P1;P2;P3;P1];% Looping back to P1
    
    timeStep = 0.01;
    
    velocity_const = 100000;
    measured = [];
    
    for i = 1:size(pts,1)-1
        goal_angles(i,:) = pp.ik3001(pts(i,:));
    end
    
    simulatedMeasured = [pp.ik3001(P3);0 0 0];
    DataP = []; DataV=[];
    
    for i=1:size(pts)-1
%         if part7motion
        goal = pts(i, :);
%         end
%         if part8
%            plotter.plot_input(pp);
%            [x,z] = ginput(1);
%            goal = [x 0 z];
%         end
       
       for j = 1:100
           location = [];
          if part8
              measured = simulatedMeasured;
              fk = pp.fk3001(measured(1,:));
              location = [fk(1,4), fk(2,4),fk(3,4)];
          end
          if part7motion
             measured = pp.measured_js(true, true);
             location = pp.position_cp();
          end
          DataP = [DataP; location];
          DataV = [DataV; [measured(2,1), measured(2,2),measured(2,3)]];
          unit_vec = pp.unit_vector(location, goal);
          distance = sqrt((location(1)-goal(1))^2 + (location(2)-goal(2))^2 + (location(3)-goal(3))^2);
          speed = velocity_const/50 * distance; %add distance for p control
          if speed > velocity_const
              speed = velocity_const;
          end
          velocity_vec = pp.scale_vector(unit_vec, speed);
          invJ = pp.idk3001(measured(1,:));
          jointVel = pp.ivk3001(invJ, velocity_vec);
%           jointVel = [jointVel(3); jointVel(2); jointVel(1)];
          nextPos = pp.simulated_js(measured, jointVel.', timeStep);
%           disp("Location")
%           disp(location);
%           disp("Goal")
%           disp(goal);
%           disp("Velocity Vec")
%           disp(velocity_vec);
%           disp("Joint Vel")
%           disp(jointVel)
%           disp("Pos")
%           disp(measured)
%           disp("Next pos")
%           disp(nextPos);
          if part8
              hold on
             simulatedMeasured = nextPos; 
             plotter.plot_arm_triangle_measured(pp, goal_angles, measured);
             pause(timeStep)
          end
          if part7motion
              pp.interpolate_jp(nextPos(1,:), timeStep);
              plotter.plot_arm_triangle(pp, goal_angles);
              pause(timeStep);
          end
          
       end
       disp("Goal")
       disp(goal)
       disp("Actual")
       if part7motion
           disp(pp.position_cp())
       end
       if part8
           fk = pp.fk3001(measured(1,:));
           disp([fk(1,4), fk(2,4),fk(3,4)])
       end
    end
    hold off
    
    figure(3)
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
    plot(diff(DataV(:,1)))
    hold on 
    plot(diff(DataV(:,2)))
    plot(diff(DataV(:,3)))
    
    
    figure(5)
    title("Motion of arm in 3D task space")
    plot3(DataP(:,1), DataP(:,2), DataP(:,3));
    
  end
    
  if part6
     goal = [0 45 -90];
     
     dJdata = [];
     DataP = [];

     for i = 1:0.1:45
        Jq = pp.jacob3001(goal);
        Jq_3 = [Jq(1,:); Jq(2,:); Jq(3,:)];
        dJ = det(Jq_3); 
        disp(goal);
        disp(dJ);

        pp.interpolate_jp(goal,0.1);
         data = dJ;
        dJdata = [dJdata; data];
        
          DataP = [DataP; pp.position_cp()];
%         disp(dJdata);
        if (dJ >= -100) && (dJ <= 100)

            disp('Error');
            error("Reaching singularity");
        else
            goal = [0,45+i,-90];

        end
        plotter.plot_arm(pp);
        pause(0.0001)
     end

     
     figure(6);
     plot(1:0.1:45, dJdata);
     title("Determinant vs Angle Increment")
     figure(7)
    plot3(DataP(:,1), DataP(:,2), DataP(:,3));
    title("Motion of arm in 3D task space")
     axis([-2 160; -150 160; -2 195])
     
  end

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()