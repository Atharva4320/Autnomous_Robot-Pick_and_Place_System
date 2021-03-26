classdef Robot
    properties
        %hidDevice;
        %hidService;
        myHIDSimplePacketComs
        pol 
        SERVO_READ_POS = 1910;
        SERVO_READ_VEL = 1822;
        SERV_ID = 1848;
        GRIPPER_ID = 1962;
        setpoints = [0 0 0];
    end
    methods
	%The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
	    %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
             packet.myHIDSimplePacketComs=dev; 
            packet.pol = java.lang.Boolean(false);
        end
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        function  write(packet, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);anglesanglesanglesaanglesanglesanglesanglesngles
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        %This function takes two Boolean values named GETPOS and GETVEL
        %and returns a 2x3 array which contains current joint positionsin 
        %degrees (1strow) and current joint velocities (2ndrow)
        function ret = measured_js(obj, GETPOS, GETVEL)
            try
               ret = [0 0 0; 0 0 0];
               if GETPOS
                   pos = obj.read(obj.SERVO_READ_POS);
                   ret(1,1) = pos(3);
                   ret(1,2) = pos(5);
                   ret(1,3) = pos(7);
               end
               if GETVEL
                   vel = obj.read(obj.SERVO_READ_VEL);
                   ret(2,1) = vel(3);
                   ret(2,2) = vel(6);
                   ret(2,3) = vel(9);
               end
            catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
            end
        end
        %This function takes in a 1x3 array of command joint values (in
        %degrees) and sends directly to the actuators, bypassing the
        %interpolation
        function  apacket = servo_jp(obj, jointVals)
                try
                   obj.setpoints = jointVals;
                   apacket = zeros(15, 1, 'single');
                   apacket(1) = 1000;%one second time
                   apacket(2) = 0;%linear interpolation
                   apacket(3) = jointVals(1);% First value of array to motor1
                   apacket(4) = jointVals(2);% Second value of array to motor2
                   apacket(5) = jointVals(3);% Third value of array to motor3

                  % Send packet to the server and get the response      
                  %pp.write sends a 15 float packet to the micro controller
                   obj.write(obj.SERV_ID, apacket); 

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        %This function takes a 1x3 array of commanded end of the motion
        %joint values and an interpolation time (in ms) to get there
        function interpolate_jp(obj, posVals, interTime)
                try
                   obj.setpoints = posVals;
                   aPacket = zeros(15, 1, 'single');
                   aPacket(1) = interTime;%one second time
                   aPacket(2) = 1;% sinusoidal interpolation
                   aPacket(3) = posVals(1); % First value of array to motor1
                   aPacket(4) = posVals(2);% Second value of array to motor2
                   aPacket(5) = posVals(3);% Third value of array to motor3

                  % Send packet to the server and get the response      
                  %pp.write sends a 15 float packet to the micro controller
                   obj.write(obj.SERV_ID, aPacket); 

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        %This function sets the value of the gripper
        function setGripper(obj, gripper_val)
                try
                    ds = javaArray('java.lang.Byte',15);
                    ds(1) = java.lang.Byte(gripper_val);
                    for i=2:15
                        ds(i)= java.lang.Byte(0);
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(obj.GRIPPER_ID);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);anglesanglesanglesaanglesanglesanglesanglesngles
                    obj.myHIDSimplePacketComs.writeBytes(intid,  ds, obj.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        %This function closes the gripper
        function closeGripper(obj)
                obj.setGripper(0);
        end
        %This function opens the gripper
        function openGripper(obj)
                obj.setGripper(180);
        end
        %This function returns a 1x3 array which contains commanded end of 
        %motion joint set point positions in degrees`
        function ret = goal_js(obj)
            ret = obj.setpoints;
        end
        %This function returns a 1x3 array which contains current joint 
        %set point positions in degrees
        function setpoints = setpoint_js(obj)
                try
                   pos = obj.read(obj.SERVO_READ_POS);
                   setpoints(1) = pos(2);
                   setpoints(2) = pos(4);
                   setpoints(3) = pos(6);
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        %This function takes in a 1x4 array corresponding to a row of the
        %DH parameter table for a given link, generates the intermediate
        %transformation and returns a corresponding symbolic 4x4 homogeneous
        %transformation matrix
        %The DH parameter is in the order of :- [theta, d, a, alpha]
        function transMatrix = dh2mat(obj, DHparam)
                transMatrix = [cosd(DHparam(1)), -sind(DHparam(1))*cosd(DHparam(4)), sind(DHparam(1))*sind(DHparam(4)), (DHparam(3))*cosd(DHparam(1));
                               sind(DHparam(1)), cosd(DHparam(1))*cosd(DHparam(4)), -cosd(DHparam(1))*sind(DHparam(4)), (DHparam(3))*sind(DHparam(1));
                                             0,                 sind(DHparam(4)),                  cosd(DHparam(4)),                 (DHparam(2));
                                             0,                               0,                                0,                           1];
            
        end
        %this method takes in an nx4 array corresponding to the n rows of 
        %the full DH parameter table. It then generates a corresponding 
        %symbolic 4x4 homogeneous transformation matrix for the composite transformation.
        function transMatrix = dh2fk(obj, dhParams)
           transMatrix = eye(4);
           for i = 1:size(dhParams,1)
               transMatrix = transMatrix * obj.dh2mat(dhParams(i,:));
           end
        end
        %This method takes n joint configurations as inputs in the form 
        %of an nx1 vector (i.e. your three joint angles
        %It returns a 4x4 homogeneous transformation matrix representing 
        %the position and orientation of the tip frame with 
        %respect to the base frame
        %Uses degrees and milimeters as units
        function TBaseTip = fk3001(obj, jointAngles)
           dhParams = [0 95 0 -90; -90 0 100 0; 90 0 100  0];
           TBaseTip = eye(4);
           for i = 1:size(jointAngles,2)
               dhParam = [jointAngles(i) 0 0 0];
               if i < size(dhParams,2)
                   dhParam = dhParam + dhParams(i,:);
               end
               TBaseTip = TBaseTip * obj.dh2mat(dhParam);
           end
        end
        %This function takes data from measured_js() and returns a 
        %4x4 homogeneous transformation matrix based upon the current joint 
        %positions in degrees
        function TBaseTip = measured_cp(obj)
            data = obj.measured_js(true, false);
            jointAngles = data(1,:);
            TBaseTip = obj.fk3001(jointAngles);
        end
        %This function takes data from setpoint_js() and returns a 4x4
        %homogenous transformation matrix based upon the current joint set
        %values.
        %Uses degrees as units
        function TBaseTip = setpoint_cp(obj) 
            data = obj.setpoint_js();
            TBaseTip = obj.fk3001(data);
        end
        % This function takes data from goal_js and returns a 4x4
        % transformation matrix based upon the commanded end of motion
        % joint set point positions in degrees.
        function goalMatrix = goal_cp(obj)
            data = obj.goal_js();
            goalMatrix = obj.fk3001(data);
        end
        %This function takes a 3x1 task space position vector as the input
        %and returns a set of corresponding joint angles that would make
        %the robot's end-effector move to that target position.
        
        function angles = ik3001(obj, PBaseTip)
            basicWorkspace = [-2 160; -150 160; -2 300];
            if(~(PBaseTip(1) >= basicWorkspace(1,1) && PBaseTip(1) <= basicWorkspace(1,2) && ...
                PBaseTip(2) >= basicWorkspace(2,1) && PBaseTip(2) <= basicWorkspace(2,2) && ...
                PBaseTip(3) >= basicWorkspace(3,1) && PBaseTip(3) <= basicWorkspace(3,2)))
                error("Outside reachable workspace\nInputted coordinates were:\t%f\t%f\t%f\n"+...
                    "Allowed dimensions are:\t%d to %d\t%d to %d\t%d to %d", ...
                    PBaseTip(1), PBaseTip(2), PBaseTip(3),basicWorkspace(1,1),basicWorkspace(1,2),...
                    basicWorkspace(2,1),basicWorkspace(2,2),...
                    basicWorkspace(3,1),basicWorkspace(3,2));
            end
            
            L0 = 55; %the length of link 0
            L1 = 40; %the length of link 1
            L2 = 100; %the length of link 2
            L3 = 100; %the lenght of link 3

            q1 =  round (atan2d (PBaseTip(2),(PBaseTip(1)))); %calculating the value of q1

            d = PBaseTip(3) - L1 - L0; % the perpendicular distance between the end-effector and the horizontal x-y plane
            b = sqrt(PBaseTip(1)^2 + PBaseTip(2)^2); % general length of a point on x-y plane from the origin
            A = sqrt(b^2 + d^2); % calculting the variable length A from the end-effector to frame 1

            beta2 = atan2d(d,b); % angle between A and the x-y plane 
            %--------------------------------------------
            %To convert acos to atan2
            
            cosa2 = ((A)^2 + (L2)^2 - (L3)^2)/(2*A*L2); % from law of cosines
            cosa2_Sq = (cosa2)^2; % calculating the square of cosine
            sina2 =  sqrt(1-cosa2_Sq); % from the trigonometric identity
            
            alpha2 = atan2d(sina2,cosa2); % calculating the angle between L2 and A
            
             q2 = round (90 - (alpha2 + beta2)); % substituting the values to find q2
            %---------------------------------------------
            %To convert acos to atan2
            
            cosa3 = ((L3)^2 + (L2)^2 - (A)^2)/(2*L3*L2); % from law of cosines
            cosa3_Sq = (cosa3)^2; % calculating the square of cosine
            sina3 = sqrt(1-cosa3_Sq); % from the trigonometric identity
            
            alpha3 = atan2d(sina3,cosa3); % calculating the abgle between L3 and L2

            jointRange = [-92 92; -10 104; -85 68];
            q3 = round (90 - alpha3); % substituting the values to find q3
            %------------------------------------------------
            if(q1 >= jointRange(1,1) && q1 <= jointRange(1,2) && ...
                q2 >= jointRange(2,1) && q2 <= jointRange(2,2) && ...
                q3 >= jointRange(3,1) && q3 <= jointRange(3,2))

                angles = [q1, q2, q3];
            else 
                error("Outside reachable workspace\nCalculated angles were:\t%f\t%f\t%f\n"+...
                    "Allowed angles are:\t%d to %d\t%d to %d\t%d to %d", ...
                    q1, q2, q3,jointRange(1,1),jointRange(1,2),...
                    jointRange(2,1),jointRange(2,2),...
                    jointRange(3,1),jointRange(3,2));
            end
        end
       %The function takes the configuration "q" (i.e. all the current
       %joint angles at the time the function runs) a 1x3 array and returns
       %the corresponding numeric 6x3 Jacobian matrix
       function jMat = jacob3001(obj, q)
          jMat = [ -100*cosd(q(2) + q(3))*sind(q(1)), -100*cosd(q(1))*sind(q(2) + q(3)) + cosd(q(2)), -100*cosd(q(1))*sind(q(2) + q(3));
                    100*cosd(q(1))*cosd(q(2) + q(3)), -100*sind(q(1))*sind(q(2) + q(3)) + cosd(q(2)), -100*sind(q(1))*sind(q(2) + q(3));
                                                 0, -100*sind(q(2)) - 100*cosd(q(2) + q(3)), -100*cosd(q(2) + q(3));
                                                 0, -sind(q(1)), -sind(q(1));
                                                 0, cosd(q(1)), cosd(q(1));
                                                 1, 0, 0];
       end
       %The method takes your configuration q(i.e. all of the current joint
       %angles at the time the function is run) and the vector of
       %instantaneous joint velocities 𝒒̇ as inputs.
       %It should return the 6x1 vector including the task-space linear
       %velocities 𝒑̇ and angular velocities 𝝎̇.
       function jVec = fdk3001(obj, q, qdot)
          jMat = obj.jacob3001(q);
          jVel = qdot.';
          jVec = jMat*jVel;
       end
       %This function takes a 1x3 array of joint angles 
       %and outputs the inverse  of the 3x3 jacobian matrix 
       function invJ = idk3001(obj,measuredPos)
           jacob = obj.jacob3001(measuredPos);
           jacob3 = [jacob(1,:); jacob(2,:); jacob(3,:)];
           invJ = pinv(jacob3);                
       end
       
       %This function takes the inverse jacobian matrix and the
       %instanteneous velocity vector as the inputs and outputs the desired
       %instanteneous joint velocities
       function jointVel = ivk3001(obj, invJ, x_dot)
           jointVel = invJ * x_dot;
       end
       %This function simulates the motion for the robot. It takes in a
       %simulated 2x3 matrix of the joint position and velocity, the 1x3 of
       %the angular velocity, and timestep. It returns the new joint
       %position and velocity
       function newMeasured = simulated_js(obj, measured, angular_vel, timestep)
          newMeasured(1,:) = measured(1,:)+angular_vel.*timestep;
          newMeasured(2,:) = angular_vel;
       end
%--------------------------------------------------Part 7  and 8 Helper Functions--------------------------------------------------------

       %This function takes two 1x3 array corresponding to two points and
       %outputs a unit vector
       function uVect = unit_vector(obj, v1, v2)
             x = v2(1) - v1(1);
             y = v2(2) - v1(2);
             z = v2(3) - v1(3);
             mag = sqrt((x)^2 + (y)^2 + (z)^2);
             uX = x/mag;
             uY = y/mag;
             uZ = z/mag;
             uVect = [uX, uY, uZ];
       end

       %This function utilizes measured_cp() to generate a 4x4 homogeneous
       %transformtion matrix and returns the first three elements of the 4th
       %column in a 1x3 array
       function pos = position_cp (obj)
           data = obj.measured_cp();
           xVal = data(1,4);
           yVal = data(2,4);
           zVal = data(3,4);
           pos = [xVal, yVal, zVal];
       end
       
       %This function takes a 1x3 array of the position unit vector and a
       %scalar value and returns a 3x1 task space instantenous velocity
       %vector
       function tSpace = scale_vector(obj, matrix, scalar)
           xVal = matrix(1) * scalar;
           yVal = matrix(2) * scalar;
           zVal = matrix(3) * scalar;
           tSpace = [xVal; yVal; zVal];
       end
    end
end