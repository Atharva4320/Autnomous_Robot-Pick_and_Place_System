classdef Traj_Planner
    properties
        robot;
        hoverHeight = 100;
        ballHeight = 10;
        timeStep = 0.01;
        moveTime = 0.5;
    end
    methods
        function obj = Traj_Planner(robot)
            obj.robot = robot;
        end
        function traj = cubic_traj(~, t0, tf, v0, vf, q0, qf)
            
            A = [
                1,  t0, t0^2,   t0^3;
                0,  1,  2*t0,   3*t0^2;
                1,  tf, tf^2,   tf^3;
                0,  1,  2*tf,   3*tf^2;
                ];
            
            x = [q0; v0; qf; vf];
            
            traj = inv(A) * x;
        end        
        function coeff = quintic_traj(~,t0,tf,v0,vf,q0,qf)
            
            A = [
                1,  t0, (t0)^2,     (t0)^3,         (t0)^4,         (t0)^5;
                0,  1,  2*(t0),     3*((t0)^2),     4*((t0)^3),     5*((t0)^4);
                0,  0,  2,          6*(t0),         12*((t0)^2),    20*((t0)^3);
                1,  tf, (tf)^2,     (tf)^3,         (tf)^4,         (tf)^5;
                0,  1,  2*(tf),     3*((tf)^2),     4*((tf)^3),     5*((tf)^4);
                0,  0,  2,          6*(tf),         12*((tf)^2),    20*((tf)^3);
                ];
             
             b = [q0; v0; 0; qf; vf; 0];
             
             coeff = inv(A) * b;
        end
        function linear = linear_traj(~,t0,tf,q0,qf)
            
            linear = obj.cubic_traj(t0,tf,0,0,q0,qf);
        end
        %Takes a single array of start and end points from
        %which to pick and place a ball
        function pickAndPlace(obj, startPt, endPt)
            hover = [startPt(1:2) obj.hoverHeight];
            lower = [startPt(1:2) obj.ballHeight];
            obj.moveTo(hover);
            obj.lower(lower, true);
            hover = [endPt(1:2) obj.hoverHeight];
            lower = [endPt(1:2) obj.ballHeight];
            obj.moveTo(hover);
            obj.lower(lower, false);
        end
        function moveTo(obj, pts)
            disp("Move to ")
            disp(pts);
            pts(2,:) = pts(1,:);
            pts(1,:) = obj.robot.position_cp();
            i = 1;
            int_time = 0:obj.timeStep:obj.moveTime;

            syms t;

                x_coeff = obj.quintic_traj(0,obj.moveTime,0,0,pts(i,1),pts(i+1,1));
                y_coeff = obj.quintic_traj(0,obj.moveTime,0,0,pts(i,2),pts(i+1,2));
                z_coeff = obj.quintic_traj(0,obj.moveTime,0,0,pts(i,3),pts(i+1,3));


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
                    angles = obj.robot.ik3001(packet);
                    obj.robot.interpolate_jp(angles,5);
                    pause(obj.timeStep)
                end
            disp(obj.robot.position_cp());
        end
        function lower(obj, endPt, isGrabbing)
            disp("Lower to ")
            disp(endPt);
            pts(2,:) = endPt(1,:);
            pts(1,:) = obj.robot.position_cp();
            pts(3,:) = pts(1,:);
            pts(4,:) = pts(1,:);
            int_time = 0:obj.timeStep:obj.moveTime;
            
            if isGrabbing
                obj.robot.openGripper();
            else
                obj.robot.closeGripper();
            end

            syms t;
            for i=1:size(pts)-1

                x_coeff = obj.quintic_traj(0,obj.moveTime,0,0,pts(i,1),pts(i+1,1));
                y_coeff = obj.quintic_traj(0,obj.moveTime,0,0,pts(i,2),pts(i+1,2));
                z_coeff = obj.quintic_traj(0,obj.moveTime,0,0,pts(i,3),pts(i+1,3));


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
                    angles = obj.robot.ik3001(packet);
                    obj.robot.interpolate_jp(angles,5);
                    pause(obj.timeStep)
                end 
                if isGrabbing
                    obj.robot.closeGripper();
                else
                    obj.robot.openGripper();
                end
            end
            disp(obj.robot.position_cp());
        end
        function moveHome(obj)
           transform = obj.robot.fk3001([0 0 0]);
           pts = [transform(1,4) transform(2,4) transform(3,4)];
           obj.moveTo(pts);
        end
    end 
end
