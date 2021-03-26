classdef Plot
    methods
        function plot = plot_arm(obj, arm)
            data = arm.measured_js(true, false);
            jointAngles = data(1,:);
            dhParams = [jointAngles(1) 95 0 -90; jointAngles(2)-90 0 100 0; jointAngles(3)+90 0 100  0];
            t1 = arm.dh2mat(dhParams(1,:));
            t2 = t1 * arm.dh2mat(dhParams(2,:));
            t3 = t2 * arm.dh2mat(dhParams(3,:));
            links = [0, t1(1,4), t2(1,4), t3(1,4);0, t1(2,4), t2(2,4), t3(2,4);0, t1(3,4), t2(3,4), t3(3,4)];
            plot = plot3(links(1,:),links(2,:),links(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
        end
        function plot = plot_arm_triangle_measured(obj, arm, goal, measured)
            figure(1)
            data = measured;
            jointAngles = data(1,:);
            dhParams = [jointAngles(1) 95 0 -90; jointAngles(2)-90 0 100 0; jointAngles(3)+90 0 100  0];
            t1 = arm.dh2mat(dhParams(1,:));
            t2 = t1 * arm.dh2mat(dhParams(2,:));
            t3 = t2 * arm.dh2mat(dhParams(3,:));
            vertex1 = arm.fk3001(goal(1,:));
            vertex2 = arm.fk3001(goal(2,:));
            vertex3 = arm.fk3001(goal(3,:));
            links = [0, t1(1,4), t2(1,4), t3(1,4);0, t1(2,4), t2(2,4), t3(2,4);0, t1(3,4), t2(3,4), t3(3,4)];
            triangle = [vertex1(1,4) vertex2(1,4) vertex3(1,4) vertex1(1,4);
                vertex1(2,4) vertex2(2,4) vertex3(2,4) vertex1(2,4);
                vertex1(3,4) vertex2(3,4) vertex3(3,4) vertex1(3,4)];
%             plot3(vertex1(1,:),links(2,:),links(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
%             plot = plot3(triangle(1,:),triangle(2,:),triangle(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000');
            plot = plot3(links(1,:),links(2,:),links(3,:),'-o', triangle(1,:),triangle(2,:),triangle(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000');
        end
        function plot = plot_workspace(obj, arm)
            t1=linspace(-90,90,180);
            t2=linspace(-0,90,180);
            t3=linspace(-90,0,180);
            [theta1,theta2,theta3]=ndgrid(t1,t2,t3);
            x = 100.*cos((pi.*theta1)/180).*cos((pi.*(theta2 - 90))/180) + 100.*cos((pi.*theta1)/180).*cos((pi.*(theta2 - 90))/180).*cos((pi.*(theta3 + 90))/180) - 100.*cos((pi.*theta1)/180).*sin((pi.*(theta2 - 90))/180).*sin((pi.*(theta3 + 90))/180);
            y = 100.*sin((pi.*theta1)/180).*cos((pi.*(theta2 - 90))/180) + 100.*sin((pi.*theta1)/180).*cos((pi.*(theta2 - 90))/180).*cos((pi.*(theta3 + 90))/180) - 100.*sin((pi.*theta1)/180).*sin((pi.*(theta2 - 90))/180).*sin((pi.*(theta3 + 90))/180);
            z = 95 - 100.*cos((pi.*(theta2 - 90))/180).*sin((pi.*(theta3 + 90))/180) - 100.*cos((pi.*(theta3 + 90))/180).*sin((pi.*(theta2 - 90))/180) - 100.*sin((pi.*(theta2 - 90))/180);
            plot = plot3(x(:),y(:),z(:),'.');
        end
        function plot = plot_arm_triangle(obj, arm, goal)
            measured = arm.measured_js(true, false);
            plot = obj.plot_arm_triangle_measured(arm, goal, measured);
        end
        function plot = plot_arm_velocity(obj, arm, goal)
            figure(1)
            data = arm.measured_js(true, true);
            jointAngles = data(1,:);
            qdot = data(2,:);
            dhParams = [jointAngles(1) 95 0 -90; jointAngles(2)-90 0 100 0; jointAngles(3)+90 0 100  0];
            t1 = arm.dh2mat(dhParams(1,:));
            t2 = t1 * arm.dh2mat(dhParams(2,:));
            t3 = t2 * arm.dh2mat(dhParams(3,:));
            vertex1 = arm.fk3001(goal(1,:));
            vertex2 = arm.fk3001(goal(2,:));
            vertex3 = arm.fk3001(goal(3,:));
            links = [0, t1(1,4), t2(1,4), t3(1,4);0, t1(2,4), t2(2,4), t3(2,4);0, t1(3,4), t2(3,4), t3(3,4)];
            triangle = [vertex1(1,4) vertex2(1,4) vertex3(1,4) vertex1(1,4);
                vertex1(2,4) vertex2(2,4) vertex3(2,4) vertex1(2,4);
                vertex1(3,4) vertex2(3,4) vertex3(3,4) vertex1(3,4)];
%             plot3(vertex1(1,:),links(2,:),links(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
%             plot = plot3(triangle(1,:),triangle(2,:),triangle(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000');
            plot = plot3(links(1,:),links(2,:),links(3,:),'-o', triangle(1,:),triangle(2,:),triangle(3,:),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000');
            hold on
            jVec = arm.fdk3001(jointAngles, qdot);
            quiver3(t3(1,4), t3(2,4), t3(3,4), jVec(1,1), jVec(2,1), jVec(3,1), 0.1);
            hold off
        end
        function plotIt = plot_input(obj, arm)
            figure(5)
            data = arm.measured_js(true, false);
            jointAngles = data(1,:);
            dhParams = [jointAngles(1) 95 0 -90; jointAngles(2)-90 0 100 0; jointAngles(3)+90 0 100  0];
            t1 = arm.dh2mat(dhParams(1,:));
            t2 = t1 * arm.dh2mat(dhParams(2,:));
            t3 = t2 * arm.dh2mat(dhParams(3,:));
            links = [0, t1(1,4), t2(1,4), t3(1,4);0, t1(2,4), t2(2,4), t3(2,4);0, t1(3,4), t2(3,4), t3(3,4)];
            plotIt = plot(links(1,:),links(3,:));
        end
    end
end
