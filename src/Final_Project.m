%%
% RBE3001 - Final Project
%
% Robot Pick and Place System
%
% Lines 9-29 perform necessary library initializations. You can skip reading
% to line 31.

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
robot = Robot(myHIDSimplePacketComs);; % Creating the Robot Object
plotter = Plot(); % Creating the Plot Object
traj = Traj_Planner(robot);
traj.moveHome();
cam = Camera();% Creating the Camera Object
sort = Sorting();

try


    SERV_ID = 1848;            % we will be talking to server ID 1848 on
    % the Nucleo
    SERVER_ID_READ =1910;% ID of the read packet
    DEBUG   = false;          % enables/disables debug prints
    
    pts = [574 480; 959 469; 536 834; 967 834];
    part2 = false;
    part3 = false;
    part4 = false;
    
    ballPts = [];
    startPts = [];
    endPts = [];
    colors = [];
    
    %State machine
    ballIdentification = 1;
    ballSorting = 2;
    returnHome = 3;
    sortingComplete = 4;
    pickAndPlace = 5;
    
    state = ballIdentification;
    running = true;
    
    
    %----------------------------Should be calculated at the begining------------------------------------------------------------------
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    
    T_Image_Checker = cam.getCameraPose(); % Creates a transformation matrix from image to checker
    
    T_0_Checker = robot.dh2mat([0, 0, 50, 180])* robot.dh2mat([-90, 0, -100, 0]); % Creates a transformation matrix from robot base to checker
    
    cam_params = cam.params;
    
    %----------------------------------------------------------------------------------------------------------------------------------
    %-----------------------To be included in the state machine------------
    while running
        fprintf("State is %d\n", state)
        if ballIdentification == state
            disp('Put the objects on the checkerboard, then press any key to continue');
            pause;
            fprintf('\n')
            inputPic = snapshot(cam.cam);
            im = undistortImage(inputPic,cam.params, 'OutputView', 'full');
            im = imresize(im, [1080 1920]);

            %Setting all boolean values to true for sorting
            pinkD = true;
            redD = true;
            yellowD = true;
            greenD = true;
            randObj = true;
            
            ballPts = [];
            startPts = [];
            endPts = [];
            colors = [];

            if (pinkD)
                disp('Check for pink ball');
                M1 = pinkMask(im);
                erodeI = medfilt2(M1,[75,75]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    pinkD = false; %Pink Color not found
                    disp('Pink ball not found');
                else
                    disp('Found pink ball');
                    Pt = centroidGeneration(BW);
                    colors = [colors; sort.pink];
                    ballPts = [ballPts; coordGeneration(Pt, T_Image_Checker, T_0_Checker, cam.params)];

                    figure (1)
                    imshow(BW);
                    hold on
                    plot(Pt(1),Pt(2),'b*')
                    hold off

%                     redD = false;
%                     yellowD = false;
%                     greenD = false;
                end
            end

            if (redD)
                disp('Check for red ball');
                M1 = redMask(im);
                erodeI = medfilt2(M1,[75,75]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    redD = false; %Red Color not found
                    disp('Red ball not found');
                else
                    disp('Found red ball');
                    Pt = centroidGeneration(BW);
                    colors = [colors; sort.red];
                    ballPts = [ballPts; coordGeneration(Pt, T_Image_Checker, T_0_Checker, cam.params)];

                    figure (1)
                    imshow(BW);
                    hold on
                    plot(Pt(1),Pt(2),'b*')
                    hold off

%                     yellowD = false;
%                     greenD = false;

                end
            end

%             if (yellowD)
%                 disp('Check for yellow ball');
%                 M1 = yellowMask(im);
%                 erodeI = medfilt2(M1,[75,75]);
%                 BW = imfill(erodeI, 'holes');
%                 B = nnz(BW);
% 
%                 if (B == 0)
%                     yellowD = false; %Yellow color not found
%                     disp('Yellow ball not found');
%                 else
%                     disp('Found yellow ball');
%                     Pt = centroidGeneration(BW);
%                     colors = [colors; sort.yellow];
%                     ballPts = [ballPts; coordGeneration(Pt, T_Image_Checker, T_0_Checker, cam.params)];
% 
%                     figure (1)
%                     imshow(BW);
%                     hold on
%                     plot(Pt(1),Pt(2),'b*')
%                     hold off
% 
% %                     greenD = false;
%                 end
%             end

            if (greenD)
                disp('Check for green ball');
                M1 = greenMask(im);
                erodeI = medfilt2(M1,[75,75]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    greenD = false; %Yellow color not found
                    disp('Green ball not found');
                else
                    disp('Found green ball');
                    Pt = centroidGeneration(BW);
                    colors = [colors; sort.green];
                    ballPts = [ballPts; coordGeneration(Pt, T_Image_Checker, T_0_Checker, cam.params)];

                    figure (1)
                    imshow(BW);
                    hold on
                    plot(Pt(1),Pt(2),'b*')
                    hold off
                end
%             else
%                 disp('No balls to sort!');
            end
            
            if (randObj)
                disp('Check for random object');
                M1 = randObjMask(im);
                erodeI = medfilt2(M1,[75,75]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    redD = false; %Red Color not found
                    disp('Random object not found');
                else
                    disp('Found random objectl');
                    Pt = centroidGeneration(BW);
                    colors = [colors; sort.red];
                    ballPts = [ballPts; coordGeneration(Pt, T_Image_Checker, T_0_Checker, cam.params)];

                    figure (1)
                    imshow(BW);
                    hold on
                    plot(Pt(1),Pt(2),'b*')
                    hold off
                end
            end
            
            state = ballSorting;
            
        elseif state == ballSorting
            [startPts, endPts] = sort.sort(colors, ballPts);
            if endPts(1,1) < 0
               disp("Nothing to sort!")
               disp("Completing the run")
               state = sortingComplete;
            else
                state = pickAndPlace;
            end
        elseif state == pickAndPlace
            for i = 1:size(endPts,1)
                disp("Pick and place for")
                disp(colors)
                disp("From")
                disp(startPts)
                disp("To")
                disp(endPts)
                traj.pickAndPlace(startPts(i,:), endPts(i,:));
            end
            disp("Pick and place complete, returning home")
            disp(ballPts);
            state = returnHome;
        elseif state == returnHome
            traj.moveHome();
            state = ballIdentification;
        elseif state == sortingComplete
            traj.moveHome();
            running = false;
        end
    end
%----------------------------------------------------------------------------
%     if part4 == true
%         
%         disp('Put the objects on the checkerboard, then press any key to continue');
%         pause;
%         fprintf('\n')
%         inputPic = snapshot(cam.cam);
%         im = undistortImage(inputPic,cam.params, 'OutputView', 'full');
%         
%         M1 = pinkMask(im);
%         
%         erodeI = medfilt2(M1,[75,75]);
%         BW = imfill(erodeI, 'holes');
%         
%         B = nnz(BW); % checks if the binary image has the object
%                      % return 0 if no object seen
%         
%         Point = centroidGeneration(BW); % Call for helper function which generates centroid
%         
%         disp(T_Image_Checker);
%         
%         actualPoint = coordGeneration(Point, T_Image_Checker,T_0_Checker,cam_params); % Call for helper function which converts
%                                                                                       % pixel coordinates to x-y coordinates wrt base frame
%         
%         disp(actualPoint);
%         
%         figure(1)
%         imshow(im);
%         
%         
%         figure(2)
%         imshow(BW);
%         hold on
%         plot(Point(1),Point(2),'b*')
%         hold on
%         plot(actualPoint(1),actualPoint(2),'rx')
%         hold off
%         
%     end
%     if part3 == true
%         disp('Put the objects on the checkerboard, then press any key to continue');
%         pause;
%         fprintf('\n')
%         inputPic = snapshot(cam.cam);
%         im = undistortImage(inputPic,cam.params, 'OutputView', 'full');
%         
%         
%         M1 = pinkMask(im);
%         
%         
%         erodeI = medfilt2(M1,[75,75]);
%         BW = imfill(erodeI, 'holes');
%         
%         B = nnz(BW); % checks if the binary image has the object
%         % return 0 if no object seen
%         %
% %         BI = regionprops(erodeI, 'Centroid');
% %         centroids = BI.Centroid;
%         Pt = centroidGeneration(BW);
%         
%         
%         figure(1)
%         imshow(im);
%         
%         figure(2)
%         imshow(M1);
%         
%         figure(3)
%         imshow(BW);
%         hold on
%         plot(Pt(1),Pt(2),'b*')
%         hold off
%         
%         disp((Pt));
%                 
%     end
%     
%     fprintf('\n')
%     
%     
    if part2 == true
        inputPic = snapshot(cam.cam);
        im = undistortImage(inputPic,cam.params, 'OutputView', 'full');
        im = imresize(im, [1080 1920]);
        figure(7)
        imshow(im);
        figure(8)
        imshow(inputPic);
        undistort = undistortPoints(pts, cam.params);
        for i = 1:size(pts,1)
            disp("Pixels")
            disp(pts(i,:))
            actualPts = coordGeneration(pts(i,:),T_Image_Checker,T_0_Checker,cam_params);
            disp("Coords")
            disp(actualPts)
            actualPts = coordGeneration(undistort(i,:),T_Image_Checker,T_0_Checker,cam_params);
            disp("Coords Undistorted")
            disp(actualPts)
        end
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

% Clear up mem
robot.shutdown()