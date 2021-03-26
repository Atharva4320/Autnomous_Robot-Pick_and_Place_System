function finalPoint = coordGeneration(Point,T_Image_Checker,T_0_Checker,params)

% Point = undistortPoints(Point, params);

% The rotation matrix calculated from T_Image_Checker
R = [T_Image_Checker(1,1), T_Image_Checker(1,2), T_Image_Checker(1,3);
    T_Image_Checker(2,1), T_Image_Checker(2,2), T_Image_Checker(2,3);
    T_Image_Checker(3,1), T_Image_Checker(3,2), T_Image_Checker(3,3)];

% The position vector calcualted from T_Image_Checker
P = [T_Image_Checker(1,4), T_Image_Checker(2,4), T_Image_Checker(3,4)];

R_0_Checker = [T_0_Checker(:,1), T_0_Checker(:,2), T_0_Checker(:,3)]; % The rotation matrix from robot base to checker

checkerPoints = pointsToWorld(params,R,P,Point); % Creates X and Y position in the checkerboard frame (in mm)
msg1 = ['The X and Y position in the checkerboard frame at Point is ',num2str(checkerPoints)];
disp(msg1);

%Compensations for height:
P = checkerPoints(2);
H = 101.6 + (31.75+ 38.1);
h = 12;

p = (P/H)*h;

actualPoint = [checkerPoints(1), (checkerPoints(2))]; % + (p+1.5))];
msg2 = ['The X and Y position after height compensation at Point is ',num2str(actualPoint)];
disp(msg2);

P_Checker = [actualPoint(1); actualPoint(2); 0; 1]; % The position metrix
T_image_Checker = horzcat(R_0_Checker,P_Checker); % Appending the rotation matrix and position matrix

% disp('The transformation matrix:');
% disp(T_image_Checker);

finalT = T_0_Checker* T_image_Checker;
disp('The transformation matrix from the base:');
disp(finalT);

pointX = finalT(1,4);
pointY = finalT(2,4);

P = pointY;
H = 101.6 + (31.75 + 38.1);
h = 6;

p = (P/H)*h;

pointY = pointY;% + (p+1.5);

actualPoint = [pointX, pointY];
msg3 = ['The X and Y position after height compensation at Point is ',num2str(actualPoint)];
disp(msg3);

finalPoint = [pointX,pointY];
fprintf('\n')
end

