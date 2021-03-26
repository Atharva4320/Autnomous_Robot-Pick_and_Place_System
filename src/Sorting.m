classdef Sorting
    properties
        green = 0;
        pink = 1;
        red = 2;
        yellow = 3;
        greenLocation = [150 75 0];
        pinkLocation = [50 -150 0];
        redLocation = [50 150 0];
        yellowLocation = [100 -75 0];
        limit = 25;
        stagingArea = [75 0 0];
    end
    methods
        %Takes in an array of the colors and a nx3 array of points
        %These lists must be synchronized
        %Returns the next move(s) 
        %Returns all -1 if no next moves are found
        function [startPts, endPts] = sort(obj, colors, points)
           startPts = [-1 -1 -1];
           endPts = [-1 -1 -1];
           index = -1;
           for i = 1:size(colors,2)
              if colors(i) == obj.green && obj.distance(obj.greenLocation,points(i,:)) > obj.limit
                  startPts = points(i,:);
                  endPts = obj.greenLocation;
                  index = i;
              elseif colors(i) == obj.pink && obj.distance(obj.pinkLocation,points(i,:)) > obj.limit
                  startPts = points(i,:);
                  endPts = obj.pinkLocation;
                  index = i;
              elseif colors(i) == obj.red && obj.distance(obj.redLocation,points(i,:)) > obj.limit
                  startPts = points(i,:);
                  endPts = obj.redLocation;
                  index = i;
              elseif colors(i) == obj.yellow && obj.distance(obj.yellowLocation,points(i,:)) > obj.limit
                  startPts = points(i,:);
                  endPts = obj.yellowLocation;
                  index = i;
              end
           end
           
%            disp(index)
%            disp(endPts)
           
           if index > -1
           
               for i = 1:size(colors,2)
%                    disp(obj.distance(endPts(1,:),points(i,:)) < obj.limit)
                   if i ~= index && obj.distance(endPts(1,:),points(i,:)) < obj.limit
                      startPts(2,:) = startPts(1,:);
                      startPts(1,:) = points(i,:);
                      endPts(2,:) = endPts(1,:);
                      endPts(1,:) = obj.stagingArea;
                   end
               end
           end    
        end
        function dist = distance(obj, pts1, pts2)
%             disp("Distance")
%             disp(pts2)
%             disp(pts1)
           dist = sqrt((pts1(1) - pts2(1))^2 +  (pts1(2) - pts2(2))^2);
        end
    end 
end
