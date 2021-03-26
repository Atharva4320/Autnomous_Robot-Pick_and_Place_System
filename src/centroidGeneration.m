function Point = centroidGeneration(BW)
BI = regionprops(BW, 'Centroid');
centroids = BI.Centroid;
x_coord = centroids(:,1);
y_coord = centroids(:,2);
Point = [x_coord,y_coord];
end

