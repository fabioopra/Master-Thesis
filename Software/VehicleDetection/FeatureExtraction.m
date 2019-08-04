%CANCELLARE NEL MAIN IN USCITA BW1

function [ Features] = FeatureExtraction(BW)
%This function computes features of the white blobs in the Black and White
%Image

%INPUT
    %BW         %Black and White image

%OUTPUTS

%Features       Table containing all the usefull information of
                %the bright objects:

% Area                      Actual area of the bright
                            %object in px
% Centroid(x,y)             Center of mass of the bright
                            %object (x,y) coordinate
% Bounding Box              [Top_Left_x_coordinate, Top_left_y_coordinate, width, height]
% Top                       y coordinate of the top side of 
                            %the bounding_box
% Bottom                    y coordinate of the bottom side
                            %of the bounding_box
% Left                      x coordinate of the left side
                            %of the bounding_box
% Right                     x coordinate of the right side
                            %of the bounding_box
% Height                    Height of the Bounding Box
% Width                     Width of the Bounding Box

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    BW1 =  bwconncomp(BW); %Assings to each pixel the number of the object
                           %to which it belongs
                           
   Features = regionprops('table',BW1,'centroid','Area','BoundingBox');
    % Bounding Box [Top_Left_x_coordinate, Top_left_y_coordinate, width, height]
    %if the Frame size is 1080 1920 3, y goes from 1 to 1080, x goes from 1 to 1920
    %regionprops is a MATLAB function that computes the written features.
    
    if height(Features) > 0
     
        Features.top=ceil(Features.BoundingBox(:,2)); %define the top coordinate of the Blob
        Features.bottom=floor(Features.BoundingBox(:,2))+floor(Features.BoundingBox(:,4)); %define the bottom coordinate of the Blob
        Features.left=ceil(Features.BoundingBox(:,1));%define the left coordinate of the Blob
        Features.right=floor(Features.BoundingBox(:,1))+floor(Features.BoundingBox(:,3)); %define the right coordinate of the Blob
        Features.height=floor(Features.BoundingBox(:,4)); %define the height of the Blob
        Features.width=floor(Features.BoundingBox(:,3));  %define the width of the Blob
        Features.x=Features.left+Features.width/2;
        Features.y=Features.top+Features.height/2;
       
    end
   
end

