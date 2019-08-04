function [ linesl, linesr ] = houghROI( BW ,n,l,ThetaMinLeft,ThetaMaxLeft,ThetaMinRight,ThetaMaxRight,ResolutionHoughTheta,NumPeaks,Threshold,FillGap,MinLength,x_optical,y_optical,AroundOpticalCenter)
%This function finds the lines on the left and on the right of the
%image between the defined theta range, all the parameters have been
%explained above

%INPUT:

    %BW                     Edge detected version of the frame (Lower - Half)
    %n                      Height of the image
    %l                      Width of the image
    %ThetaMinLeft           Minimum angle for the left ROI 0 
    %ThetaMaxLeft           Maximum angle for the left ROI 90
    %ThetaMinRight          Minimum angle for the right ROI -89
    %ThetaMaxRight          Maximum angle for the right ROI 0
    %ResolutionHoughTheta   Resolution for the values of theta
    %NumPeaks               Number of peaks of the Hough space taken
    %Threshold              Threshold for the minimum number of times that
                            %a certain (rho,theta) occur, this value is in
                            %percentage(0-1) wih respect to the most found
                            %parameters (rho,theta)
    %FillGap                Maximum space between lines which is filled
                            %automatically
    %MinLength              Minimum length of the lines found with the
                            %parameters of the hough peaks (rho, theta)
    %x_optical              x-coordinate of the principal point 
    %y_optical              y-coordinate of the principal point
    %AroundOpticalCenter    parameter used to identify the range around the
                            %principal point in which lines passing through
                            %it are accepted
                            
%OUTPUT

    %linesl                 %parameters of all the lines found at the left
    %linesr                 %parameters of all the lines found at the right
    
    %lines structure
            %lines.rho
            %lines.theta
            %lines.point1           [x,y] point at the top of the line
            %lines.point2           [x,y] point at the bottom of the line

            
            
            
%% Computation of the Hough space for each half
     [Hl,Tl,Rl] = hough(BW(:,1:l/2), 'Theta', ThetaMinLeft:ResolutionHoughTheta:ThetaMaxLeft);
     %computation of the hough space for the left bottom half of the image
     [Hr,Tr,Rr] = hough(BW(:,l/2+1:l), 'Theta', ThetaMinRight:ResolutionHoughTheta:ThetaMaxRight);
     %computation of the hough space for the right bottom half of the image
     
    
%%  Find the parameters (rho, theta) that repeats the most
    Pl  = houghpeaks(Hl,NumPeaks,'threshold',ceil(Threshold*max(Hl(:))));
    %Pl contains the coordinates (rho,theta) of the hough space at which
    %peaks are located, for the left bottom half of the image
    Pr  = houghpeaks(Hr,NumPeaks,'threshold',ceil(Threshold*max(Hr(:))));
    %Pr contains the coordinates (rho,theta) of the hough space at which
    %peaks are located, for the right bottom half of the image
    
%%  Finds all the lines in the image whose parameters coincide with those
%%  found in the previous step
    linesl = houghlines(BW(:,1:l/2),Tl,Rl,Pl,'FillGap',FillGap,'MinLength',MinLength);
    %this function look for the lines with parameters (rho, theta) -> (Rl,Tl)
    %that coincide with the peaks found in the previous step (Pl), the
    %outcome is a variable containing for each line found:
    %line.rho
    %line.theta
    %line.point1     (x,y)
    %line.point2     (x,y)
    
    %Note
    %the two points     
    %line.point1     (x,y)
    %line.point2     (x,y)
    %Are at different order, sometimes point1 is above and sometimes it is
    %below therefore in the following steps this is organized
    linesr = houghlines(BW(:,l/2+1:l),Tr,Rr,Pr,'FillGap',FillGap,'MinLength',MinLength);
    %Same as before but for the right part
  
    
%% PREPROCESSING + PERSPECTIVE FILTERING
    
    %Pre-processing
    %moves the lines to the correct reference frame, since the previous
    %computations where done using parts of the complete image, where the
    %parameters (rho, theta) were computed from the (0,0) position of this
    %quarter of image, therfore since (0,0) position of each quarter of 
    %image (top-left corner of the quarter) do not coincide with the
    %absolute (0,0) (top-left corner of the image) then a traslation must 
    %be done

    
    %Perspective filtering
    %The filtering is done in order to delete lines that are for sure
    %wrong, this is done with the following rules;
    %
    %Distance to the optical center:
    %   -The lines are extended to the y-coordinate of the optical center
    %   then the horizontal distance from the line to the optical center is
    %   computed, if this distance is below 'AroundOpticalCenter' the line
    %   is kept, otherwise it is deleted.
    %
    %   -The short lines close to the optical center are usually noisy,
    %   therefore the lines whose bottom point's y-coordinate is lower than
    %   650 are deleted

    i=1;
    while i <= length(linesr)
            %PREPROCESSING
        if linesr(i).point1(2) > linesr(i).point2(2)
            %sometimes the points of the right lines are given in the
            %incorrect order in other words, point1 is at the bottom and
            %point2 is at the top then this 'if' is used to organize the
            %points of the line in the correct order.
            %Point1 will have the lowest y-coordinate (top)
            %Point2 will have the highest y-coordinate (bottom)
            auxpoint_x=linesr(i).point1(1);
            auxpoint_y=linesr(i).point1(2);
            linesr(i).point1=linesr(i).point2;
            linesr(i).point2=[auxpoint_x auxpoint_y];
        end
            linesr(i).point1(1)=linesr(i).point1(1)+l/2;
            linesr(i).point2(1)=linesr(i).point2(1)+l/2;
            linesr(i).point1(2)=linesr(i).point1(2)+n/2;
            linesr(i).point2(2)=linesr(i).point2(2)+n/2;
            linesr(i).rho=linesr(i).rho+cos(linesr(i).theta*pi/180)*l/2+sin(linesr(i).theta*pi/180)*n/2;
            %This piece of code is used to translate the lines from the
            %quarter reference corner to the full image reference corner.
            
%%          COMMENT THIS TO SEE ALL THE LINES FOUND WITHOUT FILTERING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %FILTERING
            xi=(linesr(i).rho-y_optical*sin(linesr(i).theta*pi/180))/cos(linesr(i).theta*pi/180);
            %computes the x-coordinate of the line at the height of the
            %optical point
            
             if (xi > x_optical+AroundOpticalCenter) || (xi < x_optical-AroundOpticalCenter)
                 %if the x-coordinate computed is outside the range definde
                 %by AroundOpticalCenter, the line is deleted
                 linesr(i)=[];
                 i=i-1;
             elseif linesr(i).point2(2) < 650
                 %if a line pass close to the optical center, the position
                 %of the line is checked if they are too close to the
                 %optical center, it is Ymax (bottom) belongs to (540,650)
                 %the line is deleted
                 linesr(i)=[];
                 i=i-1;
             end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             i=i+1;
    end
    
    i=1;    
    while i <= length(linesl)
        %PREPROCESSING
        
        if linesl(i).point1(2) > linesl(i).point2(2)
            %sometimes the points of the left lines are given in the
            %incorrect order in other words, point1 is at the bottom and
            %point2 is at the top then this 'if' is used to adjust this
            auxpoint_x=linesl(i).point1(1);
            auxpoint_y=linesl(i).point1(2);
            linesl(i).point1=linesl(i).point2;
            linesl(i).point2=[auxpoint_x auxpoint_y];
        end
        
        linesl(i).point1(2)=linesl(i).point1(2)+n/2;
        linesl(i).point2(2)=linesl(i).point2(2)+n/2;
        linesl(i).rho=linesl(i).rho+sin(linesl(i).theta*pi/180)*n/2;
        %translation
%%          COMMENT THIS TO SEE ALL THE LINES FOUND WITHOUT FILTERING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %FILTERING
            xi=(linesl(i).rho-y_optical*sin(linesl(i).theta*pi/180))/cos(linesl(i).theta*pi/180);
            
             if xi > x_optical+AroundOpticalCenter || xi < x_optical-AroundOpticalCenter
             %same as before
                  linesl(i)=[];
                  i=i-1;
        
             elseif linesl(i).point2(2) < 650
            %same condition as before
                  linesl(i)=[];
                  i=i-1;
             end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              i=i+1;
    end

end