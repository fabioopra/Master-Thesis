function [PV,Features ] = PossibleVehicles( Features, idx2,noLightsAbove,noLightsBelow,TP)
%Section A) It stores the useful features for tracking of a possible 
%vehicle(pairs) (bounding box, height,width...) and returns that features 
%in a table

%Section B)It removes all the blobs that are included in the big bounding 
%-box: that includes the paired lights, the lights with overlap 1*height 
%above the paired lights and the lights with centroid 2.5*height below the 
%paired lights

    
%INPUTS
    %Features       Table with all the features of all the blobs
    %idx2           Matrix with two columns with the two index of the
                    %paired blobs
    %noLightsAbove  Amount of times of the height of the pair bounding box
                    %above the pair in which we do not search for lights
    %noLightsBelow  Amount of times of the height of the pair bounding box
                    %below the pair in which we do not search for lights
    %TP             Table used for tracking
%OUTPUTS
    %PV         Table with the features useful for tracking. At the bottom
                %of the page it is found the complete structure of PV
    %Features   Table with all the features of the blobs that have not been
                %paired or erased 
                
                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Section A %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
%Creating the feature vector useful for tracking

[n,m]= size(idx2);
PV= table;

%if the vector of the pairs is not empty we calculate some pairs features
%that will be useful for "Tracking"
    if n>0 && m>0
        
        %the higher y coordinate of the bounding-box that covers only the
        %vehicle lights
        Top=min(Features.top(idx2(:,1)),Features.top(idx2(:,2)));
        
        %the lower y coordinate of the bounding-box that covers only the
        %vehicle lights
        Bottom=max(Features.bottom(idx2(:,1)),Features.bottom(idx2(:,2)));
        
        %height of the bounding-box that covers only the vehicle lights
        Height=Bottom-Top;
        
        %higher y coordinate of the big boundingbox that covers all the
        %vehicle
        PV.Top=Top-noLightsAbove*Height; %1
        
        %lower y coordinate of the big boundingbox that covers all the
        %vehicle
        PV.Bottom=Bottom+noLightsBelow*Height; %2
        
        %x coordinate of the left side of the boundingbox (big or small)
        PV.Left=min(Features.left(idx2(:,1)),Features.left(idx2(:,2))); %3
        
        %x coordinate of the right side of the boundingbox (big or small)
        PV.Right=max(Features.right(idx2(:,1)),Features.right(idx2(:,2)));%4
        
        %width of the boundingbox (big or small)
        PV.Width=PV.Right-PV.Left; %5
        
        %height of the big boundingbox that covers all the vehicle
        PV.Height=Height*(noLightsAbove+noLightsBelow+1); %6
        
        %ratio of the width/height-ratio of the big boundingbox that covers
        %all the vehicle
        PV.Ratio=PV.Width./PV.Height; %7
       
        %Area of the big boundingbox
        PV.Area=PV.Width.*PV.Height; %8
        
        %x coordinate of the center of the big boundingbox
        PV.x= (PV.Right + PV.Left)/2; %9
        
        %y coordinate of the center of the big boundingbox
        PV.y= (PV.Top + PV.Bottom)/2; %10    
        
        %flag that tells if this vehicle is found in the current frame
        %(initialization of the variable to 0)
        PV.Flag=zeros(n,1); %11

    end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Section B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Removing lights on the car
%in this section the tracked vehicles table is scanned and for each tracked
%vehicle it is checked whether there are some newly founded blobs that
%overlaps with the tracked vehicles. If this happens, the new blob is
%deleted

    del=[];
    %scanning tracked vehicles
    for i=1:height(TP)
        
        j=1;
        %scanning new blobs
        while j<=height(Features)
            %vertical overlapping between pairs and blobs (positive if they
            %overlaps)
            Hij = min(Features.bottom(j),TP.Bottom(i))-max(Features.top(j),TP.Top(i));  
            
            %horizontal overlapping between pairs and blobs (positive if 
            %they overlaps)
            Wij = min(Features.right(j),TP.Right(i))-max(Features.left(j),TP.Left(i));

            %overlapping area between tracked pairs and blobs
            Aij = Hij*Wij; 

            %overlapping between tracked pairs and blobs (percentage with
            %respect the maximum area)
            Overlap_ij=Aij/max(Features.Area(j),TP.Area(i)); % overlapping area (%)
            
            %if there is an overlapping then the blob is deleted
            if Overlap_ij>0 && Hij>0 && Wij>0
                del=[del,j];
                                
            end
            
            j=j+1;
        end
        
    end

    Features(del,:)=[];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%structure of PV:
%1 top
%2 bottom
%3 left
%4 right
%5 width
%6 height
%7 ratio (width/height)
%8 area
%9 x (centre of bounding box)
%10 y (centre of bounding box)
%11 flag (1 if it is found 1 time in previous frames; >1 if it is found multiple times in
%         previous frames; 0 if it is not found in previous frames)
