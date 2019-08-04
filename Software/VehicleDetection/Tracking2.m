function [ TP,ID,state_est,P_est,width1,height1,bbox ] = Tracking2(TP,PV,ID,OverlArea_thresh,ratioBB_thresh,...
                                                   Hor_thresh,missing_vehicles,state_est,P_est,width1,height1,x_optical,y_optical)
%this function executes the tracking of two paired blobs in time

%INPUTS
    %TP                 Tracked possible vehicles: features of paired objects up to frame t-1
    %PV                 Possible vehicles: features of paired objects at the current frame
    %ID                 Identification number that identifies each vehicle (vector)
    %OverlArea_thresh   Threshold of the overlapping area of the bounding
                        %boxes of different frames to be tracked
    %ratioBB_thresh     Threshold of the ratio of the ratio of W/H of the 
                        %bounding boxes of different frames to be tracked
    %missing_vehicles   Number of frames that should pass for deleting a
                        %tracked vehicle
%OUTPUTS
    %TP         Tracked possible vehicles: features of paired objects up to frame t-1
    %PV         Possible vehicles: features of paired objects at the current frame

                
% TP is a Table with as many rows as the number of vehicles (for each
% field of the struct there are as many elements as the number of vehicles)
%the full structure of TP and PV at the bottom of the function
          

%if TP is empty skip all this computations because there is no vehicle to betracked
if height(TP)>0 
    n=height(PV); %number of vehicle in frame t
    m=height(TP); %number of vehicle stored up to frame t-1
        for i=1:m
            TP.Flag(i)=0; %initialization of flag of TP
                           
            for j=1:n

                %computation of the common features between detected
                %vehicles at frame t and t-1:
                %1)checking in the bounding boxed have similar proportions
                %2)checking if the bounding boxes highly overlapped 
                
                ratio_h = min(PV.Height(j),TP.Height(i))/max(PV.Height(j),TP.Height(i));
                ratio_w = min(PV.Width(j),TP.Width(i))/max(PV.Width(j),TP.Width(i));

                %finding the measures of the common area of the two boundingbox
                %of the two different frames (finding before the overlapping
                %height and width and then multiplying them to obtain A)
                Hij = min(PV.Bottom(j),TP.Bottom(i))-max(PV.Top(j),TP.Top(i));  
                Wij = min(PV.Right(j),TP.Right(i))-max(PV.Left(j),TP.Left(i));

                Aij = Hij*Wij; %common area between boundingboxes of two different frames

                Overlap_ij=Aij/max(PV.Area(j),TP.Area(i)); % overlapping area (%)

                %symmetry score: ratio between the ratios of width/height
                RatioBB=min(PV.Ratio(j),TP.Ratio(i))/max(PV.Ratio(j),TP.Ratio(i));
                
                Hor_over=Wij/max(PV.Width(j),TP.Width(i));

    %%%%%%%%% a) UPDATE
                if  ((Hij> 0) && (Wij> 0) && (Overlap_ij > OverlArea_thresh) && (RatioBB > ratioBB_thresh))%...
                       % && Hor_over>=Hor_thresh
                    %if two vehicles match

                    TP{i,1:13}=PV{j,1:13};  %update TP with the new feature of that vehicle
                    TP.Found(i)=TP.Found(i)+1; %counter of that vehicle increases
                    TP.Missing(i)=0; %not missing
                    TP.Flag(i)=TP.Flag(i)+1; %this element of TP has been found
                    PV.Flag(j)=PV.Flag(j)+1; %this element of PV has been found
                      %this element of TP has been found
                end   

            end %end inner for

            if TP.Flag(i)==0  %if this vehicle of TP has not been found in the new frame
                
                %calculating the kalman estimated state features
                x_est=state_est(1,i)+x_optical;
                y_est=state_est(2,i)+y_optical;
                left_est=x_est-(width1(i))/2;
                top_est=y_est-(height1(i))/2;
                width_est=width1(i);
                height_est=height1(i);
                right_est=left_est+width_est;
                bottom_est=top_est+height_est;
                ratio_est=width_est/height_est;
                
                %using the kalman predicted features in place of the stored
                %ones
                TP.Top(i)=top_est;
                TP.Bottom(i)=bottom_est;
                TP.Left(i)=left_est;
                TP.Right(i)=right_est;
                TP.Width(i)=width_est;
                TP.Height(i)=height_est;
                TP.Ratio(i)=ratio_est;
                TP.x(i)=x_est;
                TP.y(i)=y_est;
                TP.Missing(i)=TP.Missing(i)+1; %counter of missing that vehicle increases
            end

         end %end outer for
         
    %%%%%%%%% b) APPEAR
        %looking for that NEW vehicles that do not match any of the old ones
        %and adding that vehicle to the list of all vehicles (adding to TP)
        for i=1:n
            if PV.Flag(i)==0
                ID=ID+1;
                TP{end+1,:}=[PV{i,1:13},0,1,0,ID]; %adding new vehicle
%                 TP.Flag(end)=0;
%                 TP.Found(end)=1;    %counter found =1
%                 TP.Missing(end)=0;  %counter missing =0
                state_est(:,end+1)=zeros(6,1);
                P_est(:,:,end+1)=zeros(6,6);
                width1(end+1,1)=TP.Width(end);
                height1(end+1,1)=TP.Height(end);
            end
        end


    %%%%%%%%% c) DISAPPEAR
       %if an old tracked vehicle (TP) does not track any potential vehicle
       %(PV) for more than 3 consecutive frames, than this vehicle is removed
       %from the tracking 
       i=1;
       while i<=height(TP)   %i inspect all the TP vector
            if TP.Missing(i)>=missing_vehicles
                TP(i,:)=[]; %removing that vehicle
                state_est(:,i)=[];
                P_est(:,:,i)=[];
                width1(i)=[];
                height1(i)=[];
                i=i-1;
            end
            i=i+1; 
       end
       
       %1)removing double (or triple...) vehicles: it could happen that in
       %a frame the same vehicle is not found due to the noise; in this
       %case the algorithm add this vehicle as if it were a new one. In one
       %of the nex frame the camera moves again vertically (noise) and the
       %new box matches with both the previous two. In this case the
       %features of this box are updated (equally) in both this rows of the
       %TP Table. At the end the result is that there could be multiple
       %instances of exactly the same vehicle. The redundant istaces (with
       %less occurence) are removed.
       %2)removing two boundingboxes that overlap, keeping just the one
          %that has been found more times
       del=[];
       i=1;
       while i<=(height(TP)-1)
            j=i+1;
            while j<=height(TP)
               
                if TP{i,1:13}==TP{j,1:13} %1)
                    del=[del,j];
                end
                
                j=j+1;
            end
            i=i+1;
       end
        TP(del,:)=[];
        state_est(:,del)=[];
        P_est(:,:,del)=[];
        width1(del)=[];
        height1(del)=[];
       
        
       del=[];
       i=1;
       while i<=(height(TP)-1)
            j=i+1;
            while j<=height(TP)
                
                    Hij = min(TP.Bottom(j),TP.Bottom(i))-max(TP.Top(j),TP.Top(i));  
                    Wij = min(TP.Right(j),TP.Right(i))-max(TP.Left(j),TP.Left(i));
                    Aij = Hij*Wij; %common area between boundingboxes of two different frames
                    Overlap_ij=Aij/max(TP.Area(j),TP.Area(i)); % overlapping area (%)

                    if Overlap_ij>0 && Hij>0 && Wij>0 %2)

                        if TP.Found(i)>TP.Found(j)+11
                            del=[del,j];
                        elseif TP.Found(i)+11<TP.Found(j)
                            del=[del,i];  
                        end

                end
                j=j+1;
            end
            i=i+1;
       end
        TP(del,:)=[];
        state_est(:,del)=[];
        P_est(:,:,del)=[];
        width1(del)=[];
        height1(del)=[];
        
        
else %if no tracked vehicle is present (TP empty)
    if height(PV)>0 %if vehicles are found in current frame (PV not empty)
        TP=PV; %TP is equal to PV because the are no stored vehicles
        TP.Found=ones(height(PV),1);
        TP.Missing=zeros(height(PV),1);
        TP.ID=(1:1:height(PV))';
        ID=TP.ID(end);
        width1=TP.Width;
        height1=TP.Height;
    else %if vehicles are not found in current frame (PV empty)
        ID=0; 
    end
end %end if  
  
%%%%%%%%% KALMAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if height(TP)>0
    [bbox,state_est,P_est,width1,height1] = kalman2(TP,state_est,P_est,width1,height1,x_optical,y_optical);
else bbox=[];
end



end %end function





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%structure of PV:
%1 red
%2 green
%3 blue
%4 top
%5 bottom
%6 left
%7 right
%8 width
%9 height
%10 ratio (width/height)
%11 area
%12 x (centre of bounding box)
%13 y (centre of bounding box)
%14 flag (1 if it is found 1 time in previous frames; >1 if it is found multiple times in
%          previous frames; 0 if it is not found in previous frames)

%structure of TP:
%1:13 as PV
%14 flag of TP
%15 found (counter that sais how many times this old vehicle has been found
%          in the new frame)
%16 missing (counter that sais how many times this old vehicle is missing
%            in the last frames)