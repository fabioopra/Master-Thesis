function [ TP,ID,state_est,P_est,bbox ] = Tracking3(TP,PV,ID,OverlArea_thresh,ratioBB_thresh,...
                                                   Hor_thresh,missing_vehicles,state_est,P_est,x_optical,y_optical,P0)
%this function executes the tracking of two paired blobs in time

%INPUT
    %TP                 Tracked possible vehicles: features of paired objects up to frame t-1
    %PV                 Possible vehicles: features of paired objects at the current frame
    %ID                 Identification number that identifies each vehicle
    %OverlArea_thresh   Threshold of the overlapping area of the bounding
                        %boxes of different frames to be tracked
    %ratioBB_thresh     Threshold of the ratio of the ratio of W/H of the 
                        %bounding boxes of different frames to be tracked
    %missing_vehicles   Number of frames that should pass for deleting a
                        %tracked vehicle
    %state_est          kalman states of each vehicles: each column has the               
                        %complete state of a vehicle 
    %P_est              %kalman covariance matrix of each vehicle:
                        %threedimensional matrix: each matrix has the 
                        %variace matrix of one vehicle
    %x_optical          x coordinate of the optical center
    %y_optical          y coordinate of the optical center
    %P0                 initializaton of the covariance matrix P in Kalman
    
    %OUTPUTS
    %TP                 Tracked possible vehicles: features of paired 
                        %objects up to frame t-1
    %ID                 univoque identification number assigned to each car
    %state_est          kalman states of each vehicles: each column has the               
                        %complete state of a vehicle 
    %P_est              %kalman covariance matrix of each vehicle:
                        %threedimensional matrix: each matrix has the 
                        %variace matrix of one vehicle
    %bbox               [x(top-left), y(top-left), width, height] of the
                        %boundingbox of each tracked vehicle: each row has
                        %the values for one vehicle
                
% TP is a Table with as many rows as the number of vehicles (for each
% field of the struct there are as many elements as the number of vehicles)
% the full structure of TP and PV at the bottom of the function
          

%if TP is empty skip all this computations because there is no vehicle to
%be tracked



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if height(TP)>0 
%%%%%%%%%%%%%% SECTION A %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    n=height(PV); %number of possible vehicle in frame t
    m=height(TP); %number of vehicle stored up to frame t-1
        for i=1:m
            TP.Flag(i)=0; %initialization of flag of TP
                           
            for j=1:n

                %computation of the common features between detected
                %vehicles at frame t and t-1:
                %1)checking in the bounding boxed have similar proportions
                %2)checking if the bounding boxes highly overlapped 
                

                %finding the measures of the common area of the two boundingbox
                %of the two different frames (finding before the overlapping
                %height and width and then multiplying them to obtain A)
                Hij = min(PV.Bottom(j),TP.Bottom(i))-max(PV.Top(j),TP.Top(i));  
                Wij = min(PV.Right(j),TP.Right(i))-max(PV.Left(j),TP.Left(i));
                Aij = Hij*Wij; %common area between boundingboxes of two different frames
                    
                % overlapping area (percentage)
                Overlap_ij=Aij/max(PV.Area(j),TP.Area(i)); 
                
                %symmetry score: ratio between the ratios of width/height
                RatioBB=min(PV.Ratio(j),TP.Ratio(i))/max(PV.Ratio(j),TP.Ratio(i));
                
                %horizontal overlapping between tracked vehicles and new
                %possible vehicles
                Hor_over=Wij/max(PV.Width(j),TP.Width(i));

%%%%%%%%%%%%% A1) UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if  ((Hij> 0) && (Wij> 0) && (Overlap_ij > OverlArea_thresh) && (RatioBB > ratioBB_thresh))&& PV.Flag(j)==0
                    %if two vehicles match: (overlapping area) && (similar
                    %proportions) && the newly found vehicle must match 
                    %with only one tracked vehicle, otherwise it can create
                    %multiple similar new vehicles
                    
%in this temporal matching each new data (PV) is used to update the kalman
%estimate of the previous tracked vehicles. So the matrix "state_est" will
%contain all the state vectors (in column) of all the tracked vehicles.
%After having updated the estimate of the the matrix "state_est" this
%features are substituted in the table TP that contains all the features of
%all the tracked vehicles
                    
                    %calculating the kalman state measuremet for the 
                    %current considered vehicle:
                    
                    %rho measured value
                    state_meas(1)=sqrt((PV.x(j)-x_optical)^2+(PV.y(j)-y_optical)^2);
                    %rho-speed measured value (not measured)
                    state_meas(2)=0;
                    %theta measured value
                    state_meas(3)=atan2((PV.y(j)-y_optical),(PV.x(j)-x_optical));
                    %width measured value
                    state_meas(4)=PV.Width(j);
                    %width-speed measured value (not measured)
                    state_meas(5)=0;
                    
                    %assigning the kalman state estimate for the 
                    %current considered vehicle:
                    %we assign the previous estimate 
                    state_e(1)=state_est(1,i);
                    state_e(2)=state_est(2,i);
                    state_e(3)=state_est(3,i);
                    state_e(4)=state_est(4,i);
                    state_e(5)=state_est(5,i);
                    
                    %assigning the kalman variance amatrix for the 
                    %current considered vehicle:
                    %we assign the previous prediction matrix 
                    P=P_est(:,:,i);
                    
                    %kalman predicts the future position based on the new
                    %measurement 
                    [state_est(:,i),P_est(:,:,i)] = kalman3(state_e',P,state_meas');

                    %calculating all the values obtained after the new
                    %kalman estimate (riottengo le coordinate cartesiane)
                    x_est=state_est(1,i)*cos(state_est(3,i))+x_optical;
                    y_est=state_est(1,i)*sin(state_est(3,i))+y_optical;
                    width_est=state_est(4,i);
                    height1=0.1*PV.Height(j)+0.9*TP.Height(i);
                    left_est=x_est-width_est/2;
                    top_est=y_est-(height1)/2;
                    right_est=left_est+width_est;
                    bottom_est=top_est+height1;
                    ratio_est=width_est/height1;

                    %updating the kalman predicted features in the vector
                    %of the stored vehicles (TP)
                    TP.Top(i)=top_est;
                    TP.Bottom(i)=bottom_est;
                    TP.Left(i)=left_est;
                    TP.Right(i)=right_est;
                    TP.Width(i)=width_est;
                    TP.Height(i)=height1;
                    TP.Ratio(i)=ratio_est;
                    TP.Area(i)=TP.Width(i)*TP.Height(i);
                    TP.x(i)=x_est;
                    TP.y(i)=y_est;
                    TP.Found(i)=TP.Found(i)+1; %counter of that vehicle increases
                    TP.Missing(i)=0; %not missing
                    TP.Flag(i)=TP.Flag(i)+1; %this element of TP has been found
                    PV.Flag(j)=PV.Flag(j)+1; %this element of PV has been found
                      %this element of TP has been found
                      
                end  

            end %end inner for

            if TP.Flag(i)==0  % (se non si trova il veicolo nel frame corrente e l'avevo trovato in precedenza faccio predizione pura)
                %if this vehicle of TP has not been found in the new frame,
                %hence if the new measurement doesn't match with the kalman 
                %estimate, the new estimate is updated and the missing
                %of that vehicle increments
                
                %in this case the previously calculated estimate is 
                %assigned to the state measurements 
                state_meas(1)=state_est(1,i);
                state_meas(2)=state_est(2,i);
                state_meas(3)=state_est(3,i);
                state_meas(4)=state_est(4,i);
                state_meas(5)=state_est(5,i);
                
                %also the new estimate state has the previously calculated
                %estimate to the state measurements: so in this case since
                %no measurements are available, the state is left free to 
                %evolve according to the model
                state_e(1)=state_est(1,i);
                state_e(2)=state_est(2,i);
                state_e(3)=state_est(3,i);
                state_e(4)=state_est(4,i);
                state_e(5)=state_est(5,i);

                %assigning covariance
                P=P_est(:,:,i);
                
                %calling kalman 
                [state_est(:,i),P_est(:,:,i)] = kalman3(state_e',P,state_meas');

                %calculating the new features
                x_est=state_est(1,i)*cos(state_est(3,i))+x_optical;
                y_est=state_est(1,i)*sin(state_est(3,i))+y_optical;
                width_est=state_est(4,i);
                height1=TP.Height(i);
                left_est=x_est-width_est/2;
                top_est=y_est-(height1)/2;
                right_est=left_est+width_est;
                bottom_est=top_est+height1;
                ratio_est=width_est/height1;

                %updating the kalman predicted features in the vector
                %of the stored vehicles (TP)
                TP.Top(i)=top_est;
                TP.Bottom(i)=bottom_est;
                TP.Left(i)=left_est;
                TP.Right(i)=right_est;
                TP.Width(i)=width_est;
                TP.Height(i)=height1;
                TP.Ratio(i)=ratio_est;
                TP.Area(i)=TP.Width(i)*TP.Height(i);
                TP.x(i)=x_est;
                TP.y(i)=y_est;
                TP.Missing(i)=TP.Missing(i)+1; %missing counter of that 
                                               %vehicle increases 
            end

         end %end outer for
%%%%%%%%%%%%%% A1)END UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         

         
%%%%%%%%%%%%%% A2) APPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %looking for that NEW vehicles that do not match any of the old ones
       %and adding that vehicle to the list of all vehicles (adding to TP)
       % veicoli nuovi mai trovati prima
        for i=1:n
            if PV.Flag(i)==0
                %creating a new univoque id number
                ID=ID+1;  % NB: l'ID serve solo per il plotting, può essere commentato!!
                %TP is equals to PV
                TP{end+1,:}=[PV{i,1:10},0,1,0,ID]; %adding new vehicle
                %initialization of the flag
                TP.Flag(end)=0;
                %this vehicle it is founded for the first time
                TP.Found(end)=1;    %counter found =1
                %not missing
                TP.Missing(end)=0;  %counter missing =0
                
                %kalman initialization
                state_est(:,end+1)=[(sqrt((PV.x(i)-x_optical)^2+(PV.y(i)-y_optical)^2));
                                    0;
                                    (atan2(((PV.y(i))-y_optical),(PV.x(i)-x_optical)));
                                    PV.Width(i)';
                                    0];

                 P_est(:,:,end+1)=P0;
                  
                state_e=state_est(:,end);
            %of course the measurement vector in this case can only be 
            %equal to the estimate vector since so far we only have 1 datum
                state_meas=state_e;

                %calculating the first prediction of kalman based only on
                %the model since it is the first one
                [state_est(:,end),P_est(:,:,end)] = kalman3(state_e,P0,state_meas);

            end
        end
%%%%%%%%%%%%%% A2)END APPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

%%%%%%%%%%%%%% A3) DISAPPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %if an old tracked vehicle (TP) does not track any potential vehicle
       %(PV) for more than "missing_vehicles" consecutive frames, than this
       %vehicle is removed from the tracking 
       i=1;
       while i<=height(TP)   %i inspect all the TP vector
            if TP.Missing(i)>=missing_vehicles  % missing_vehicles: numero di frame mancanti necessari per rimuovere un veicolo
                %it is necessary to delete the rows of TP and in parallel the same
                %rows of the kalman matrices since they are parallel structure: at
                %the same rows there are data relatively at the same vehicle
                TP(i,:)=[]; %removing that vehicle
                %one a vehicle is removed (or added) from (to) TP, in
                %parallel it is removed (or added) from (to) the kalman
                %state vector and kalman covariance matrix, so that one
                %vehicle has the same position in both kalman structures
                %and TP
                state_est(:,i)=[];
                P_est(:,:,i)=[];
                i=i-1;
            end
            i=i+1; 
       end
%%%%%%%%%%%%%% A3)END DISAPPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

%%%%%%%%%%%%%% A-REFINEMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %REFINEMENT: removing two boundingboxes that overlap, keeping just 
       %the one that has been found more times
       del=[];
       i=1;
       while i<=(height(TP)-1)
            j=i+1;
            while j<=height(TP)
                
                    %calculating the overlapping of two pairs
                    Hij = min(TP.Bottom(j),TP.Bottom(i))-max(TP.Top(j),TP.Top(i));  
                    Wij = min(TP.Right(j),TP.Right(i))-max(TP.Left(j),TP.Left(i));
                    Aij = Hij*Wij; %common area between boundingboxes of two different frames
                    Overlap_ij=Aij/max(TP.Area(j),TP.Area(i)); % overlapping area (%)
                    
                    %if the overlapping area is positive (with both
                    %horizontal and vertical distance positive, hence with
                    %a real overlapping; in fact an overlapping>0 can be
                    %obtained with both H and W negative but in that case
                    %it wouldn't be an actual overlapping)
                    if Overlap_ij>0 && Hij>0 && Wij>0 
                        
                        %one box must have more "Found" than the other +
                        %a threshold in order to delete another box
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
       %it is necessary to delete the rows of TP and in parallel the same
       %rows of the kalman matrices since they are parallel structure: at
       %the same rows there are data relatively at the same vehicle
        TP(del,:)=[];
        state_est(:,del)=[];
        P_est(:,:,del)=[];
%%%%%%%%%%%%%% END A-REFINEMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        
%%%%%%%%%%%%%% END SECTION A %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     
else %if no tracked vehicle is present (TP empty)
%%%%%%%%%%%%%% SECTION B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if height(PV)>0 %if vehicles are found in current frame (PV not empty)
        TP=PV; %TP is equal to PV because the are no stored vehicles
        TP.Found=ones(height(PV),1); %all the new vehicles are found once
        TP.Missing=zeros(height(PV),1);%all the new vehicles aren't missing
        TP.ID=(1:1:height(PV))'; %assigning an univoque id number
        ID=TP.ID(end); %updating the next expendable id for the successive 
                       %vehicle that appears

        %kalman initialization
        state_est(1,1:height(PV))=(sqrt((PV.x-x_optical).^2+(PV.y-y_optical).^2))';
        state_est(2,1:height(PV))=zeros(1,height(PV));
        state_est(3,1:height(PV))=(atan2(((PV.y)-y_optical),(PV.x-x_optical)))';
        state_est(4,1:height(PV))=PV.Width';
        state_est(5,1:height(PV))=zeros(1,height(PV));
        
        
        %since the elements of TP are all new vehicles, the kalman estimate
        %for each of them must be computed 
        for i=1:height(PV)
            state_e=state_est(:,i);
            %of course the measurement vector in this case can only be 
            %equal to the estimate vector since so far we only have 1 datum
            state_meas=state_e;
            [state_est(:,i),P_est(:,:,i)] = kalman3(state_e,P0,state_meas);
        end
    else %if vehicles are not found in current frame (PV empty)
        ID=0;%resetting the identification number since there are neither 
             %tracked vehicles nor new vehicles
    end
    
%%%%%%%%%%%%%% END SECTION B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end %end if 


    %building the structure used for plotting the bounding box of the
    %vehicles: it will be a matrix where in each row the bounding box
    %features are present( (x,y) of the upper left corner of the bb, width
    %and height of the bounding box)
    if height(TP)>0
    bbox=[TP.Left,TP.Top,TP.Width,TP.Height];
    else
        bbox=[];
    end
    
    
    
    
    
end %end function





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


%structure of TP:
%1:10 as PV
%11 flag of TP
%12 found (counter that sais how many times this old vehicle has been found
%          in the new frame)
%13 missing (counter that sais how many times this old vehicle is missing
%            in the last frames)