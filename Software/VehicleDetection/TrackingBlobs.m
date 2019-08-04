function [ Features_old,state_estB,P_estB] = TrackingBlobs( Features_old, Features,Ph_thresh, Pv_thresh, ratio_h_thresh,ratio_w_thresh,ratio_A_thresh,...
    missingblob,state_estB,P_estB,x_optical,y_optical,P0B)

%This function tracks the single blobs: "Features_old" is a table 
%(similarly to TP for the pairs) that contains all the tracked blobs. It is
%updated by the new data coming from "Features" (as PV for pairs).
%"Features" is the same table used at the beginning of the algorithm with
%all the connected components. In this table a lot of blobs are already
%deleted in previous steps: initially all the very small blobs are deleted,
%in function "Pairing" the paired blobs are deleted, 
%in function "PossibleVehicles" all the blobs overlapping with the
%tracked pairs (TP) are deleted. The resulting blobs are the ones to be
%tracked beacuse they should not correspond to disturbances

%INPUT
%     Features_old          Table that contains the features of the tracked
                            %blobs
%     Features              Table that contains the features of the blobs
                            %that are left after the pairing
%     Ph_thresh             Threshold for the horizontal overlap
%     Pv_thresh             Threshold for the vertical overlap
%     ratio_h_thresh        Threshold for the ratio of the heights
%     ratio_w_thresh        Threshold for the ratio of the width
%     ratio_A_thresh        Threshold for the ratio of the Area
%     missingblob           Amount of consecutive frames that a blob in
                            %memory has not been matched before deleting it
%     state_estB            matrix with kalman states on each column
%     P_estB                three-dimensional matrix containing all the
                            %covariance matrices for each tracked blob
%     x_optical             x coordinate of the optical centre
%     y_optical             y coordinate of the optical centre
%     P0B                   initialization of kalman covariance matrix

%OUTPUT
%     Features_old          Updated Table that contains the features of the
                            %tracked blobs
%     state_estB            matrix with kalman states on each column
%     P_estB                three-dimensional matrix containing all the
                            %covariance matrices for each tracked blob
                            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controllo se la matrice dei blob trackati è vuota
% Se si -> Section A, assegno tutte le features dei blob trovati come se
% fossero da trackare e inizializzo kalman
   
if isempty(Features_old)
%%%%%%%%%%%%%% SECTION A %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      Features_old=Features;
      if height(Features_old) > 0
          Features_old.found=ones(height(Features_old),1);
          Features_old.missing=zeros(height(Features_old),1);
          
          %kalman initialization
          state_estB(1,1:height(Features))=(sqrt((Features.x-x_optical).^2+(Features.y-y_optical).^2))';
          state_estB(2,1:height(Features))=zeros(1,height(Features));
          state_estB(3,1:height(Features))=(atan2(((Features.y)-y_optical),(Features.x-x_optical)))';
            

            
            for i=1:height(Features)
                state_e=state_estB(:,i);
                %of course the measurement vector in this case can only be 
                %equal to the estimate vector since so far we only have 1 datum
                state_meas=state_e;
                [state_estB(:,i),P_estB(:,:,i)] = kalman3B(state_e,P0B,state_meas);

            end
      end
%%%%%%%%%%%%%% END SECTION A %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

else
%%%%%%%%%%%%%% SECTION B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% Tracking vero e proprio

      i=1;
      while i <= height(Features_old)
          flag=0; %used to know if a blob has been matched
          q=1;
          while q <= height(Features)
              
              Dh=max(Features_old.left(i),Features.left(q))-min(Features_old.right(i),Features.right(q));
              %horizontal distance
              Dv=max(Features_old.top(i),Features.top(q))-min(Features_old.bottom(i),Features.bottom(q));
              %vertical distance
              
              Ph = -Dh/(min(Features_old.width(i),Features.width(q)));
              %horizontal overlap
              Pv = -Dv/(min(Features_old.height(i),Features.height(q)));
              %vertical overlap
              
              ratio_h = min(Features_old.height(i),Features.height(q))/max(Features_old.height(i),Features.height(q));
              % ratio between height of boxes
        
              ratio_A=min(Features_old.Area(i),Features.Area(q))/max(Features_old.Area(i),Features.Area(q));
              %ratio between Area of bright elements
        
              ratio_w=min(Features_old.width(i),Features.width(q))/max(Features_old.width(i),Features.width(q));
              %ratio between width of boxes
              
              
              if Ph>Ph_thresh && Pv>Pv_thresh && ratio_h>ratio_h_thresh && ratio_A>ratio_A_thresh && ratio_w>ratio_w_thresh
%%%%%%%%%%%%% B1) UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  
                  
                    state_meas(1)=sqrt((Features.x(q)-x_optical)^2+(Features.y(q)-y_optical)^2);
                    state_meas(2)=0;
                    state_meas(3)=atan2((Features.y(q)-y_optical),(Features.x(q)-x_optical));
                
                    
                    state_e(1)=state_estB(1,i);
                    state_e(2)=state_estB(2,i);
                    state_e(3)=state_estB(3,i);

                    
                    P=P_estB(:,:,i);
                    
                    %kalman predicts the future position based on the new
                    %measurement 
                    [state_estB(:,i),P_estB(:,:,i)] = kalman3B(state_e',P,state_meas');
                    
                    
                    x_est=state_estB(1,i)*cos(state_estB(3,i))+x_optical;
                    y_est=state_estB(1,i)*sin(state_estB(3,i))+y_optical;
                    width1B=0.1*Features.width(q)+0.9*Features_old.width(i);
                    height1B=0.1*Features.height(q)+0.9*Features_old.height(i);
                    left_est=x_est-width1B/2;
                    top_est=y_est-height1B/2;
                    right_est=left_est+width1B;
                    bottom_est=top_est+height1B;

                    %updating the kalman predicted features in the vector
                    %of the stored vehicles (TP)
                    Features_old.top(i)=top_est;
                    Features_old.bottom(i)=bottom_est;
                    Features_old.left(i)=left_est;
                    Features_old.right(i)=right_est;
                    Features_old.width(i)=width1B;
                    Features_old.height(i)=height1B;
                    Features_old.Area(i)=Features.Area(q);
                    Features_old.x(i)=x_est;
                    Features_old.y(i)=y_est;                
                    Features_old.BoundingBox(i,:)=[left_est top_est width1B height1B];
                    
                    Features_old.found(i)=Features_old.found(i)+1;
                    %increase the amount of frames in which it has been found
                    Features_old.missing(i)=0;
                    %restart the amount of frames in which it has been missed
                    Features(q,:)=[];
                    flag=1;
                    q=q-1;
              end
              q=q+1;
          end %end inner while
          
          if flag ==0
              %blob was not matched in the current frame
              
                    state_meas(1)=state_estB(1,i);
                    state_meas(2)=state_estB(2,i);
                    state_meas(3)=state_estB(3,i);
                
                    
                    state_e(1)=state_estB(1,i);
                    state_e(2)=state_estB(2,i);
                    state_e(3)=state_estB(3,i);

                    
                    P=P_estB(:,:,i);
                    
                    %kalman predicts the future position based on the new
                    %measurement 
                    [state_estB(:,i),P_estB(:,:,i)] = kalman3B(state_e',P,state_meas');
                    
                    
                    x_est=state_estB(1,i)*cos(state_estB(3,i))+x_optical;
                    y_est=state_estB(1,i)*sin(state_estB(3,i))+y_optical;
                    width1B=Features_old.width(i);
                    height1B=Features_old.height(i);
                    left_est=x_est-width1B/2;
                    top_est=y_est-height1B/2;
                    right_est=left_est+width1B;
                    bottom_est=top_est+height1B;

                    %updating the kalman predicted features in the vector
                    %of the stored vehicles (TP)
                    Features_old.top(i)=top_est;
                    Features_old.bottom(i)=bottom_est;
                    Features_old.left(i)=left_est;
                    Features_old.right(i)=right_est;
                    Features_old.width(i)=width1B;
                    Features_old.height(i)=height1B;
%                     Features_old.Area(i)=Features.Area(q);
                    Features_old.x(i)=x_est;
                    Features_old.y(i)=y_est; 
                    Features_old.BoundingBox(i,:)=[left_est top_est width1B height1B];
                    Features_old.missing(i)=Features_old.missing(i)+1;
          %increase the amount of frames in which this blob has been missed
                    
%%%%%%%%%%%%% B1) END UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
                

%%%%%%%%%%%%% B2) DISAPPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              %DISAPPEAR
              if Features_old.missing(i) >= missingblob
                 %if missed for more than 'missingblob' it is deleted
                 Features_old(i,:)=[];
                 state_estB(:,i)=[];
                 P_estB(:,:,i)=[];
                 i=i-1;
              end
%%%%%%%%%%%%% B2) END DISAPPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          end 
          
          i=i+1;
      end %end outer while
      
%%%%%%%%%%%%%% END SECTION B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
end


%%%%%%%%%%%%% C) APPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %APPEAR
   %the blobs in the current frame that has not been matched are kept in
   %memory
  if height(Features) > 0
      for i=1:height(Features)
          Features_old{end+1,:}=[Features{i,:},1,0];
          %features found=1 missing=0
          
          state_estB(:,end+1)=[(sqrt((Features.x(i)-x_optical)^2+(Features.y(i)-y_optical)^2));
                                    0;
                                    (atan2(((Features.y(i))-y_optical),(Features.x(i)-x_optical)))];

            t=2;
            var_ap=0.01;
            var_theta=0.1;
            P0B=[t^4/4*var_ap    t^3/2*var_ap      0;   
               t^3/2*var_ap     t^2*var_ap       0;
                  0                 0        var_theta];
             P_estB(:,:,end+1)=P0B;

            state_e=state_estB(:,end);
           %of course the measurement vector in this case can only be 
           %equal to the estimate vector since so far we only have 1 datum
            state_meas=state_e;

            [state_estB(:,end),P_estB(:,:,end)] = kalman3B(state_e,P0B,state_meas);
      end
  end
%%%%%%%%%%%%% C) END APPEAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

