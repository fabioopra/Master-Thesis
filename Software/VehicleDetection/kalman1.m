function [bbox,state_est,P_est,width1,height1] = kalman1(TP,state_est,P_est,width1,height1)

[m,n]=size(TP);
%%%%%%MODEL1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state: [x, y, vx, vy]'
% equation: x(t)=x(t-1)+t*v(t-1)
t=1.5;                                                      
%state space model 1: constant speed
A=[1 0 t 0;
   0 1 0 t;
   0 0 1 0;
   0 0 0 1];
C=[1 0 0 0;                 
   0 1 0 0];

%state covariance
Q=[t^4/4   0   t^3/2   0;   
    0    t^4/4   0   t^3/2;
   t^3/2   0    t^2    0;
    0    t^3/2   0    t^2]*0.01;
%output covariance
ex=10;
ey=50;
R=[ex 0;  
   0 ey];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%measurements of the state
state_meas(1,:)=TP.x;
state_meas(2,:)=TP.y;
state_meas(3:4,:)=zeros(2,m); %all the vx and vy are initialized at 0
seen=TP.Found;
missing=TP.Missing;


L=[];

%%%%%%KALMAN FOR EACH VEHICLE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:length(seen)
       
 %initializiation
            if seen(i)==1 && missing(i)==0
                P(:,:,i)=Q;    %initial covariance of prediction error
                state_est(:,i)=state_meas(:,i);
                width1(i)=TP.Width(i);
                height1(i)=TP.Height(i);
            else
                if missing(i)>=1
                    state_meas(1,i)=state_est(1,i);
                    state_meas(2,i)=state_est(2,i);
                else
                    width1(i)=0.1*TP.Width(i)+0.9*width1(i);
                    height1(i)=0.1*TP.Height(i)+0.9*height1(i);
                end
                P(:,:,i)=P_est(:,:,i);
                state_meas(3,i)=state_est(3,i);
                state_meas(4,i)=state_est(4,i);
            end 
           
            
            %initialization of the measured state
            state(:,i)=[state_meas(1,i);state_meas(2,i);state_meas(3,i);state_meas(4,i)];
            
        %model 2
        % x_old(i)=bbox.x(i); %real position x at t-2
        % y_old(i)=bbox.y(i); %real position y at t-2
        % state_old(:,i)=A*[x_old(i);y_old(i);vx_old(i);vy_old(i)];

        L(:,:,i)=A*P(:,:,i)*C'*inv(C*P(:,:,i)*C'+R); %kalman gain
        state_est_new(:,i)=A * state_est(:,i) + L(:,:,i) * (C*state(:,i) - C*state_est(:,i));  %state update
        Pnew(:,:,i)=A*P(:,:,i)*A'+Q-L(:,:,i)*C*P(:,:,i)*A'; %covariance update

        state_est(:,i)=state_est_new(:,i); %function output: state prediction
        P_est(:,:,i)=Pnew(:,:,i); %function output: covariance prediction
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%row vectors
left=state_est(1,:)-width1'/2;
top=state_est(2,:)-height1'/2;


bbox=[left',top',width1,height1];

end