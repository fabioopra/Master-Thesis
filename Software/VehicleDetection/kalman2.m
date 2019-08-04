


function [bbox,state_est,P_est,width1,height1] = kalman2(TP,state_est,P_est,width1,height1,x_optical,y_optical)

[m,n]=size(TP);
%%%%%%MODEL1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state: [x, y, vx, vy width width_speed]'
% equations: x(t)=x(t-1)+t*v(t-1)
%            y(t)=y(t-1)+t*v(t-1)
%            vx(t)=vx(t-1)
%            vy(t)=vx(t-1)*y/x
%            w(t)=w(t-1)+t*vw(t-1)
%            vw(t)=vw(t-1)
t=2;                                

%measurements of the state
state_meas(1,:)=TP.x-x_optical;
state_meas(2,:)=TP.y-y_optical;
state_meas(5,:)=TP.Width;
state_meas(3:4,:)=zeros(2,m);%all the vx and vy are initialized at 0
state_meas(6,:)=zeros(1,m);%all the vd are initialized at 0
seen=TP.Found;
missing=TP.Missing;


L=[];

C=[1 0 0 0 0 0;                 
   0 1 0 0 0 0;
   0 0 0 0 1 0];

%state covariance
Q=[t^4/4   0    t^3/2   t^3/2  0     0;   
    0    t^4/4    0     t^3/2  0     0;
   t^3/2   0     t^2     t^2   0     0;
   t^3/2 t^3/2   t^2     t^2   0     0;
     0     0      0       0  t^4/4*0.1 t^3/2*0.1;
     0     0      0       0  t^3/2*0.1  t^2*0.1]*0.01;
%output covariance
ex=5;
ey=20;
ed=20;
R=[ex 0 0;  
   0 ey 0;
   0  0 ed];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%KALMAN FOR EACH VEHICLE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:length(seen)
       
 %initializiation
            if seen(i)==1 && missing(i)==0
                P(:,:,i)=Q;    %initial covariance of prediction error
                state_est(:,i)=state_meas(:,i);
                width1(i)=TP.Width(i);
                height1(i)=TP.Height(i);
                
                %checking if the x of the detected vehicle is in 0 and
                %changing 0 in 1 (only for the first frame)
                if state_est(1,i)==0
                    state_est(1,i)=1;
                end
                
            else
                
                %if x is 0 in a frame different from the first one
                if state_est(1,i)==0
                    %if vx>=0 => x=1
                    if state_est(3,i)>=0
                        state_est(1,i)=1;
                        
                    %if vx<0 => x=-1
                    elseif state_est(3,i)<0
                        state_est(1,i)=-1;
                    end
                end 
                
                
                if missing(i)>=1
                    state_meas(1,i)=state_est(1,i);
                    state_meas(2,i)=state_est(2,i);
                    state_meas(5,i)=state_est(5,i);
                else
                    width1(i)=0.5*TP.Width(i)+0.5*width1(i);
                    height1(i)=0.1*TP.Height(i)+0.9*height1(i);
                end
                P(:,:,i)=P_est(:,:,i);
                state_meas(3,i)=state_est(3,i);
                state_meas(4,i)=state_est(4,i);
                state_meas(6,i)=state_est(6,i);
            end 
           
            
            %state space model 2: constant speed 2
            a41=-state_est(2,i)*state_est(3,i)/(state_est(1,i)^2);
            a42=state_est(3,i)/state_est(1,i);
            a43=state_est(2,i)/state_est(1,i);
            A=[1    0    t    0  0  0;
               0    1    0    t  0  0;
               0    0    1    0  0  0;
               a41 a42  a43   0  0  0
               0    0    0    0  1  1
               0    0    0    0  0  1];
           
           f=[state_est(1,i)+state_est(3,i)*t;
              state_est(2,i)+state_est(4,i)*t;
              state_est(3,i);
              state_est(2,i)*state_est(3,i)/state_est(1,i);
              state_est(5,i)+state_est(6,i)*t;
              state_est(6,i)];
           
            %initialization of the measured state
            state(:,i)=[state_meas(1,i);state_meas(2,i);state_meas(3,i);state_meas(4,i);state_meas(5,i);state_meas(6,i)];
            

        L(:,:,i)=A*P(:,:,i)*C'*inv(C*P(:,:,i)*C'+R); %kalman gain
        state_est_new(:,i)=f + L(:,:,i) * (C*state(:,i) - C*state_est(:,i));  %state update
        Pnew(:,:,i)=A*P(:,:,i)*A'+Q-L(:,:,i)*C*P(:,:,i)*A'; %covariance update

        state_est(:,i)=state_est_new(:,i); %function output: state prediction
        P_est(:,:,i)=Pnew(:,:,i); %function output: covariance prediction
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%row vectors
left=state_est(1,:)-state_est(5,:)/2+x_optical;
top=state_est(2,:)-height1'/2+y_optical;


bbox=[left',top',state_est(5,:)',height1];

end