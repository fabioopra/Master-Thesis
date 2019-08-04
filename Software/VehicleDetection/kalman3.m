%this function take as inputs all the data of a SINGLE vehicle and computes
%the kalman estimate of that vehicle

%INPUTS:
%state_e     previous state vector of this vehicle
%P           previous covariance matrix of this vehicle
%state_meas  measurement vector with new data (it comes from the PV data)

%OUTPUTS:
%state_e     updated state vector of this vehicle
%Pnew        updated covariance matrix of this vehicle

function [state_e_new,Pnew] = kalman3(state_e,P,state_meas)

%%%%%%MODEL3%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Il Modello viene riassegnato tutte le volte ma è sempre costante!!!!!!

% p=rho: distance of the box from the centre of the image
% vp=velocity of rho
% theta=angle of the bounding box
% w=width of the bounding box
% vw=velocity of the width of the bb

% state: [p, vp, theta, width width_speed]'
% equations: p(t)=p(t-1)+t*vp(t-1)
%            vp(t)=vp(t-1)
%            theta(t)=theta(t-1)
%            w(t)=w(t-1)+t*vw(t-1)
%            vw(t)=vw(t-1)
t=2;                                

L=[];

A=[1 t 0 0 0;
   0 1 0 0 0;
   0 0 1 0 0;
   0 0 0 1 t;
   0 0 0 0 1];

C=[1 0 0 0 0;     %p            
   0 0 1 0 0;     %theta
   0 0 0 1 0];    %width

%state covariance
var_ap=0.01; %variance: rho acceleration
var_aw=0.001; %variance: width acceleration
var_theta=0.01; %variance: theta acceleration

Q=[t^4/4*var_ap    t^3/2*var_ap      0          0              0;   
   t^3/2*var_ap     t^2*var_ap       0          0              0;
      0                 0        var_theta      0              0;
      0                 0            0     t^4/4*var_aw    t^3/2*var_aw;
      0                 0            0     t^3/2*var_aw     t^2*var_aw];
  
%output covariance
ep=50;
etheta=0.1;
ew=50;
R=[ep  0     0;  
   0 etheta  0;
   0   0     ew];

%% Fine del modello di Kalman


%%%%%%KALMAN EQUATIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L=A*P*C'*inv(C*P*C'+R); %kalman gain
state_e_new=A * state_e + L * (C*state_meas - C*state_e);  %state update
Pnew=A*P*A'+Q-L*C*P*A'; %covariance update


end