function [couples]=SingleFrameCoupleAnalysis(linesl,linesr,scoreL,scoreR,MdlStd)
%This function takes the left and right lines to create couples to which a
%score is given, this score defines if the couple of lines is a feasible
%couple for the lane

        

%INPUT
    %linesl                 contains the information of all the left lines
                            %found and preprocessed
    %linesr                 contains the information of all the right lines
                            %found and preprocessed
    %scoreL                 contains the score given by the SVM to each one
                            %of the left lines, this score defines if the
                            %line is continuos (>0) or discontinuous (<0)
    %scoreR                 contains the score given by the SVM to each one
                            %of the right lines, this score defines if the
                            %line is continuos (>0) or discontinuous (<0)
    %MdlStd                 %This variable contains the structure of the
                            %SVM trained to define if a couple of lines is
                            %correct as lane lines or not.
  
%OUTPUT
    %couples                %Variable containing the information of all the
                            %couples found in a single frame
                            
    
    
      %STRUCTURE OF THE LINES
           %lines.rho
           %lines.theta
           %lines.point1            [x y] coordinates of the top point of
                                    %the line
           %lines.point2            [x y] coordinates of the bottom point
                                    %of the line
      %STRUCTURE OF THE couples

%each row of the matrix couple contains the following information
%couples = [theta_LEFT      rho_LEFT    score_continuous_LEFT ...
           %theta_RIGHT     rho_RIGHT   score_continuous_RIGHT...
           %sum_theta       sum_rho     score_couple];
                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lll=length(linesl); %amount of lines found on the left
rrr=length(linesr); %amount of lines found on the right

couples=zeros(lll*rrr,9); %matrix of couples empty with the size of the
                          %all posible couples found in the frame

%each row of the vector couple contains the following information
%couples = [theta_LEFT      rho_LEFT    score_continuous_LEFT ...
           %theta_RIGHT     rho_RIGHT   score_continuous_RIGHT...
           %sum_theta       sum_rho     score_couple];

%this cycles are used to compute the information of the couples of lines
for i=1:lll

    for j=1:rrr
        sum_theta= linesl(i).theta+linesr(j).theta; %feature1 -> theta_Left+theta_Right
        sum_rho= linesl(i).rho+linesr(j).rho; %feature2 -> Rho_left+Rho_Right
        Data=[sum_theta, sum_rho]; %Features vector of the couple -> feature vector
        [~,score]=predict(MdlStd,Data); %classification of the couple (SVM)
        
        c=[linesl(i).theta,linesl(i).rho,scoreL(i),...
           linesr(j).theta,linesr(j).rho,scoreR(j),...
           sum_theta, sum_rho, score(2)]; %information of the single couple
        couples((i-1)*rrr+j,:)=c;
    end

end


end

