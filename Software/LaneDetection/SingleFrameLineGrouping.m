function [ lines, lines_length,ratio_covered,label,score,lines_total] = SingleFrameLineGrouping(  lines, WidthLaneLines,extra_length,n,l,y_optical,SVMStruct2)
%This function groups the lines that are close to each other and define if
%the line is continuous or discontinuos

        

%INPUT
    %lines                 contains the information of all the lines found
                           %and preprocessed
    %WidthLaneLines        maximum space between lines to be grouped in px
    %extra_length          %px of extension of the lines at the bottom of
                           %the image
    %n                     %vertical resolution of the image 1080
    %l                     %horizontal resolution of the image 1920
    %y_optical             %y-coordinate of the optical center
    %SVMStruct2            %Variable containing the parameters of the SVM
                           %whose task is to define if a line is continuous
                           %or not, this variable has an special structure
                           %and is the result of the training of a SVM
                           %using the function 'fitrsvm', this structure
                           %can be replaced by another one containing the
                           %parameters and bias of the linear kernel SVM,
                           %then the score must be computed as the distance
                           %to the hyperplane defined by this parametes.

%OUTPUT
    %lines                  Grouped lines
    
    %lines_length           This variable contains the maximum length of a
                            %group of lines sharing similar parameters
                            
    %ratio_covered          This variable contains the ratio between the
                            %filled length of the lines and the total
                            %length of the lines
                            
    %label                  This variable contains the labels result of the
                            %classification process using the SVM
                            % 1. CONTINUOUS
                            % 0. DISCONTINUOUS
                            
    %score                  %This varible contains the score given by the
                            %SVM to each single line after the
                            %classification process, if score > 0 it is in
                            %the side of continuous lines and if score < 0
                            %the line is in the side of the discontinuous
                            %lines, the greater the magnitude of score the
                            %more continuous or discontinuous.
                            
    %lines_total            Is an array containing at each element all the
                            %lines with similar parameters (rho,theta), (used to print)
                            
    
    
      %STRUCTURE OF THE LINES
           %lines.rho
           %lines.theta
           %lines.point1            [x y] coordinates of the top point of
                                    %the line
           %lines.point2            [x y] coordinates of the bottom point
                                    %of the line
                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%Grouping

              
    %in this part of the code the lines are grouped based on its position,
    %if two lines are horizontally overlapped and close, then the lines are
    %grouped, if more than one line are groupped the resulting line will
    %have the parameters of the weighted average by the length of the
    %grouped lines.

    i=1;
    while i <= length(lines)
            %this cycles compare each line with the remaining ones
            length_i=sqrt((lines(i).point1(1)-lines(i).point2(1))^2+(lines(i).point1(2)-lines(i).point2(2))^2); %Length of the i-line
            length_acumulator=length_i; %Accumulates the length of all the 
            %lines grouped with the i-line, in order to computed the
            %weighted average at the end
            %i-line parameters computation, during the development
            %process it have been shown matematically that the average line
            %is better computed using the (m,b) average rather than the
            %(rho,theta) average.
            m_i=-cot(lines(i).theta*pi/180); %slope of the i line
            b_i=lines(i).rho/sin(lines(i).theta*pi/180);%intersect of the i
            %line
            m_acumulator=m_i*length_i; %accumulates the m parameter of the 
            %lines similar to the i-line
            b_acumulator=b_i*length_i; %accumulates the b parameter of the
            %lines similar to the i-line


            i_point1y=lines(i).point1(2) - extra_length*((lines(i).point1(2)-y_optical)/(n/2));%%
            %y-coordinate of the extendet upper point of the i-line
            %'extra_length' is weighted by the perspective term
            %((lines(i).point1(2)-y_optical)/(n/2)) the closer to the
            %optical center the lower the extension.

            i_point1x=(lines(i).rho-i_point1y*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
            %x-coordinate of the i-line at the height of the previous found
            %y-coordinate
            i_point2y=lines(i).point2(2) + extra_length*((lines(i).point2(2)-y_optical)/(n/2));%%
            i_point2x=(lines(i).rho-i_point2y*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
            %similar computations used to extend the lower point of the
            %line

            %This extension was done in order to group Lines that are not
            %horizontally overlapped but that are really close in the
            %vertical direction
    
            %this cycle compares the lines with the i-line
            pos=0;
            del=[];
            q=1;
            while q <= length(lines)
                if q==i
                    q=q+1;
                    if q > length(lines)
                        break;
                    end
                end
                q_point1y=lines(q).point1(2) - extra_length*((lines(q).point1(2)-y_optical)/(n/2));%%
                q_point1x=(lines(q).rho-q_point1y*sin(lines(q).theta*pi/180))/cos(lines(q).theta*pi/180);
                q_point2y=lines(q).point2(2) + extra_length*((lines(q).point2(2)-y_optical)/(n/2));%%
                q_point2x=(lines(q).rho-q_point2y*sin(lines(q).theta*pi/180))/cos(lines(q).theta*pi/180);
                %extension of the q-line
            %There are 4 cases in which the two lines must be grouped:
              
            %First condition:  
                % the TOP point of the i-line is ABOVE the TOP point of the
                % q-line AND the BOTTOM point of the i-line is BELOW the 
                % TOP point of the q-line

                if i_point1y <= q_point1y && i_point2y > q_point1y   
                            
                    % Additionally to the first condition
                    % FIRST CASE: if the BOTTOM point of the i-line is
                    % ABOVE the BOTTOM point of the q-line
                    
       %             * -> Top Point
       %             |   
       %  i-line ->  |          q-line -> * -> Top Point
       %             |                    |
       %             |                    |
       %             |                    |
       %             * -> bottom point    |
       %                                  |
       %                                  * -> bottom Point
                    
                    if i_point2y <= q_point2y
                        
                %With this conditions it can be said that the common
                %segment in the vertical direction is between the top point
                %of the q-line 'y1' and the bottom point of the i-line 'y2'
                        y1=q_point1y; %top point
                        y2=i_point2y; %bottom point

                        % computes the x coordinate at the height y1 and y2 for
                        % the i-line
                        xi1=(lines(i).rho-y1*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                        xi2=i_point2x;
                        
                        % computes the x coordinate at the height y1 and y2 for
                        % the q-line 
                        xj1=q_point1x;
                        xj2=(lines(q).rho-y2*sin(lines(q).theta*pi/180))/cos(lines(q).theta*pi/180);
                        
                        if abs(xi1-xj1) < WidthLaneLines*y1/n && abs(xi2-xj2) < WidthLaneLines*y2/n
                        % if the lines are close, in other words if the 
                        % distance between its points is smaller than the 
                        % width of a lane line 'WidthLaneLines'.
                        % Using the perspective factor 'y/n', the width is
                        % computed at the height of the desired points.            
                                    
                                    length_q=sqrt((lines(q).point1(1)-lines(q).point2(1))^2+(lines(q).point1(2)-lines(q).point2(2))^2); %lenght of the q-line
                                    length_acumulator=length_acumulator+length_q;
                                    %in order to group the lines a weighted
                                    %average is computed. The weight of
                                    %each line is the length of each line
                                    
% Therefore m = (m1*Length1+...+mn*Lengthn)/(Length1+...+Lengthn) weighted
% average.
                                    %then the length of the q-line is
                                    %computed and acumulated in the
                                    %variable length_acumulator
                                    m_q=-cot(lines(q).theta*pi/180);
                                    %m parameter of the q-line
                                    b_q=lines(q).rho/sin(lines(q).theta*pi/180);
                                    %b parameter of the q-line
                                    m_acumulator=m_acumulator+m_q*length_q;
                                    b_acumulator=b_acumulator+b_q*length_q;
                                    %The q-line parameters are accumulated
                                    %using the equation above mentioned
                                    
%Note: The weighted average is done using the parameters (m,b) instead of
%the parameters (rho, theta) since doing so, the resulting line is closer
%to the real street line rather than using the hough parameters, this last
%one have shown resulting lines a bit inclined with respect to the real
%average, mathematical computations can be done to probe this result.
                                    
                                    
                                    lines(i).point2(2)=lines(q).point2(2);
                                    lines(i).point2(1)=(lines(i).rho-lines(i).point2(2)*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    %line-i is emlarged to cover the whole
                                    %range covered previusly by the q-line
                                    %and i-line together
                                    i_point2y=lines(i).point2(2) + extra_length*((lines(i).point2(2)-y_optical)/(n/2));%%
                                    i_point2x=(lines(i).rho-i_point2y*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    %the extension points are computed
                                    %again for the enlarged i-line
                                    del=[del;q];
                                    if q < i
                                        pos=pos+1;
                                    end
                                    %lines(q)=[];
                                    %q=q-1;
                                    %the q-line is deleted from memory
                                    %since its information is stored in the
                                    %accumulators
                        end
                                
                    else
                
                        %Additionally to the first condition:
                        % SECOND CASE:if the BOTTOM point of the q-line is
                        % ABOVE the BOTTOM point of the i-line.
       %             * -> Top Point
       %             |   
       %  i-line ->  |          q-line -> * -> Top Point
       %             |                    |
       %             |                    |
       %             |                    |
       %             |                    |
       %             |                    |
       %             |                    * -> bottom Point
       %             * -> bottom point 
                
                    %With this conditions it can be said that the common
                    %segment in the vertical direction is between the top
                    %point of the q-line 'y1' and the bottom point of the
                    %q-line 'y2'
                        y1=q_point1y;%top point
                        y2=q_point2y;%bottom point

                        %computes the x coordinates at the heights y1 and y2
                        %for the i-line
                        xi1=(lines(i).rho-y1*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                        xi2=(lines(i).rho-y2*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                
                        %computes the x coordinates at the heights y1 and y2
                        %for the q-line
                        xj1=q_point1x;
                        xj2=q_point2x;
                        
                        %The computations are the same as in the previous
                        if abs(xi1-xj1) < WidthLaneLines*y1/n && abs(xi2-xj2) < WidthLaneLines*y2/n

                                    length_q=sqrt((lines(q).point1(1)-lines(q).point2(1))^2+(lines(q).point1(2)-lines(q).point2(2))^2);
                                    length_acumulator=length_acumulator+length_q;
                                    
                                    m_q=-cot(lines(q).theta*pi/180);
                                    b_q=lines(q).rho/sin(lines(q).theta*pi/180);
                                    m_acumulator=m_acumulator+m_q*length_q;
                                    b_acumulator=b_acumulator+b_q*length_q;
                                    
                                    del=[del;q];
                                    if q < i
                                        pos=pos+1;
                                    end
                                    %lines(q)=[];
                                    %q=q-1;
                                    
                                   
                        end
                    end
                %Second condition:
                    % the TOP point of the q-line is ABOVE the TOP point of the
                    % i-line AND the BOTTOM point of the q-line is BELOW
                    % the TOP point of the i-line.
            
                elseif q_point1y <= i_point1y && q_point2y > i_point1y 
            
                    
                    %Additionally to the second condition:
                    %THIRD CASE:
                    %if the BOTTOM point of the q-line is ABOVE
                    %the BOTTOM point of the i-line

       %                                  * -> Top Point
       %             * -> Top Point       |
       %             |                    |
       %  i-line ->  |          q-line -> | 
       %             |                    |
       %             |                    |
       %             |                    |
       %             |                    |
       %             |                    |
       %             |                    * -> bottom Point
       %             * -> bottom point 

                    if q_point2y <= i_point2y
                    
                    %With this conditions it can be said that the common
                    %segment in the vertical direction is between the top point
                    %of the i-line 'y1' and the bottom point of the q-line 'y2'
                                y1=i_point1y; % Top point
                                y2=q_point2y; % Bottom point
                                
                                xj1=(lines(q).rho-y1*sin(lines(q).theta*pi/180))/cos(lines(q).theta*pi/180);
                                xj2=q_point2x;
                                
                                xi1=i_point1x;
                                xi2=(lines(i).rho-y2*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                
                                if abs(xi1-xj1) < WidthLaneLines*y1/n && abs(xi2-xj2) < WidthLaneLines*y2/n
                                                                       
                                    length_q=sqrt((lines(q).point1(1)-lines(q).point2(1))^2+(lines(q).point1(2)-lines(q).point2(2))^2);
                                    length_acumulator=length_acumulator+length_q;
                                   
                                    m_q=-cot(lines(q).theta*pi/180);
                                    b_q=lines(q).rho/sin(lines(q).theta*pi/180);
                                    m_acumulator=m_acumulator+m_q*length_q;
                                    b_acumulator=b_acumulator+b_q*length_q;
                                    
                                    lines(i).point1(2)=lines(q).point1(2);
                                    lines(i).point1(1)=(lines(i).rho-lines(i).point1(2)*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    
                                    i_point1y=lines(i).point1(2) - extra_length*((lines(i).point1(2)-y_optical)/(n/2));%%
                                    i_point1x=(lines(i).rho-i_point1y*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    
                                    del=[del;q];
                                    if q < i
                                        pos=pos+1;
                                    end
                                    %lines(q)=[];
                                    %q=q-1;
                                    
                                    
                                end
                                
                    else
                    %Additionally to the second contidion
                    %FOURTH CASE:
                    %if the BOTTOM point of the i-line is
                    %ABOVE the BOTTOM point of the q-line

                
       %                                  * -> Top Point
       %             * -> Top Point       |
       %             |                    |
       %  i-line ->  |          q-line -> | 
       %             |                    |
       %             |                    |
       %             |                    |
       %             *  -> bottom point   |
       %                                  |
       %                                 * -> bottom Point
       %               

                    %With this conditions it can be said that the common
                    %segment in the vertical direction is between the top
                    %point of the i-line 'y1' and the bottom point of the
                    %i-line 'y2'
                                y1=i_point1y; %Top point
                                y2=i_point2y; %Bottom point
                                
                                xj1=(lines(q).rho-y1*sin(lines(q).theta*pi/180))/cos(lines(q).theta*pi/180);
                                xj2=(lines(q).rho-y2*sin(lines(q).theta*pi/180))/cos(lines(q).theta*pi/180);
                                
                                xi1=i_point1x;
                                xi2=i_point2x;
                                
                                if abs(xi1-xj1) < WidthLaneLines*y1/n && abs(xi2-xj2) < WidthLaneLines*y2/n

                                      
                                    length_q=sqrt((lines(q).point1(1)-lines(q).point2(1))^2+(lines(q).point1(2)-lines(q).point2(2))^2);
                                    length_acumulator=length_acumulator+length_q;
                                   
                                    m_q=-cot(lines(q).theta*pi/180);
                                    b_q=lines(q).rho/sin(lines(q).theta*pi/180);
                                    m_acumulator=m_acumulator+m_q*length_q;
                                    b_acumulator=b_acumulator+b_q*length_q;
                                    
                                    lines(i).point1(2)=lines(q).point1(2);
                                    lines(i).point1(1)=(lines(i).rho-lines(i).point1(2)*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    lines(i).point2(2)=lines(q).point2(2);
                                    lines(i).point2(1)=(lines(i).rho-lines(i).point2(2)*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    
                                    i_point1y=lines(i).point1(2) - extra_length*((lines(i).point1(2)-y_optical)/(n/2));%%
                                    i_point1x=(lines(i).rho-i_point1y*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    i_point2y=lines(i).point2(2) + extra_length*((lines(i).point2(2)-y_optical)/(n/2));%%
                                    i_point2x=(lines(i).rho-i_point2y*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                                    
                                    del=[del;q];
                                    if q < i
                                        pos=pos+1;
                                    end
                                    %lines(q)=[];
                                    %q=q-1;
                                    
                                end
                    end
                                      
                end
                       
                 q=q+1;
            end
            
            
           %In this part the average line is computed
            if length_acumulator > length_i
            %If the i-line has been grouped at least once, the weighted
            %average is computed
                    m_i=(m_acumulator)/(length_acumulator);
                    b_i=(b_acumulator)/(length_acumulator);
                    
                    lines(i).theta= acot(-m_i)*180/pi;
                    %computation of the theta parameter of the new i-line
                    lines(i).rho= b_i*sin(lines(i).theta*pi/180);
                    %computation of the rho parameter of the new i-line
                    lines(i).point1(1)=(lines(i).rho-lines(i).point1(2)*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                    lines(i).point2(1)=(lines(i).rho-lines(i).point2(2)*sin(lines(i).theta*pi/180))/cos(lines(i).theta*pi/180);
                    %computation of the x-coordinate at the extreme points 
                    %of the new i-line, the y-coordinate used are the last
                    %values of the i-line
            end
            
            lines(del)=[];
            i=i-pos+1;
            %i=i+1;
    end

%%voting

    %in this part the lines that share similar parameters are considered
    %and just the one with higher length is kept, this step is used in the
    %following functions to define if the line is continuous or
    %discontinuous

    %In addition, the ratio between the filled length and the lenght of the
    %line is computed as an useful feature for the classification step
     
     lines_length=zeros(1,length(lines));
     length_accumulator=zeros(1,length(lines));
     ratio_covered=zeros(1,length(lines));
    lines_total=[];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i=1;
    while i<= length(lines)
        lines1=[];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        thetai=abs(lines(i).theta);
        rhoi=abs(lines(i).rho);
        %i-line's parameters
        lengthi=sqrt((lines(i).point1(1)-lines(i).point2(1))^2+(lines(i).point1(2)-lines(i).point2(2))^2);
        %length of the i-line
        lines_length(i)=lengthi;
        ymin=lines(i).point1(2); %initial Top point of the complete line
        ymax=lines(i).point2(2); %initial Bottom point of the complete line
        length_accumulator(i)=lengthi;
        %length of the i-line
        
        
        lines1=[lines1; lines(i)];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          q=i+1;
          while q<=length(lines)
                thetaj=abs(lines(q).theta);
                rhoj=abs(lines(q).rho);
                %q-line's parameters
                lengthj=sqrt((lines(q).point1(1)-lines(q).point2(1))^2+(lines(q).point1(2)-lines(q).point2(2))^2);
                lines_length(q)=lengthj;
                %length of the q-line

                if abs(thetai-thetaj) < 5 && abs(rhoi-rhoj) < 150
                %If two lines have similar theta, the line with the
                %higher length is kept and the other one is deleted
                    
                    length_accumulator(i)=length_accumulator(i)+lengthj;
                    ymin=min(ymin,lines(q).point1(2)); %Update of the Top point
                    ymax=max(ymax,lines(q).point2(2)); %Update of the Bottom point
                    %If a line is discontinuous the bottom point at the end
                    %will be the bottom most of all the bottom points of
                    %the same line group and the top point will be the
                    %topmost point of the same line group
                    
                    
                    lines1=[lines1; lines(q)];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    
                    if lengthi >= lengthj %i longer than q
                       lines(q)=[];
                       lines_length(q)=[];
                       length_accumulator(q)=[];
                       ratio_covered(q)=[];
                       q=q-1;
                       
                    else %q longer than i
                        lines(i)=lines(q);
                        lines_length(i)=lines_length(q);
                        %the parameters of q are saved on i and the q line
                        %is then deleted
                        lenghti=lengthj;
                        lines(q)=[];
                        lines_length(q)=[];
                        length_accumulator(q)=[];
                        ratio_covered(q)=[];
                        
                        q=q-1;
                    end
                end
                q=q+1;
          end
          %after all the lines have been compared the length of the line is
          %computed using its parameters and the y-coordinates of the
          %topmost and bottommost points
          xmin=(rhoi-ymin*sin(thetai*pi/180))/cos(thetai*pi/180);
          xmax=(rhoi-ymax*sin(thetai*pi/180))/cos(thetai*pi/180);
          %The total length is computed
          total_length=sqrt((xmin-xmax)^2+(ymin-ymax)^2);
          %The ratio filled/length is the ratio between the acumulated
          %length of all the lines and the total length of the group of
          %lines
          ratio_covered(i)=length_accumulator(i)/total_length;
          %In this case the length of the line is expanded using the
          %perspective, since a line close to the optical center with the
          %same length of a line at the bottom contains more information,
          %then the length of the line close to the optical center must be
          %enlarged.
          %                                 Initial length
          %Length = -----------------------------------------------------------------
          %                Distance of the bottom point to the optical center
          %            ( -------------------------------------------------------- )
          %              Distance of the corner of the image to the optical center
          lines_length(i)=lines_length(i)/(sqrt((lines(i).point2(1)-l/2)^2+(lines(i).point2(2)-n/2)^2)/sqrt((n/2)^2+(l/2)^2));
          
          lines_total{i}=lines1;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
          i=i+1;
    end
    
    %Once all the features of the lines have been computed, the
    %classification step is done using the SVM trained in advance
   
    NewData=[lines_length' ratio_covered']; %feature vector where each row
    %is a line and each column is a feature, in this case the expanded
    %length and the ratio between the filled part and the total length
   
    [label, Sc]=predict(SVMStruct2,NewData); %classification step
    %label is 1 for continuous and 0 for discontinuous
    %Sc is a matrix with as much rows as lines and two columns containing
    %in the second column the score positive for continuous and negative 
    %for discontonuous and in the first row the same but with opposite
    %sings.
    %the second row is used for consistency.
    score=Sc(:,2);
    
end