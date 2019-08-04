function [thetal,rhol,typel,thetar,rhor,typer,score,ThetaMinLeft,ThetaMaxLeft,ThetaMinRight,ThetaMaxRight, NumPeaks, LaneWidth,flagleft,flagright, flagcouples]= FoundLaneCouple(couples_memory,found_couples,LaneWidth,n,width_difference, uwidth,foundmin)
%This function takes the couples in memory and choose the one with the
%higher score as the lane lines, in addition, the range of research of the
%lane lines is reduced around the found lines.
        

%INPUT
    %couples_memory         %Variable containing the information of all the
                            %couples that have been found in the last
                            %frames and that is updated at each frame 
    %found_couples          contains the amount of times that a couple has
                            %been found in different frames
    %LaneWidth              %Lane Width in px at the bottom of the image
    %n                      %vertical resolution of the image
    %width_difference       %Maximum difference between the variable
                            %LaneWidth and the lane width computed with the
                            %couple that has the highest score
    %uwidth                 %Factor used for the moving average that
                            %updates the lane width
    %foundmin               %Minimum amount of frames for a couple to be
                            %considered as the possible lane lines


%OUTPUT
    %thetal                 theta parameter of the Lane's left line
    %rhol                   rho parameter of the Lane's left line
    %Typel                  1 - discontinuous, 0 - continuous, parameter of the
                            %Lane's left line
    %thetar                 theta parameter of the Lane's right line
    %rhor                   rho parameter of the Lane's right line
    %typer                  1 - discontinuous, 0 - continuous, parameter of the
                            %Lane's right line
    %score                  accumulated score of the couple
    %ThetaMinLeft           minimum angle for the left ROI 0 
    %ThetaMaxLeft           maximum angle for the left ROI 90
    %ThetaMinRight          minimum angle for the right ROI -89
    %ThetaMaxRight          maximum angle for the right ROI 0
    %NumPeaks               number of peaks of the Hough space taken
    %LaneWidth              Lane Width in px at the bottom of the image (updated)
    %flagleft               flag indicating that the motorcycle is about to
                            %change lane to the left (in development)
    %flagright              flag indicating that the motorcycle is about to
                            %change lane to the right (in development)
    %flagcouples            flag indicating that a correc couple of lines
                            %have been found
    
      %STRUCTURE OF THE couples_memory

%each row of the matrix couple contains the following information
%couples_memory = [theta_LEFT      rho_LEFT    score_continuous_LEFT ...
                   %theta_RIGHT     rho_RIGHT   score_continuous_RIGHT...
                   %sum_theta       sum_rho     score_couple];
                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Initialization of the parameters of the lane lines in the current moment

%parameters are initialized out of the research space such that if a
%correct couple is found, this variables are updated

    thetal_current=100;
    rhol_current=0;
    typel_current=1;
    
    thetar_current=-100;
    rhor_current=0;
    typer_current=1;
    
    score=-1;
% initialization of the flags
    flagleft=0;
    flagright=0;
    flagcouples=0;
% initialization of the score, since the couple with the highest score is
% chosen, the value is initialized at -INF
    score_current=-Inf;
    IDX=0;
    [Length_couples,~]=size(couples_memory); %amount of couples in memory
%In this cycle the couple with the highest score is found, also the couple
%has to had been found for more than 'foundmin' frames
    for i=1:Length_couples
        if found_couples(i) >= foundmin && couples_memory(i,9) >= score_current
            
            score_current=couples_memory(i,9);
            IDX=i;
        
        end

    end

%If a correct couple has been found the parameters of the current moment
%are updated
    if IDX > 0 %RIGHT COUPLE FOUND
        thetal_current=couples_memory(IDX,1);
        rhol_current=couples_memory(IDX,2);
        if couples_memory(IDX,3) > 0
            typel_current=0; %continuous
        else 
            typel_current=1; %discontinuous
        end

        thetar_current=couples_memory(IDX,4);
        rhor_current=couples_memory(IDX,5);
        if couples_memory(IDX,6) > 0
            typer_current=0; %continuous
        else 
            typer_current=1; %discontinuous
        end
        
        %score=couples_memory(IDX,9);
        %variables used to compute the current Lanewidth
        xleft=(rhol_current-n*sin(thetal_current*pi/180))/(cos(thetal_current*pi/180));
        xright=(rhor_current-n*sin(thetar_current*pi/180))/(cos(thetar_current*pi/180));
        LaneWidth_current= xright-xleft;
        %computation of the Lane Width for the current frame
        if abs(LaneWidth_current-LaneWidth) <= width_difference
        %If the difference between the current LaneWidth and the one
        %computed after several frames is less than 'width_difference' px
        %the found lines are right and the variables are updated

            LaneWidth=uwidth*LaneWidth+(1-uwidth)*LaneWidth_current;
            %update of the Lane Width
            thetar=thetar_current;
            rhor=rhor_current;
            typer=typer_current;
            
            %update of the parameters of the right line
%             
            ThetaMinRight=max(-89,thetar-15); %minimum angle for the right ROI -89
            ThetaMaxRight=min(-1,thetar+15); %maximum angle for the right ROI 0
            %since a correct couple has been found the range of research on
            %the right is reduced

            thetal=thetal_current;
            rhol=rhol_current;
            typel=typel_current;
            %update of the parameters of the left line
            
            score=score_current;
            
            ThetaMinLeft=max(1,thetal-15); %minimum angle for the left ROI 0 
            ThetaMaxLeft=min(89,thetal+15); %maximum angle for the left ROI 90
            %since a correct couple has been found the range of research on
            %the right is reduced
            NumPeaks=5;
            %reduced number of peaks
            %if the lanewidth grows a lot it means that the detection is
            %wrong therefore the lane width is restarted
            if LaneWidth > 2500
                LaneWidth=1500;
            end
            flagcouples=1;% a correct couple has been found
        else
             %if any couple is correct, the range of research is widened as
             %much as possible
             ThetaMinRight=-89; %minimum angle for the right ROI -89
             ThetaMaxRight=-1; %maximum angle for the right ROI 0             
             ThetaMinLeft=1; %minimum angle for the left ROI 0 
             ThetaMaxLeft=89; %maximum angle for the left ROI 90
             NumPeaks=50;
            %parameters out of the range to indicate that it has not been
            %found
            thetar=-100;
            rhor=0;
            typer=1;
            thetal=100;
            rhol=0;
            typel=1;
            score=-1;
        end

    else
            %if there are no couples the range of research is widened as
            %much as possible
             ThetaMinRight=-89; %minimum angle for the right ROI -89
             ThetaMaxRight=-1; %maximum angle for the right ROI 0             
             ThetaMinLeft=1; %minimum angle for the left ROI 0 
             ThetaMaxLeft=89; %maximum angle for the left ROI 90
             NumPeaks=50;
             
            thetar=-100;
            rhor=0;
            typer=1;
            thetal=100;
            rhol=0;
            typel=1;
            score=-1;
    end

%%Lane change analysis (under development)
    if thetal <= 20
        %display('Left')
        ThetaMinRight=-89; %minimum angle for the right ROI -89
        ThetaMaxRight=-1; %maximum angle for the right ROI 0
        ThetaMinLeft=1; %minimum angle for the left ROI 0 
        ThetaMaxLeft=89; %maximum angle for the left ROI 90
        NumPeaks=50;
        flagleft=1;
    end

    if thetar >= -20
        %display('Right')
        ThetaMinRight=-89; %minimum angle for the right ROI -89
        ThetaMaxRight=-1; %maximum angle for the right ROI 0
        ThetaMinLeft=1; %minimum angle for the left ROI 0 
        ThetaMaxLeft=89; %maximum angle for the left ROI 90
        NumPeaks=50;
        flagright=1;
    end

    
end