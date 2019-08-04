function [ Frame1 ] = FrameCut( Frame1, thetal,rhol,typel,thetar,rhor,typer,LaneWidth,n,l,x_optical,y_optical, Ypx, Ypx2, fy)
%This function takes the grayscale frame and change to black all the
%px outside the road using the lane lines found

%INPUT
    %Frame1         grayscale frame
    %thetal         theta parameter of the left lane line, if the line has
                    %not been found it is 100
    %rhol           rho parameter of the left lane line, if the line has
                    %not been found it is 0
    %thetar         theta parameter of the right lane line, if the line has
                    %not been found it is -100
    %rhor           rho parameter of the right lane line, if the line has
                    %not been found it is 0
    %LaneWidth      Lane width in px, this value is updated each time both
                    %lines have been found
    %n              height of the frame
    %l              width of the frame
    %x_optical      x-coordinate of the principal point
    %y_optical      y_coordinate of the principal point
    %Ypx            height of the vehicle lights (1.5 m) at a distance of
                    %(7 m) in front of it, in px
    %Ypx2           height of the vehicle lights (1.5 m) at a distance of
                    %(100 m) in front of it, in px
    %fy             focal length vertical

%OUTPUT
    %Frame1         cut frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Variables used to add a line on the left and on the right
extra_left=1;
extra_right=1;

flag=0;
%THIS FLAG IS USED TO TELL IF AT LEAST ONE LANE LINE HAS BEEN FOUND


    if thetal < 100
        %if the left line was found
        flag =1;
        xleft= (rhol-n*sin(thetal*pi/180))/cos(thetal*pi/180);
        %THIS IS THE x-coordinate OF THE LEFT LANE LINE AT THE BOTTOM

        if thetar > -100
            %if the right line was found
            extra_right=0; 
            extra_left=0;
            %extra is returned to zero since all the information is
            %contained in the parameters of the line
            xright=(rhor-n*sin(thetar*pi/180))/cos(thetar*pi/180);
            %THIS IS THE x-coordinate OF THE RIGHT LANE LINE AT THE BOTTOM
            %LaneWidth_current=xright-xleft;
            %COMPUTES THE WIDTH OF THE LANE AS THE DIFFERENCE BETWEEN THE
            %TWO BOTTOM POINTS IN PX
            %LaneWidth=uwidth*LaneWidth+(1-uwidth)*LaneWidth_current;
            %SAVES IN MEMORY THE WIDTH FOUND IN ORDER TO KEEP TRACK OF THE
            %WIDTH OF THE LANE FOR SEVERAL FRAMES
            
            ProportionLeft=1/2+(xright-x_optical)/LaneWidth;
            ProportionRight=1/2+(x_optical-xleft)/LaneWidth;
            %COMPUTES THE FACTOR USED TO INCREASE/DECREASE THE HEIGHT OF
            %THE CUT SCENE on the right and on the left
            
            %this factor is proportional to the position of the motorbike
            %with respecto to the lane lines, the closer it is to a line,
            %the higher is the proportion at that side and the lower at the
            %opposite
            
            y_left=Ypx*ProportionLeft;
            %HEIGHT AT LEFT BOTTOM OF THE AUXILIAR LEFT LANE
            
            mleft= ((y_optical-Ypx2)-(n-y_left))/(x_optical-(xleft-LaneWidth)); %always a lane is added at the left
            bleft= (y_optical-Ypx2)-mleft*x_optical;
            %parameters of the lane's left line one lane on the left of the
            %current lane, but at the height of the average height of the
            %vehicles instead that on the floor
            
            y_right=Ypx*ProportionRight;
            %HEIGHT AT RIGHT BOTTOM OF THE AUXILIAR RIGHT LANE
            mright= ((y_optical-Ypx2)-(n-y_right))/(x_optical-(xright+(typer+extra_right)*LaneWidth));
            %A line is added on the right only if the line is discontinuous
            %it is 'typer=1' otherwise no lane is added
            bright= (y_optical-Ypx2)-mright*x_optical;
            %parameters of the right line, in this case depending on typer
            %it will be the current lane's right line or one lane on the
            %right lane's right line, again at the height of the average
            %vehicle instead that on the floor

        else %LEFT found BUT NOT RIGHT
             extra_right=1; % a line on the right is added by default
             extra_left=0;
             
            Ypx= fy*1.5/4.5; %average height of a vehicle at two lanes of
                             %distance
            Ypx2= fy*1.5/300;%average height of a vehicle close to the
                             %optical center
            %IN THIS CASE IT HAS NOT BEEN FOUND THE RIGHT LINE OF THE LANE
            %THEN, THE LEFT LINE IS USED AS A REFERENCE  to obtain the
            %right lane usig the parameter 'widthlane'

            mleft= ((y_optical-Ypx2)-(n-Ypx))/(x_optical-(xleft-(1+extra_left)*LaneWidth));
            bleft= (y_optical-Ypx2)-mleft*x_optical;
            %THE LINE AT LEFT OF THE LEFT LANE LINE IS ESTIMATED AS BEFORE
            
            mright= ((y_optical-Ypx2)-(n-Ypx))/(x_optical-(xleft+(typer+extra_right)*LaneWidth));
            bright= (y_optical-Ypx2)-mright*x_optical;
            %THE LINE AT THE RIGHT OF THE RIGHT LANE LINE IS ESTIMATED AS
            %THE x-coordinate OF THE LEFT LANE LINE PLUS TWO TIMES THE
            %LANEWIDTH. Again the line is computed at the height of the
            %average vehicle instead that on the floor
             
             
             
             
        end
        
    elseif thetar > -100 %RIGHT found BUT NOT LEFT
        flag =1;
        extra_right=0;
        extra_left=1; %one line on the left is added
        
        Ypx= fy*1.5/4.5;
        Ypx2= fy*1.5/100;
        xright=(rhor-n*sin(thetar*pi/180))/cos(thetar*pi/180);
        %same as before but for the right line
        mright= ((y_optical-Ypx2)-(n-Ypx))/(x_optical-(xright+(typer+extra_right)*LaneWidth));
        bright= (y_optical-Ypx2)-mright*x_optical;

        mleft= ((y_optical-Ypx2)-(n-Ypx))/(x_optical-(xright-(typel+extra_left)*LaneWidth));
        bleft= (y_optical-Ypx2)-mleft*x_optical;


    end
    
    %IF NO LINES ARE FOUND FLAG IS 0 AND THE FRAME IS NOT CUT
    
    %% Frame Cut
    if flag == 1
        for q=1:l
                

                for i=1:n
                    
                    if i < mleft*q+bleft || i < mright*q+bright
                        %all the pixels above the two lines computed are
                        %turn into black pixels
                        Frame1(i,q)=0;
                    end
                    
                end
                
        end            
    end
    
    
    
end


