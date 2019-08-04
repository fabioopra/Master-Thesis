clc
clear all
close all

addpath('../Icons')
addpath('../ExternalIllumination')
addpath('../Benchmark')
addpath('../Variables/Hand_Chosen')
%% PATH Video che voglio processare

% v = VideoReader('C:\Users\Fabio\Desktop\ScriptNew 11\Videos/LightTest.mov','CurrentTime',0); %video to be analyzed
% v = VideoReader('D:\Fabio\Scuola\Universit?\V_Anno\II semestre\Tesi\Benchmark\BenchMark-light/2016_1111_082700_002_short.mov','CurrentTime',26.5); %video light
% v = VideoReader('/Users/Nicolas/Documents/Polimi/Italia_anno2_semestre2/Thesis/Matlab/ScriptNew 12/Benchmark/giorno23.mov','CurrentTime',5);

% v = VideoReader('/Users/Dario/Dropbox/Adaptive lights/Benchmark/Dark.mov');
v = VideoReader('Light.mov');

% CurrentTime -> Secondo in cui parte il video

%% SYMBOLS USED FOR THE HMI INTERFACE
HIGH_BEAMS_SYMBOL=imread('HIGHBEAMS64.jpg');
LOW_BEAMS_SYMBOL=imread('LOWBEAMS64.jpg');
STREET_LIGHT_SYMBOL=imread('STREETLIGHT64.jpg');
RIGHT_TURN_ON_SYMBOL=imread('RightTurn_ON.jpg');
RIGHT_TURN_OFF_SYMBOL=imread('RightTurn_OFF.jpg');
LEFT_TURN_ON_SYMBOL=imread('LeftTurn_ON.jpg');
LEFT_TURN_OFF_SYMBOL=imread('LeftTurn_OFF.jpg');
ROAD_SYMBOL=imread('Road_symbol.jpg');
LOAD_SYMBOL=imread('Load_symbol.jpg');
FOUND_SYMBOL=imread('Found_symbol.jpg');

mkdir('VideoResult')
gray1 = VideoWriter('VideoResult/TestLineLightAdaptive1.avi'); %create a video called gray1.avi
gray1.FrameRate = v.FrameRate; %uses this info to create a new video with the analized frames

gray2 = VideoWriter('VideoResult/TestLineLightAdaptiveCUT.avi'); %create a video called gray1.avi
gray2.FrameRate = v.FrameRate; %uses this info to create a new video with the analized frames

%% INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%principal variables to load
load('cameraparam.mat') %camera parameters found with calibration
load('SVM.mat') %SVM to define if a line is continuous or Discontinuous
load('SVM_pairs.mat') %SVM to define if a couple of lines is a Lane
%% Camera and frames parameters
n=1080; % height of the image
l=1920; % width of the image

FrameOr=zeros(n,l,3); %original frame
FrameWithLines=zeros(n,l,3); %frame with all the information
Frame1=zeros(n,l); %frame for vehicle detection
Frame2=zeros(n,l); %frame for external light detection
BW=zeros(1080,1920); %frame used at several steps

x_optical=l/2; 
y_optical=n/2;
%optical point after calibration

% x_optical=cameraParams.PrincipalPoint(1);
% y_optical=cameraParams.PrincipalPoint(2);
%optical point before calibration

fx= cameraParams.FocalLength(1);
fy= cameraParams.FocalLength(2);
%focal length in px both horizontal and vertical


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LANE DETECTION PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% parameters houghROI
%parameters for the hough transform
ThetaMinLeft=1; %minimum angle for the left ROI 0 
ThetaMaxLeft=89; %maximum angle for the left ROI 90
ThetaMinRight=-89; %minimum angle for the right ROI -89
ThetaMaxRight=-1; %maximum angle for the right ROI 0

ResolutionHoughTheta=1; %resolution for the values of theta
NumPeaks=50; %number of peaks of the Hough space taken
ThresholdHough=0.005; % Threshold for the minimum value acceptable for a
                     % peak, it goes from 0 to 1, where 1 is a threshold at
                     % the same value as the maximum in the hough space
FillGap=5; %10 space between lines which is filled automatically
MinLength=20; %20 % minimum length of the lines found with the parameters of
              % of the hough peaks
%parameter for the filtering of the lines
AroundOpticalCenter=100; % PARAMETER USED FOR THE RANGE AROUND THE OPTICAL POINT AT WHICH LINES
                        %ARE ACCEPTED
              
%% Parameters SingleFrameLineGrouping

WidthLaneLines=200; %80 % PARAMETER USED TO DEFINE THE MAXIMUM DISTANCE BETWEEN THE POINTS
                   % OF TWO LINES TO CONSIDER THEM AS LINES THAT SHARE THE
                   % SAME LANE                
extra_length=5;    %Extra length added to lines for line grouping

%SVMStruct2        variable contining the structure of the SVM trained to 
                   %define if a couple of lines can be or not a lane

%% Parameters SingleFrameCoupleAnalysis

%MdlStd variable contining the structure of the SVM trained to define
%if a line is continuous or discontinuous

%% Parameters TemporalMatchingCouples

couples_memory=[];          %Variable containing the information of all the
                            %couples that have been found in the last
                            %frames and that is updated at each frame 
found_couples=[];           %contains the amount of times that a couple has
                            %been found in different frames
missing_couples=[];         %contains the amount of times that a couple has
                            %not been found in the las frames
uCouples=0.8;               %This variable is the parameter for the weighted
                            %average for the updating of the parameters of
                            %the couples
MaxMissCouples=10;          %This variable contains the maximum amount of
                            %frames a couple can be lost before it is
                            %deleted
%% parameters FoundLaneCouple
LaneWidth=1400;             %Lane Width in px at the bottom of the image

width_difference=450;       %Maximum difference between the variable
                            %LaneWidth and the lane width computed with the
                            %couple that has the highest score

uwidth=0.95;                %Factor used for the moving average that
                            %updates the lane width
foundmin=5;                 %Minimum amount of frames for a couple to be
                            %considered as the possible lane lines
%% Parameters for Lane Lines
%initial lane lines parameters, if this parameters are the current values
%for the lines, the lines are not plotted

%Left line's parameters
thetal=100;
rhol=0;
typel=1; % 0-continuous, 1-discontinuous
%Right line's parameters
thetar=-100;
rhor=0;
typer=1;              

%% Parameters for FrameCut
Ypx= fy*1.5/4.5; %height of 1.5 m at a distance of 7 m from the camera
Ypx2= fy*1.5/300; %height of 1.5 m at a distance of 100 m from the camera
%this parameters are used to cut the frame

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PARAMETERS OF EXTERNAL LIGHT DETECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameters DarkorLight
plotAvInCut_Sm=[]; %vector with all the average filtered intensity value for each time instant
AvIn_Sm=[]; %scalar with the current average filtered intensity
flagLampDL=0; %if dark -> 1; if light -> 0
plot_flagLampDL=[]; %vector with all the flagLampDL values for each time instant
width1B=[];
height1B=[];
plotI=[];
num_plot=[];

j=1; %used to know at which frame is the algorithm
open(gray1) %start to work in the new video
open(gray2)

flag_adaptive = 0;    % flag del vehicle detection, in questo script viene mantenuto == 0 (non c'e' la parte di vehicle detection)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START MAIN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


while hasFrame(v) && j<=100 %while the video v has not ended
     tic
     
     % Plot immagine originale (senza correzione distorsione camera)
     
%     framedis=readFrame(v);
%     figure
%     imshow(framedis);
%     imwrite(framedis,'Dist.png');

    [FrameOr,newOrigin] = undistortImage(readFrame(v),cameraParams);
    
     % Plot immagine con correzione distorsione camera
    
%     figure
%     imshow(FrameOr);
%     imwrite(FrameOr,'Undist.png');
     %FrameOr is the current frame to be analyzed
     
%% LINE DETECTION --------------------------------------------------------

    %% Add guide lines (aggiunge un cerchio intorno al centro ottico)
    
     FrameWithLines=FrameOr; %FrameWithLines will contain all the valuable
                             %information: Lines and vehicles
     FrameWithLines=insertShape(FrameWithLines,'Circle',[x_optical y_optical 10],'Color','White'); %adds a circle in the position of the optical center
     FrameLight=FrameOr;     %Frame used for the headlamp detection  
     
    %% RGB to Gray
    
    Frame1=rgb2gray(FrameOr);%transformation from RGB to gray
    
    % Plot RGB to GRAY
    
%     figure
%     imshow(Frame1);
%     imwrite(Frame1,'Grey.png');

    Frame2=Frame1; %Frame used for the external illumination detection
    
    %% BLUR AND EDGE DETECTION
    
    BW=zeros(n/2,l/2); %matrix for edge detection (creo lo spazio in memoria)
    
    BW(:,1:l/2) = edge(Frame1(n/2+1:n,1:l/2),'sobel',[],'both','nothinning'); %edge detection of the left - bottom quarter
    
    BW(:,l/2+1:l) = edge(Frame1(n/2+1:n,l/2+1:l),'sobel',[],'both','nothinning'); %edge detection of the right - bottom quarter
    
    %Sobel Edge detector computed both in horizontal and vertical direction
    % [] is used to define the threshold to decide which are the strong
    % edges, in this case is automatically

    % Plot immagine binaria sx e dx
    
%     figure
%     imshow(BW(:,l/2+1:l));
%     imwrite(BW(:,l/2+1:l),'EdgeRight.png');
%     
%     figure
%     imshow(BW(:,1:l/2));
%     imwrite(BW(:,1:l/2),'EdgeLeft.png');

    
      %% HOUGH TRANSFORM TO FIND ALL POSSIBLE LINES IN THE ACTUAL FRAME
      
    [ linesl, linesr ] = houghROI( BW ,n,l,ThetaMinLeft,ThetaMaxLeft,...
        ThetaMinRight,ThetaMaxRight,ResolutionHoughTheta,NumPeaks,...
        ThresholdHough,FillGap,MinLength,x_optical,y_optical,AroundOpticalCenter);
     %This function find the lines on the left and on the right of the
     %image between the defined theta range, all the parameters have been
     %explained above

    %lines structure
            %lines.rho
            %lines.theta
            %lines.point1           [x,y] point at the top of the line
            %lines.point2           [x,y] point at the bottom of the line

    %% CODE USED TO PLOT ALL THE LINES
    
% % %   this code can be used to print all the lines found with the hough
% % transform, Left lines appear green and right ones yellow
%     FrameWithLines=FrameOr;
%     for i=1:length(linesr)
%                     Endpoints = [linesr(i).point1 linesr(i).point2];
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',3);
%     end
% % %     
% %     figure
% % %   Right Part
% %     imshow(FrameWithLines(n/2+1:n,l/2+1:l,:)); %% show the image
% %     imwrite(FrameWithLines(n/2+1:n,l/2+1:l,:),'FilterRight.png'); %save
% %     the image
% %     
%     for i=1:length(linesl)
%                     Endpoints = [linesl(i).point1 linesl(i).point2];
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',3,'Color','green');
%     end
% %     figure
% % %   Left Part
% %     imshow(FrameWithLines(y_optical:n,1:l/2,:));
% %     imwrite(FrameWithLines(y_optical:n,1:l/2,:),'FilterLeft.png');

     %% LINE GROUPING (con identificazione linea continua - tratteggiata, in fondo alla funzione SingleFrameLineGrouping c'e' il codice
     %% per modifica SVM)
     
    %linesll=linesl;
    
    % Grouping sull'immagine di sinistra
    
    [ linesl, linesl_length,ratio_coveredL,labelL,scoreL,linesll] = SingleFrameLineGrouping(  linesl, WidthLaneLines,extra_length,n,l,y_optical,SVMStruct2);
   
    %This function groups the lines that are close to each other and define
    %if the line is continuous or discontinuos
    %STRUCTURE OF THE LINES
           %lines.rho
           %lines.theta
           %lines.point1            [x y] coordinates of the top point of
                                    %the line
           %lines.point2            [x y] coordinates of the bottom point
                                    %of the line   
    
    % Grouping sull'immagine di destra                              
                                    
    [ linesr, linesr_length,ratio_coveredR,labelR,scoreR,linesrr] = SingleFrameLineGrouping(  linesr, WidthLaneLines,extra_length,n,l,y_optical,SVMStruct2);


    %% Plot of the Groups of Lines (plot di tutte le linee)
    
    %In this step all the lines groups are plotted with the respective
    %classification result, red for continuous and magenta for
    %discontinuous, the variables linesrr and linesll are just used to plot
    
    %linesrr is an array in which each element is an array of the lines
    %belonging to the same group (same parameters (rho,theta))
%     FrameWithLines=FrameOr;
%     for i=1:length(linesrr)
%                 linesrrr=linesrr{i};
%                 for q=1:length(linesrrr)
%                     Endpoints = [linesrrr(q).point1 linesrrr(q).point2];
%                     if labelR(i)==0
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%                     else
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%                     end
%                 end
%     end
%     
% % %   whole picture
% %     imshow(FrameWithLines);
% % %   Right part
% %     figure
% %     imshow(FrameWithLines(n/2+1:n,l/2+1:l,:));
% %     imwrite(FrameWithLines(n/2+1:n,l/2+1:l,:),'GroupRight.png');
%     
%     for i=1:length(linesll)
%         lineslll=linesll{i};
%         for q=1:length(lineslll)
%                     Endpoints = [lineslll(q).point1 lineslll(q).point2];
%                     if labelL(i)==0
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%                     else
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%                     end
%         end
%     end
%     
% % %   Left part    
% %     figure
% %     imshow(FrameWithLines(y_optical:n,1:l/2,:));
% %     imwrite(FrameWithLines(y_optical:n,1:l/2,:),'GroupLeft.png');

    %% CODE USED TO PLOT ALL THE LANE MARKINGS FOUND (plot della sola linea pi? lunga di ogni gruppo, ANCHE SE CI SONO PIU' CORSIE)
    
% %   this code can be used to print all the lines found with the hough
% transform, Left lines appear green and right ones yellow
%     FrameWithLines=FrameOr;
%     for i=1:length(linesr)
%                     Endpoints = [linesr(i).point1 linesr(i).point2];
%                     if labelR(i)==0
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%                     else
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%                     end
%     end
% % %     
% %     figure
% % %   Right Part
% %     imshow(FrameWithLines(n/2+1:n,l/2+1:l,:)); %% show the image
% %     imwrite(FrameWithLines(n/2+1:n,l/2+1:l,:),'LaneMarkingsRight.png'); %save
% %     the image
% %     
%     for i=1:length(linesl)
%                     Endpoints = [linesl(i).point1 linesl(i).point2];
%                     if labelL(i)==0
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%                     else
%                     FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%                     end
%     end
% %     figure
% % %   Left Part
% %     imshow(FrameWithLines(y_optical:n,1:l/2,:));
% %     imwrite(FrameWithLines(y_optical:n,1:l/2,:),'LaneMarkingsLeft.png');
    %% Single frame Couple analysis (SVM features corsia, ogni coppia di linee trovata ha uno score)

    [couples]=SingleFrameCoupleAnalysis(linesl,linesr,scoreL,scoreR,MdlStd);
    
%This function takes the left and right lines to create couples to which a
%score is given, this score defines if the couple of lines is a feasible
%couple for the lane

%each row of the matrix couple contains the following information
%couples = [theta_LEFT      rho_LEFT    score_continuous_LEFT ...
           %theta_RIGHT     rho_RIGHT   score_continuous_RIGHT...
           %sum_theta       sum_rho     score_couple];
           
%% CODE USED TO PLOT ALL THE COUPLES MARKINGS FOUND IN THE CURRENT FRAME (plot di tutte le coppie di linee con relativo score corsia)

% % %   this code can be used to print all the couples found,
% % %   the score is presented on the right

% h=400;
%     [lll,~]=size(couples);
%     for i=1:lll
%         FrameWithLines=FrameOr;
%         figure
%         theta=couples(i,1);
%         rho=couples(i,2);
%         m=-cos(theta*pi/180)/sin(theta*pi/180);
%         b=rho/sin(theta*pi/180);
%         Endpoints = Extremes(n,l,m,b,n,n-h);
%         if couples(i,3)<0
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%         else
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%         end
%         
%         theta=couples(i,4);
%         rho=couples(i,5);
%         m=-cos(theta*pi/180)/sin(theta*pi/180);
%         b=rho/sin(theta*pi/180);
%         Endpoints = Extremes(n,l,m,b,n,n-h);
%         if couples(i,6)<0
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%         else
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%         end
%         
%         if couples(i,9) > 1
%             score_per=100;
%         elseif couples(i,9) > -1
%             score_per=(0.5+couples(i,9)/2)*100;
%         else
%             score_per=0;
%         end
%         
%         positionText=[1500 516];
%         FrameWithLines=insertText(FrameWithLines,positionText,sprintf('%d %%',round(score_per)),'FontSize',24,'BoxColor','black','BoxOpacity',0.1,'TextColor','white');
%         imshow(FrameWithLines)
%     end

     %% Temporal Matching of couples
    
    [couples_memory,found_couples,missing_couples] = TemporalMatchingCouples(couples_memory,found_couples,missing_couples,uCouples,MaxMissCouples,couples);
%This function takes the couples in memory and the couples in the current
%frame and updates the information of the couples in memory, if a couple in
%memory has not been matched for several frames it is deleted, and if a
%couple in the current frame is not matched with any of the couples in
%memory this one is added to the couples in memory.
     %% CODE USED TO PLOT ALL THE COUPLES MARKINGS IN MEMORY (plot delle coppie di linee in memoria)
     
% %   this code can be used to print all the couples in memory,
% %   the score is presented on the right
% h=400;
%     [lll,~]=size(couples_memory);
%     for i=1:lll
%         FrameWithLines=FrameOr;
%         figure
%         theta=couples_memory(i,1);
%         rho=couples_memory(i,2);
%         m=-cos(theta*pi/180)/sin(theta*pi/180);
%         b=rho/sin(theta*pi/180);
%         Endpoints = Extremes(n,l,m,b,n,n-h);
%         if couples_memory(i,3)<0
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%         else
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%         end
%         
%         theta=couples_memory(i,4);
%         rho=couples_memory(i,5);
%         m=-cos(theta*pi/180)/sin(theta*pi/180);
%         b=rho/sin(theta*pi/180);
%         Endpoints = Extremes(n,l,m,b,n,n-h);
%         if couples_memory(i,6)<0
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','magenta');
%         else
%         FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',5,'Color','Red');
%         end
%         
%         if couples_memory(i,9) > 1
%             score_per=100;
%         elseif couples_memory(i,9) > -1
%             score_per=(0.5+couples_memory(i,9)/2)*100;
%         else
%             score_per=0;
%         end
%         
%         positionText=[1500 516];
%         FrameWithLines=insertText(FrameWithLines,positionText,sprintf('%d %%',round(score_per)),'FontSize',24,'BoxColor','black','BoxOpacity',0.1,'TextColor','white');
%         imshow(FrameWithLines)
%     end

   %% Lane couple
    [thetal,rhol,typel,thetar,rhor,typer,score,ThetaMinLeft,ThetaMaxLeft,ThetaMinRight,ThetaMaxRight, NumPeaks, LaneWidth,flagleft,flagright,flagcouples]= FoundLaneCouple(couples_memory,found_couples,LaneWidth,n,width_difference, uwidth,foundmin);
%This function takes the couples in memory and choose the one with the
%higher score as the lane lines, in addition, the range of research of the
%lane lines is reduced around the found lines.
%     sum_theta=thetal+thetar;
%     sum_rho=rhol+rhor;
%     Data=[sum_theta, sum_rho];
%     [~,score]=predict(MdlStd,Data);
    if score > 1
        score_per=100;
    elseif score >0
        score_per=(0.5+score/2)*100;
    else
        score_per=0;
    end
    %% The lane Lines are plotted with an speciall color (plot delle linee vere della strada)
    
    h=400;                % Valore di y fino al quale si vuole plottare le linee in pixels
    if thetal < 100
            
            m=-cos(thetal*pi/180)/sin(thetal*pi/180);
            b=rhol/sin(thetal*pi/180);
            Endpoints = Extremes(n,l,m,b,n,n-h);
            if typel == 0
            FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',8,'Color','red');
            else 
            FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',8,'Color','magenta');
            end
    end
    
    h=400;
    if thetar > -100       
            m=-cos(thetar*pi/180)/sin(thetar*pi/180);
            b=rhor/sin(thetar*pi/180);
            Endpoints = Extremes(n,l,m,b,n,n-h);
            if typer == 0
            FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',8,'Color','red');
            else 
            FrameWithLines = insertShape(FrameWithLines,'Line',Endpoints,'LineWidth',8,'Color','magenta');
            end
    end
% 
%     figure
%     imshow(FrameWithLines)

    %% LANE CHANGE SYMBOLS (UNDER DEVELOPMENT)
%  if flagleft == 1
%         left=1700;
%         top=400;
%         color=[255 255 255];
%         [ FrameWithLines ] = AddSymbol( FrameWithLines, LEFT_TURN_ON_SYMBOL,left,top,color);
%  else
%         left=1700;
%         top=400;
%         color=[255 255 255];
%         [ FrameWithLines ] = AddSymbol( FrameWithLines, LEFT_TURN_OFF_SYMBOL,left,top,color);
%  end
%  if flagright == 1
%         left=1733;
%         top=400;
%         color=[255 255 255];
%         [ FrameWithLines ] = AddSymbol( FrameWithLines, RIGHT_TURN_ON_SYMBOL,left,top,color);
%   else
%         left=1733;
%         top=400;
%         color=[255 255 255];
%         [ FrameWithLines ] = AddSymbol( FrameWithLines, RIGHT_TURN_OFF_SYMBOL,left,top,color);
%   end 

%% LANE DETECTION SYMBOLS
  if flagcouples == 1
        left=1700;
        top=500;
        color=[255 255 255];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, ROAD_SYMBOL,left,top,color);
        left=1600;
        top=500;
        color=[255 255 255];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, FOUND_SYMBOL,left,top,color);
        positionText=[1500 516];
        FrameWithLines=insertText(FrameWithLines,positionText,sprintf('%d %%',round(score_per)),'FontSize',24,'BoxColor','black','BoxOpacity',0.1,'TextColor','white');
 else
        left=1700;
        top=500;
        color=[100 100 100];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, ROAD_SYMBOL,left,top,color);
        left=1600;
        top=500;
        color=[255 255 255];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, LOAD_SYMBOL,left,top,color);
        positionText=[1500 516];
        FrameWithLines=insertText(FrameWithLines,positionText,sprintf('%d %%',round(score_per)),'FontSize',24,'BoxColor','black','BoxOpacity',0.1,'TextColor','white');
 end
    %% BOX FOR CUTTING (plot del frame cutting)
    
    %uncomment this in order to see how the frame is cut
%  Ypx= fy*1.5/4.5;
%  Ypx2= fy*1.5/300;
%  
%  xleft= (rhol-n*sin(thetal*pi/180))/cos(thetal*pi/180);
%  xright=(rhor-n*sin(thetar*pi/180))/cos(thetar*pi/180);
%  LaneWidth=xright-xleft;
%  ProportionLeft=1/2+(xright-x_optical)/LaneWidth;
%  ProportionRight=1/2+(x_optical-xleft)/LaneWidth;
%  
%  
%  
%  
%  mleftdown= ((y_optical)-n)/(x_optical-(xleft-LaneWidth));
%  bleftdown= (y_optical)-mleftdown*x_optical;
%  xleftdown= round((n-bleftdown)/mleftdown);
%  
%  y_left=Ypx*ProportionLeft;
%  mleftup= ((y_optical-Ypx2)-(n-y_left))/(x_optical-(xleft-LaneWidth));
%  bleftup= (y_optical-Ypx2)-mleftup*x_optical;
%  yleftup= round(mleftup*xleftdown+bleftup);
%  
%  
%  
%  mrightdown= (y_optical-n)/(x_optical-(xright+LaneWidth));
%  brightdown= (y_optical)-mrightdown*x_optical;
%  xrightdown= round((n-brightdown)/mrightdown);
%  
%  y_right=Ypx*ProportionRight;
%  mrightup= ((y_optical-Ypx2)-(n-y_right))/(x_optical-(xright+LaneWidth));
%  brightup= (y_optical-Ypx2)-mrightup*x_optical;    
%  yrightup= round(mrightup*xrightdown+brightup);
%  
%  
%  N=n;
%  L=(-xleftdown)+xrightdown;
%  
%  FrameBox=zeros(N,L+100,3);
%  
%  for i=1:n
%      for q=1:l
%          
%          FrameBox(i,(-xleftdown)+50+q,1)=double(FrameWithLines(i,q,1));
%          FrameBox(i,(-xleftdown)+50+q,2)=double(FrameWithLines(i,q,2));
%          FrameBox(i,(-xleftdown)+50+q,3)=double(FrameWithLines(i,q,3));
%      end
%  end
%  
%  FrameBox=uint8(FrameBox);
%  Endpoints = [50,n,(-xleftdown)+50+x_optical,y_optical];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',5,'Color','white');
%  
%  Endpoints = [50,n,50,yleftup];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',10,'Color','white');
%  
%  Endpoints = [50,yleftup,(-xleftdown)+50+x_optical,y_optical-Ypx2];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',5,'Color','white');
%  
%  Endpoints = [(-xleftdown)+xrightdown+50,n,(-xleftdown)+50+x_optical,y_optical];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',5,'Color','white');
%  
%  Endpoints = [(-xleftdown)+xrightdown+50,n,(-xleftdown)+xrightdown+50,yrightup];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',10,'Color','white');
%  
%  Endpoints = [(-xleftdown)+xrightdown+50,yrightup,(-xleftdown)+50+x_optical,y_optical-Ypx2];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',5,'Color','white');
%  
%  Endpoints = [(-xleftdown)+50+x_optical,y_optical,(-xleftdown)+50+x_optical,y_optical-Ypx2];
%  FrameBox = insertShape(FrameBox,'Line',Endpoints,'LineWidth',10,'Color','white');
%  
%   imshow(FrameBox(:,(-xleftdown)+50:(-xleftdown)+50+l,:));
%    imwrite(FrameBox,'LongCut2.png');
%    imwrite(FrameBox(:,(-xleftdown)+50:(-xleftdown)+50+l,:),'ShortCut2.png');
%  imwrite(Frame1,'FrameForVehicleDetection.png');
%  imwrite(FrameWithLines,'FrameWithLines.png');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% External illumination detection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dark or Light? 
%this function identifies if there is not enought illumination in the 
%street so that it is necessary to switch on the light

    [flagLampDL,AvIn_Sm,AverageIntensityCut, Frame4, num ] = DarkorLight(Frame2,AvIn_Sm,flagLampDL,n,l,j);
    plot_flagLampDL=[plot_flagLampDL flagLampDL];
    plotAvInCut_Sm=[plotAvInCut_Sm AvIn_Sm];
    num_plot=[num_plot num];
    %plotAvIn=[plotAvIn AverageIntensityCut];
    plotI=[plotI AverageIntensityCut];
    
 %% FRAME CUT
     [ Frame1 ] = FrameCut( Frame1, thetal,rhol,typel,thetar,rhor,typer,LaneWidth,n,l,x_optical,y_optical, Ypx, Ypx2, fy);
%This function takes the grayscale frame and change to black all the
%px outside the road using the lane lines found
%imshow(Frame1) %frame after cut with lane lines

if flagLampDL==1  
%   figure
%   imshow(FrameWithLines);
%   title('traked blobs')
        left=1700;
        top=300;
        color=[100 100 100];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, STREET_LIGHT_SYMBOL,left,top,color);
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
 %% LIGHT DETECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if flag_adaptive == 0
            %tic
            % [ Frame1,FrameWithLines,Ellipse] = LightDetection(Frame1,4,FrameWithLines,j,Ellipse); % Scontornamento faro
            %time(j)=toc;
            end    
 else
        flag_adaptive=1;
        left=1700;
        top=300;
        color=[255 255 255];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, STREET_LIGHT_SYMBOL,left,top,color);
 end
%% ADAPTIVE LIGHTS
  if flag_adaptive == 1
      
        %FrameWithLines=insertShape(FrameWithLines,'Circle',[l*0.9 n*0.1 30],'Color','White');
        left=1700;
        top=100;
        color=[0 255 0];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, LOW_BEAMS_SYMBOL,left,top,color);
  
  else
        %FrameWithLines=insertShape(FrameWithLines,'FilledCircle',[l*0.9 n*0.1 30],'Color','White');
        left=1700;
        top=100;
        color=[0 0 255];
        [ FrameWithLines ] = AddSymbol( FrameWithLines, HIGH_BEAMS_SYMBOL,left,top,color);
        
  end
  
  
  time(j)=toc;
  %% SAVES THE FRAME IN THE VIDEO
    writeVideo(gray1,FrameWithLines);
    writeVideo(gray2,Frame1);
    j=j+1;
end
close(gray1)
close(gray2)