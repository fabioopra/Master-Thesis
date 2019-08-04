function [ flag,AvIn,AverageIntensityCut,FramePlot,num] = DarkorLight(FramePlot,AvIn,flag,n,l,j)
%To decide if the light should be activated the average intensity of the
%upper half of the image is analyzed (with an appropriate cut). The value 
%of the intensity is filtered in time in order to be less erratic and two 
%thresold have been defined: in order to switch from dark to light (from 
%lights on to lights off), the filtered average intensity should go beyond 
%the higher threshold; to switch from lights off to lights on the it should 
%go beyond the lower threshold

%INPUTS:
%FramePlot:    B&W image
%AvIn:      current filtered average intensity of the top-half-image 
%flag:      if 1 -> light on ; if 0 -> light off; if 2->daylight
%n:         vertical resolution of the image
%l:         horizotal resolution of the image
%j:         time (number of frame)

%OUTPUTS:
%flag:                  if 1 -> light on ; if 0 -> light off
%AvIn                   filtered value of the average intenity
%AverageIntensityCut    un-filtered value of the average intensity 
%FramePlot              image with only the part considered for the average
                        %intensity computation (the remaining part is set 
                        %to 0). It also have the bounging box around the
                        %detected street-lamps 
%num                    number of stree-laps detected in this frame

%defining the values of the three thresholds
thresh_low=1.5; %lower-hysteresis threshold
thresh_high=2.5; %higher-hysteresis threshold 
thresh_superhigh=20; %threshold that detects if it is daylight

%average intensity of the zone where the streetlamps are present
Frame2=FramePlot(1:n/2,1:l/2); %left-upper part
Frame3=FramePlot(1:n/2,l/2+1:l); %right-upper part

FrameClean=FramePlot;

%in this variable it is accumulated the overall intesity of the cosidered
%region of the image
count=0;

%calculating average intensity in the appropirate region of the image
for y=1:n/2 %y is the y coordinate
    for x=1:l/2 %x is the x coordinate
       
        %selecting the appropriate region for calculating intensity
        if y<=n/l*x && y<n/2-100
            count=count+double(Frame2(y,x));
        end
        if y<=-n/l*x+n/2 && y<n/2-100
            count=count+double(Frame3(y,x));
        end
        
    end
end

%average intensity of the considered area
AverageIntensityCut=count/(n*l/4);


%erasing the non-considered zone for plotting the image (it is only useful
%to visualize the region in which the intensitty is calculated 
for y=1:n %y is the y coordinate
    for x=1:l %x is the x coordinate
        if y>n/l*x || y>-n/l*x+n || y>=n/2-100
            FramePlot(y,x)=0;
        end
    end
end


%low-pass filtering the value of the average intensity in order to reduce
%the erraticity
    if j==1
        AvIn=AverageIntensityCut;
    else
        AvIn=AvIn*0.99+AverageIntensityCut*0.01;
    end
    
    
%checking if a light switch is needed:

    %if the  intensity go above the super threshold the light should be off
    %and it is useless to calculate the nuber of lights
    if AvIn>=thresh_superhigh 
        flag=2;
        num=[];
    else
        %calculating the number of streetlamps
        [FramePlot, num ] = numlamps( FramePlot,FrameClean,n,l);
        
        %if the  intensity go under the lower threshold and the number of  
        %detected lights is 0the light should be switched on
        if AvIn<=thresh_low & num==0 
            flag=1;  
            
        %if the  intensity go over the higher threshold and the number of  
        %detected lights is greater than 0 the light should be switched off
        elseif AvIn>=thresh_high & num>0
            flag=0; 
    end

end
end
