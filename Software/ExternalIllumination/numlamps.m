function [FramePlot, num ] = numlamps( FramePlot,FrameClean,n,l);
%This function counts the number of theoretical streetlights in the scene

%INPUT:
%FramePlot:    B&W image that will be used to plot what the fuction does
%FrameClean:   B&W image
%n:            height of the image (1080px)
%l:            width of the image (1920px)
%OUTPUT
%FramePlot:    B&W image that will be used to plot what the fuction does
%num           number of high level intesity lights in the upper part of 
               %the image
     
%creating a new smaller image that is the upper half of the B&W frame
    FrameClean=FrameClean(1:n/2,1:l);     

%otsu thresh. finds the brightest objects
    thresh = multithresh(FrameClean,4); %Find the thresholds of the image
    %thresh is an array with the values of the threshold
    
%imbinarize sets as 0 de values below the threshold and as 1 the values
%avobe the threshold 
    FrameClean = imbinarize(FrameClean,double(thresh(4))/255);

%finding the connected components
    BW1 =  bwconncomp(FrameClean);
  
%creating a table with all the connected components with some feature
    lamps = regionprops('table',BW1,'centroid','Area','BoundingBox');

 num=height(lamps);
 
    del=[];
%if the centroid of blob is under the selected region or, the area is small 
%it is deleted. So only the region where the streetlamps are present 
%is considered.
    for i=1:num
        if lamps.Centroid(i,2)>n/l*lamps.Centroid(i,1) || lamps.Centroid(i,2)>-n/l*lamps.Centroid(i,1)+n ...
           || lamps.Centroid(i,2)>n/2-100 || lamps.Area(i)<100
                        del=[del i];
        end
    end
    
    lamps(del,:)=[];
    
%this is used to plot a boundingbox around the selected blobs (streetlamps)
    for i=1:height(lamps)
        position=lamps.BoundingBox(i,:);
        FramePlot=insertObjectAnnotation(FramePlot,'rectangle',position,sprintf('%d',lamps.Area(i)));
    end
    

%the final number of theoretical streetlights
    num=height(lamps);
 
end
