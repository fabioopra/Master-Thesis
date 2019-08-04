function [ BW ] = Thresholding( Frame,levels )
%This function computes the Otsu thresholdong for a frame
%
%INPUT
    %Frame      GrayScale Image to apply the thresholding
    %levels     number of thresholds to apply 
    
%OUTPUT
    %BW         Image Black and White with the brigth objects as 1 and the
                %remaining pixels as 0 

    thresh = multithresh(Frame,levels); %Find the thresholds of the image
    %thresh is an array with the values of the threshold
    
    BW = imbinarize(Frame,double(thresh(levels))/255);
    %imbinarize sets as 0 de values below the threshold and as 1 the values
    %avobe the threshold

end

