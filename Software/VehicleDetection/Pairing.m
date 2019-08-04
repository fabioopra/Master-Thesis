function [Features, idx2 ] = Pairing( Features, BW,Pv_thresh,ratioH_thresh,...
                                      ratioW_thresh,ratioA_thresh,ratioWH_thresh,maxHorDist,minHorDist)
%This functions pairs the bright objects found in the previous steps if two
%lights are similar

%INPUTS
    %Features       Table with the features of each bright object in the
                    %frame   
    %BW             Binary image with the ROI in white color
    %Pv_thresh      Threshold for vertical overlapping [0,1]
    %ratioH_thresh  Threshold for ratio of the Heights of the two blobs
                    %[0,1]
    %ratioW_thresh  Threshold for ratio of the Widths of the two blobs
                    %[0,1]
    %ratioA_thresh  Threshold for ratio of the Areas of the two blobs
                    %[0,1]
    %ratioWH_thresh Threshold for ratio of the ratios W/H of the two blobs
                    %[0,1]
    %maxHorDist     Maximum Amount of times of the max width (of the two 
                    %blobs) that defines the horizontal distance of 
                    %research [0,inf)
    %minHorDist     Minimum Amount of times of the max width (of the two 
                    %blobs) that defines the horizontal distance of 
                    %research [0,maxHorDist)
%OUTPUTS      
    %Features       Updates Features with Group and color info for the
                    %paired vehicles, for the others this is just 0
    %idx2           Matrix with two columns with the two index of the
                    %paired blobs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Section A %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

idx2=[];
%initialize the idx2 variable as an empty array 

i=1;
while i <= height(Features) %this loop runs throught all the bright elements in
                    %the "Features" table with index contained in idx
        j=i+1;
        while j <= height(Features) %this loop runs from the i+1 element to the
                               %last element in the "Features" table
            
            
            Dh = max(Features.left(i),Features.left(j)) - min(Features.right(i),Features.right(j));
            %horizhontal distance between bounding box
            %negative if there is overlap
        
            Dv = max(Features.top(i),Features.top(j))-min(Features.bottom(i),Features.bottom(j));
            %vertical distance between bounding box
            %negative if there is overlap

            Pv = -Dv/(min(Features.height(i),Features.height(j))); 
            %vertical overlapping measure
            %positive if there is an actual overlap

        
            ratio_h = min(Features.height(i),Features.height(j))/max(Features.height(i),Features.height(j));
            % ratio between height of boxes
        
            ratio_A=min(Features.Area(i),Features.Area(j))/max(Features.Area(i),Features.Area(j));
            %ratio between Area of bright elements
        
            ratio_w=min(Features.width(i),Features.width(j))/max(Features.width(i),Features.width(j));
            %ratio between width of boxes
            
            ratio_WHi=Features.width(i)/Features.height(i);
            ratio_WHj=Features.width(j)/Features.height(j);
            ratio_ratio_WH=min(ratio_WHi,ratio_WHj)/max(ratio_WHi,ratio_WHj);
            %ratio between ratiios of width/height of boxes
            
           if ((Pv >= Pv_thresh) && (Dh<maxHorDist*max(Features.width(i),Features.width(j))) ...
                   && (ratio_h>ratioH_thresh) && (ratio_A>ratioA_thresh) && (ratio_w>ratioW_thresh)) &&...
                   ratio_ratio_WH>=ratioWH_thresh && Dh>minHorDist*max(Features.width(i),Features.width(j))

                %Conditions used to pre-pair the objects:
                % *Pv: vertical projection Overlapping > 0.5
                % *Dh: horizontal distance < maximum width of the two 
                                              %bright objects * 8
                % *ratio_h: height ratio > 0.3
                % *ratio_A: Area ratio > 0.3
                % *ratio_w: width ratio > 0.3
                % *ratio_ratioWH: ratio of the width/heighy ratios > 0.5
                % *Dh: horizontal distance >0                
                idx2=[idx2;i,j];
                %idx2 saves the indexes of the paired blobs in a
                %matrix where in the rows of the matrix there
                %are the pair of indexes of the paired blobs
                  
           end
            
        j=j+1;   
        end
        
i=i+1;        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% Section B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%REFINEMENTS:
%0)deleting the pairs that are irreasonably big (doesn't work well)
%1)if a single blob appears in more than one pair the highest matching is
%  searhed and all the other "uncorrect" pairs are cleared
%2)if there are multiple pairs in a single car, one above and one below,
%  we keep just the one above (that should correspond to main rear lights)


%This funtion calculates a distance of the pair from the optical point and
%ereases all the pairs that are too big for that distance.
%0)NOT USED
% i=1;
% del=[];
%         while i<=size(idx2,1)%row of elements to be compared
%             Vwidth=Features.right(idx2(i,2))-Features.left(idx2(i,1));
%             xdist=(max(Features.right(idx2(i,1)),Features.right(idx2(i,2)))+...
%                  min(Features.left(idx2(i,1)),Features.left(idx2(i,2))))/2;
%             ydist=(max(Features.bottom(idx2(i,1)),Features.bottom(idx2(i,2)))+...
%                  min(Features.top(idx2(i,1)),Features.top(idx2(i,2))))/2;
%             dist=sqrt((xdist-960)^2+(ydist-540)^2);
%             dist_tot=sqrt(120^2+800^2);
%             if Vwidth>=dist/dist_tot*700
%                 del=[del i];
%             end
%             i=i+1;
%         end
%         idx2(del,:)=[];



i=1;
del=[];
        while i<=size(idx2,1)%row of elements to be compared
            j=i+1;
            while j<=size(idx2,1)%others rows of elements
                
                %calculating features for 2)
                
                %vertical distance between each pairs (negative if they
                %overlap)
                Dvert=min(Features.top(idx2(j,1)),Features.top(idx2(j,2)))-max(Features.bottom(idx2(i,1)),Features.bottom(idx2(i,2)));
                
                %horizontal distance between each pairs (negative if they
                %overlap)
                Dov = max(Features.left(idx2(i,1)),Features.left(idx2(j,1)))-min(Features.right(idx2(i,2)),Features.right(idx2(j,2)));
                
                %height of the i-th pair
                hi=max(Features.bottom(idx2(i,1)),Features.bottom(idx2(i,2))) - min(Features.top(idx2(i,1)),Features.top(idx2(i,2)));
                
                %height of the j-th pair
                hj=max(Features.bottom(idx2(j,1)),Features.bottom(idx2(j,2))) - min(Features.top(idx2(j,1)),Features.top(idx2(j,2)));
                
                %width of the i-th pair
                wi=max(Features.right(idx2(i,1)),Features.right(idx2(i,2)))-min(Features.left(idx2(i,1)),Features.left(idx2(i,2)));
                
                %width of the j-th pair
                wj=max(Features.right(idx2(j,1)),Features.right(idx2(j,2)))-min(Features.left(idx2(j,1)),Features.left(idx2(j,2)));
                
                %overlapping score: percentage of how much two pairs
                %overlap horizontally (positive if they overlap)
                Pov = -Dov/(max(wi,wj));

                %1)
                %if an element of the i-th row is equal to an element of
                %the j-th row (this means that one blob is paired in two 
                %different pairs) then....
                if idx2(i,1)==idx2(j,1)||idx2(i,1)==idx2(j,2)||idx2(i,2)==idx2(j,1)||idx2(i,2)==idx2(j,2)
                    wPv=0.5; 
                    wr=0.5;
                    %it is only used the horizontal overlapping and the
                    %ratio of the ratios of bb since they are the most
                    %reliable features
                   
                    %vertical distance between each blob costituting pair i
                    %(negative if they overlap)
                    Dv1 = max(Features.top(idx2(i,1)),Features.top(idx2(i,2)))-min(Features.bottom(idx2(i,1)),Features.bottom(idx2(i,2)));
                    
                    %vertical distance between each blob costituting pair j
                    %(negative if they overlap)
                    Dv2 = max(Features.top(idx2(j,1)),Features.top(idx2(j,2)))-min(Features.bottom(idx2(j,1)),Features.bottom(idx2(j,2)));
                    
                    %vertical overlap between each blob costituting pair i
                    %(posiitive)
                    Pv1 = -Dv1/(max(Features.height(idx2(i,1)),Features.height(idx2(i,2))));
                    
                    %vertical overlap between each blob costituting pair j
                    %(posiitive)
                    Pv2 = -Dv2/(max(Features.height(idx2(j,1)),Features.height(idx2(j,2))));
                    
                    %ratio width/height of the first blob of the i-th pair
                    ratio_WHi1=Features.width(idx2(i,1))/Features.height(idx2(i,1));
                    
                    %ratio width/height of the second blob of the i-th pair
                    ratio_WHi2=Features.width(idx2(i,2))/Features.height(idx2(i,2));
                    
                    %ratio of the width/height-ratios of the i-th pair
                    ratio_ratio_WHi=min(ratio_WHi1,ratio_WHi2)/max(ratio_WHi1,ratio_WHi2);
                    
                    %ratio width/height of the first blob of the j-th pair
                    ratio_WHj1=Features.width(idx2(j,1))/Features.height(idx2(j,1));
                    
                    %ratio width/height of the second blob of the j-th pair
                    ratio_WHj2=Features.width(idx2(j,2))/Features.height(idx2(j,2));
                    
                    %ratio of the width/height-ratios of the j-th pair
                    ratio_ratio_WHj=min(ratio_WHj1,ratio_WHj2)/max(ratio_WHj1,ratio_WHj2);
                    
                    %calculating the scores of  pair one and pair two so
                    %that it can be selected the pair that is more similar
                    %(the higher the score the similar two blobs are)
                    S1=wPv*Pv1+wr*ratio_ratio_WHi;
                    S2=wPv*Pv2+wr*ratio_ratio_WHj;
                    
                    %1)deleting double pairing and keeping just the pairs
                    %with an higher matching
                    if S1>S2
                       del=[del,j];
                    elseif S1<S2 
                       del=[del,i];
                       break  
                    end

                end %end if big
                %end 1)
                
%                 %2)
%                 if there are multiple pairs in a single car we keep
%                 just the higher one 
                if Pov>=0.7 && Dvert<=2*hi && Dvert>=0
                    del=[del,j];
                end 

                j=j+1; 
            end %end inner for 
            i=i+1;
        end %end outer for 
        idx2(del,:)=[];
        
end