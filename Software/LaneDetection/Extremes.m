function [ EndPoints ] = Extremes(n,l,m,b,y1,y2)
%Find the extreme points of the line starting from the bottom
%of the image to the height definded by h, if one line exits the image
%before reaching the bottom, the exiting point is kept as (x1,y1)

%INPUT
    %n          height of the image 1080
    %l          width of the image 1920
    %m          slope of the line
    %b          intersect of the line
    %y1         Bottom y-coordinate of the line
    %y2         Top y-coordinate of the line



%OUTPUT
    %EndPoints      [x1,y1,x2,y2]
    
    
x1=0;
x2=0;

if m < 0
        if (y1-b)/m >= 1
            x1=(y1-b)/m;
            
        else
            x1=1;
            y1=m*x1+b;
        end
        
        if (y2-b)/m >= 1
            x2=(y2-b)/m;
            
        else
            x2=1;
            y2=m*x2+b;
        end
        
elseif m > 0
        if (y1-b)/m <= l 
            x1=(y1-b)/m;
            
        else
            x1=l;
            y1=m*x1+b;
        end
        
        if (y2-b)/m <= l
            x2=(y2-b)/m;
            
        else
            x2=l;
            y2=m*x2+b;
        end
end


EndPoints=[x1, y1, x2, y2];
end

