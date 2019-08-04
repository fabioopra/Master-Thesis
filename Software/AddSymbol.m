function [ FrameWithLines ] = AddSymbol( FrameWithLines, Symbol,left,top,color)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

[height,width]=size(Symbol);

for i=1:height
    for j=1:width
        
            if Symbol(i,j) > 100
                FrameWithLines(top+i,left+j,1)=color(1);
                FrameWithLines(top+i,left+j,2)=color(2);
                FrameWithLines(top+i,left+j,3)=color(3);
            end
        
    end
end



end

