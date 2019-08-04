function [ parameters_current ] = LinesIncorrect_Auto( linesl,linesr,thetar,thetal,l,n )
delete_idx=[];
x_optical=l/2;
% 
% for i=1:length(linesl)
%     if abs(linesl(i).theta - thetal) < 10
%         delete_idx=[delete_idx, i];
%     end
% end
% linesl(delete_idx)=[];
% delete_idx=[];
% for i=1:length(linesr)
%     if abs(linesr(i).theta - thetar) < 10
%         delete_idx=[delete_idx, i];
%     end
% end
% linesr(delete_idx)=[];

rrr=length(linesr);
lll=length(linesl);

parameters_current=zeros(rrr*lll,5);


for kkk=1:rrr
    xright=(linesr(kkk).rho -n*sin(linesr(kkk).theta*pi/180))/cos(linesr(kkk).theta*pi/180);
    for iii=1:lll
        xleft=(linesl(iii).rho - n*sin(linesl(iii).theta*pi/180))/cos(linesl(iii).theta*pi/180);
        xcenter=(xleft+xright)/2;
        lwidth=xright-xleft;
        xratio=(x_optical-xcenter)/(lwidth);
        p = [linesl(iii).theta, linesl(iii).rho, linesr(kkk).theta, linesr(kkk).rho, xratio];
        parameters_current((kkk-1)*lll+iii,:)=p;
    end

end

[L,W]=size(parameters_current);
delete_idx=[];
for i=1:L
    if abs(parameters_current(i,1)-thetal) < 15 && abs(parameters_current(i,3)-thetar) < 15
        delete_idx=[delete_idx, i];
    end
end
parameters_current(delete_idx,:)=[];
end

