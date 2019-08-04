function [ parameters_current ] = LinesCorrect_Auto( linesl,linesr,l,n)
x_optical=l/2;
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

end

