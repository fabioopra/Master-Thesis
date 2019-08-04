function [couples_memory,found_couples,missing_couples] = TemporalMatchingCouples(couples_memory,found_couples,missing_couples,uCouples,MaxMissCouples,couples)
%This function takes the couples in memory and the couples in the current
%frame and updates the information of the couples in memory, if a couple in
%memory has not been matched for several frames it is deleted, and if a
%couple in the current frame is not matched with any of the couples in
%memory this one is added to the couples in memory.

        

%INPUT
    %couples_memory         %Variable containing the information of all the
                            %couples that have been found in the last
                            %frames and that is updated at each frame 
    %found_couples          contains the amount of times that a couple has
                            %been found in different frames
    %missing_couples        contains the amount of times that a couple has
                            %not been found in the las frames
    %uCouples               This variable is the parameter for the weighted
                            %average for the updating of the parameters of
                            %the couples
    %MaxMissCouples         %This variable contains the maximum amount of
                            %frames a couple can be lost before it is
                            %deleted
    %couples                %Variable containing the information of all the
                            %couples found in the current frame
  
%OUTPUT
    %couples_memory         %Variable containing the information of all the
                            %couples that have been found in the last       
    %found_couples          contains the amount of times that a couple has
                            %been found in different frames
    %missing_couples        contains the amount of times that a couple has
                            %not been found in the las frames
    
      %STRUCTURE OF THE couples AND couples_memory

%each row of the matrix couple contains the following information
%couples = [theta_LEFT      rho_LEFT    score_continuous_LEFT ...
           %theta_RIGHT     rho_RIGHT   score_continuous_RIGHT...
           %sum_theta       sum_rho     score_couple];
                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %if the vector 'couples_memory' is empty, all the couples found in the
    %current frame are saved in memory
    if isempty(couples_memory)
        couples_memory=couples;
        [Length_memory,~]=size(couples_memory);
        found_couples=ones(Length_memory,1); %all the couples have been found once
        missing_couples=zeros(Length_memory,1); %all the couples have not been missed
    else
    %if instead the vector 'couples_memory' is not empty, then the couples
    %found in the current frame must be matched with those in memory
        i=1;
        [Length_memory,~]=size(couples_memory);
        %in this cycles the couples in memory are compared with the couples
        %found in the current frame and the parameters are updated, the
        %couples that have not been matched for more than MaxMissCouples
        %are deleted
        while i <= Length_memory
            %parameter of the i-couple in memory
            thetal_memory=couples_memory(i,1);
            rhol_memory=couples_memory(i,2);
            thetar_memory=couples_memory(i,4);
            rhor_memory=couples_memory(i,5);
            flag=0; %this flag is used to define if a couple in memory has 
            %been matched with one couple in the current frame
            j=1;
            [Length_extra,~]=size(couples);
            while j <= Length_extra
                %parameters of the j-couple in the current frame
                thetal=couples(j,1);
                rhol=couples(j,2);
                thetar=couples(j,4);
                rhor=couples(j,5);
                
                %the couples are matched if the their lines have similar
                %parameters
                if abs(thetal_memory-thetal) < 5 && abs(rhol_memory-rhol) < 150 && ...
                   abs(thetar_memory-thetar) < 5 && abs(rhor_memory-rhor) < 150 

                    %%update
                    couples_memory(i,1)=uCouples*thetal_memory+(1-uCouples)*thetal; %theta LEFT
                    couples_memory(i,2)=uCouples*rhol_memory+(1-uCouples)*rhol; %rho LEFT
                    couples_memory(i,3)=uCouples*couples_memory(i,3)+(1-uCouples)*couples(j,3); % update the score continuous vs discontinuous Left
                    couples_memory(i,4)=uCouples*thetar_memory+(1-uCouples)*thetar; %theta RIGHT
                    couples_memory(i,5)=uCouples*rhor_memory+(1-uCouples)*rhor; %rho RIGHT
                    couples_memory(i,6)=uCouples*couples_memory(i,6)+(1-uCouples)*couples(j,6); % update the score continuous vs discontinuous Right
                    couples_memory(i,7)=uCouples*couples_memory(i,7)+(1-uCouples)*couples(j,7); % update of the sum_theta
                    couples_memory(i,8)=uCouples*couples_memory(i,8)+(1-uCouples)*couples(j,8); % update of the sum_rho
                    couples_memory(i,9)=uCouples*couples_memory(i,9)+(1-uCouples)*couples(j,9); % update of the score_couple
                    found_couples(i)=found_couples(i)+1; %The found is increased by one
                    missing_couples(i)=0; %The missing is restarted
                    flag=1; %the couple has been matched
                    couples(j,:)=[]; %the couple in the current frame is deleted
                    [Length_extra,~]=size(couples); %The length is updated in order to control the inner while
                    j=Length_extra+1; %inner while is ended 
                end
                
                j=j+1;
            end
            
            if flag==0 %couple not matched
               missing_couples(i)=missing_couples(i)+1; %missing is increased by one
            end
            %disappear
            if missing_couples(i) >= MaxMissCouples % couple not matched for several frames is deleted
                couples_memory(i,:)=[];
                missing_couples(i)=[];
                found_couples(i)=[];
                i=i-1;
                [Length_memory,~]=size(couples_memory);
            end
            i=i+1;
        end
%Once the cycle is over, all the couples in memory have been analyzed and
%the matched couples in the current frame have been deleted from the vector
%'couple', then if this vector is not empty it means that new couples have
%been found and must be added to the 'couples_memory' vector

        %%appear
        if ~isempty(couples)

            [Length_memory,~]=size(couples_memory);
            [Length_extra,~]=size(couples);

            new_couples_memory=zeros(Length_memory+Length_extra,9);
            new_missing_couples=zeros(Length_memory+Length_extra,1);
            new_found_couples=ones(Length_memory+Length_extra,1);

            new_couples_memory(1:Length_memory,:)=couples_memory;
            new_couples_memory(Length_memory+1:Length_memory+Length_extra,:)=couples;
            new_missing_couples(1:Length_memory)=missing_couples;
            new_found_couples(1:Length_memory)=found_couples;

            couples_memory = new_couples_memory;
            missing_couples = new_missing_couples;
            found_couples = new_found_couples;
        end


    end
end

