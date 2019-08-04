    % Compute the scores over a grid
%d = 0.02; % Step size of the grid
[x1Grid,x2Grid] = meshgrid(min(Data(:,1)):0.1:max(Data(:,1)),...
                           min(Data(:,2)):1:max(Data(:,2)));
xGrid = [x1Grid(:),x2Grid(:)];        % The grid
[~,scores1] = predict(MdlStd,xGrid); % The scores
%[scores1] = predict(MdlStd,xGrid); % The scores
figure;
h(1:2) = gscatter(Data(:,1),Data(:,2),class);
hold on
h(3) = plot(Data(MdlStd.IsSupportVector,1),...
            Data(MdlStd.IsSupportVector,2),'ko','MarkerSize',10);
            % Support vectors
contour(x1Grid,x2Grid,reshape(scores1(:,2),size(x1Grid)),[0 0],'k');
%contour(x1Grid,x2Grid,reshape(scores1(:),size(x1Grid)),[0 0],'k');
    % Decision boundary
title('Scatter Diagram with the Decision Boundary')
legend({'-1','1','Support Vectors'},'Location','Best');
hold off