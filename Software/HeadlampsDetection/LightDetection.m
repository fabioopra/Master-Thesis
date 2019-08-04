function [ FrameGray,FrameOr,Ellipse ] = LightDetection( FrameGray,n,FrameOr,cycle,Ellipse )
%INPUTS:
%FrameGray      Grey-sclae image with cut scene using lines
%n              number of thresholds for Otsu Thresholding
%FrameOr        RGB image with detected lines and vehicles shown
%cycle          number of the current frame
%Ellipse        Structure with all the ellipse parameters (full structure
                %at the end of the function)

%OUTPUTS:
%FrameGray      Grey-sclae image with cut scene using lines
%FrameOr        frame with the ellipse
%Ellipse        updated ellipse struct
    
    thresh = multithresh(FrameGray,n);    % Soglia Otsu adattativa a n livelli
    
    %% Scelgo la threshold che mi scontorna il faro e converto in immagine binaria
    %BW = imbinarize(FrameGray,double(thresh(1))/255);                       % Immagine binaria threshold pi? bassa (individuazione faro)
   
    n_thresh_max = 3;

%     BW_high_I = imbinarize(FrameGray,double(thresh(n_thresh_max))/255);     % Immagine binaria threshold alta luminosit?
%     
% 
%     %% Controllo la regione intorno alle zone ad alta intensita'
%     
%     % Trovo le bounding box degli oggetti piu' luminosi
%     
%     BW1_high_I = bwconncomp(BW_high_I);              % Trovo le componenti connesse
%     
%     s_high_I = regionprops('table',BW1_high_I,'BoundingBox');    % Estraggo le features che mi interessano
% %     s_high_I = regionprops('table',BW1_high_I,'centroid','Area','BoundingBox','MajorAxisLength','MinorAxisLength',...
% %                     'Orientation');
%      % Definizione nuove buonding box ingrandite di un certo fattore di scala per eliminare il rumore
%     
%     scale_factor = 1.1;
%     
%     ScaledBoundingBox = [s_high_I.BoundingBox];
%     
%     ScaledBoundingBox(:,1:2) = ScaledBoundingBox(:,1:2) - ScaledBoundingBox(:,3:4)*0.5*(scale_factor - 1); % metto a posto il left corner
% 
%     ScaledBoundingBox(:,3:4) = ScaledBoundingBox(:,3:4)*scale_factor; % scalo altezza e larghezza
%     
%      % Controllo quanti sono in percentuale i pixel sotto la soglia nelle bounding box
%      
%     for j = 1:height(s_high_I)
% 
%         ROICropped{j} = imcrop(FrameGray,ScaledBoundingBox(j,:));   % Prendo solo l'interno del bounding box
%         
%         for k = 1:(length(thresh)+1)
%             
%             if k == 1
%                 
%                 thresh_percent{j}(k) = length(find(ROICropped{j} < thresh(k)))/numel(ROICropped{j});   % Percentuali pixel per ogni threshold
%                 
%             elseif k == length(thresh)+1
%                 
%                 thresh_percent{j}(k) = length(find(ROICropped{j} >= thresh(k-1)))/numel(ROICropped{j});
% 
%             elseif k ~= 1 & k ~= length(thresh)+1
%                 
%                 thresh_percent{j}(k) = length(find((ROICropped{j} < thresh(k)) & (ROICropped{j} >= thresh(k-1))))/numel(ROICropped{j});
% 
%             end
%                         
%         end
%         
%         good_pixels(j) = sum(thresh_percent{j}(2:n_thresh_max));  %da aggiungere n_thresh_min          % Sommo percentuali pixel 'buoni'
%         black_pixels(j) = thresh_percent{j}(1);                             % Percentuale pixel 'neri'
% 
%     end
%     
% %     thresh_percent
%     
%     % Elimino dalla lista gli oggetti luminosi nella cui bounding box sono contenuti troppi pixel 'buoni' e pochi 'neri'
%     index=1:height(s_high_I); %aggiunto da noi
%     
%     index(find(good_pixels > 0.4 & black_pixels < 0.3)) = [];
%     
%     % Plot degli oggetti da eliminare   
% %     for j = 1:length(index)
% %         
% %         FrameOr=insertObjectAnnotation(FrameOr,'rectangle',ScaledBoundingBox(index(j),:),sprintf('%d',j),...
% %                                                  'LineWidth',4,'Color','yellow');
% %     end
% 
% 
%     %% Elimino gli oggetti identificati come troppo luminosi
%     
%     %ROI_clear = FrameGray;      % Immagine con solo le regioni di interesse
%     
%     % Definizione bounding box per l'eliminazione dei cartelli
%     
%     scale_factor_2 = 1.5;       % Elimino un boundingbox che ? 1.5 per il boundingbox originale
%     
%     DelBoundingBox = [s_high_I.BoundingBox];
%     
%     DelBoundingBox(:,1:2) = DelBoundingBox(:,1:2) - DelBoundingBox(:,3:4)*0.5*(scale_factor_2 - 1); % metto a posto il left corner
% 
%     DelBoundingBox(:,3:4) = DelBoundingBox(:,3:4)*scale_factor_2; % scalo altezza e larghezza
% 
%  % Converto i bounding box degli oggetti da eliminare in poligoni in coordinate cartesiane, creo le maschere e le elimino dall'immagine 
%     
%     for j = 1:length(index)
%         ymin=round(max(min(DelBoundingBox(index(j),2),1080),1));
%         ymax=round(max(min(DelBoundingBox(index(j),2)+DelBoundingBox(index(j),4),1080),1));
%         xmin=round(max(min(DelBoundingBox(index(j),1),1920),1));
%         xmax=round(max(min(DelBoundingBox(index(j),1)+DelBoundingBox(index(j),3),1920),1));
%         FrameGray(ymin:ymax,xmin:xmax)=0;
%     end
%     s_high_I(index,:)=[];
%% Scelgo l'area con la componente connessa maggiore
    
    % Filtraggio Gaussiano
    
    %FrameBlurClear = imgaussfilt(FrameGray, 3);
     %BW_clear = imbinarize(FrameBlurClear,double(thresh(1))/255);
   
% BINARIZE the image selecting all the levels greater than the first (only
% the lower intensity pixels are set to 0, the others are set to 1)
    BW_clear = imbinarize(FrameGray,double(thresh(1))/255);

  % Connected components analysis (restituisce una struct, non un'immagine)
    BW1 =  bwconncomp(BW_clear);  
    
  % Interesting features
    s = regionprops('table',BW1,'centroid','Area','BoundingBox','MajorAxisLength','MinorAxisLength',...
                   'Orientation');  % 'ConvexHull'
    
               
    % Scelgo solo la regione con l'area maggiore
    % Selecting the connected component with the greatest area
    A = s.Area;
    clear index
    [maxA,index] = max(A);
    % the index of the bigger connected component is saved in variable
    % "index"
    
    
    %SELECTING ONLY THE BIGGEST CONNECTED COMPONENT:
    %with "labelmatrix" all the connected components are labeled in the
    %image with the number of the relative connected component (the first
    %conn.comp. will have 1 for all its pixels, the second all 2 ...)  
    % With "ismember" it is possbible to obtain a binary image in which
    % there is 1 for the conn.comp. labeled with the value "index", 0 for
    % all the others (this is the structure of BW2)
    BW2 = ismember(labelmatrix(BW1), index);
    %BW2 = ismember(labelmatrix(BW1_high_I), index);
    
    %filling the biggest connected component
    BW3= imfill(BW2,'holes');
%     BWperim = bwperim(BW3);
% 
%     RGBperim = double(cat(3, BWperim, BWperim, BWperim));

    %% Trovo il centro dell'area illuminata e il poligono di contorno
    
    center = round(int32([s.Centroid(index,:)]));
    
    %% Trovo il centroide calcolato in base alla luminosita' dei pixel

    %ROI = FrameBlurClear;
    
    %Prendo come region of interest solo l'area scontornata
    %Selecting only the region of the biggest connected component in the
    %greysclae image (in order to exploit luminance pixels values)
    ROI = FrameGray;
    ROI(BW3==0) = 0;
    %ROI is a greyscale image with all 0 except for the biggest conn.comp.
    %pixels that have the luminace value

    sum_mx = 0;
    sum_my = 0;
    
    
    %in the following it is created an augmented bounding-box in order to
    %consider a bigger patch of image that contains the connected component
    scale_factor = 1.1; %increasing factor
    ScaledBoundingBox = [s.BoundingBox(index,:)];
    % metto a posto il left corner
    %relocating the left-top corner
    ScaledBoundingBox(1:2) = ScaledBoundingBox(1:2) - ScaledBoundingBox(3:4)*0.5*(scale_factor - 1); 
    %rescaling width and height
    ScaledBoundingBox(3:4) = ScaledBoundingBox(3:4)*scale_factor;
    
    %obtaining the limits of the augmented bounding-box
    ymin=round(max(min(ScaledBoundingBox(2),1080),1));
    ymax=round(max(min(ScaledBoundingBox(2)+ScaledBoundingBox(4),1080),1));
    xmin=round(max(min(ScaledBoundingBox(1),1920),1));
    xmax=round(max(min(ScaledBoundingBox(1)+ScaledBoundingBox(3),1920),1));

    
    sum_all=0;
    for r = ymin:ymax
        for c = xmin:xmax
            
            sum_mx = sum_mx + double(ROI(r,c))*c;
            sum_my = sum_my + double(ROI(r,c))*r;
            sum_all= sum_all + double(ROI(r,c));
            
        end
    end
    

    % Ricavo tutti i parametri dell'ellisse frame dopo frame
    % Filtro con media mobile su 10 frame
    %If the algorithm is processing the first 10 frames all the features
    %values are simply stored in order (position 1-first value, position 
    %2-second value ...).
    %If it is processing successive frames the new values are stored in the
    %10-th position while the first is cleared 

    if cycle<=10
        Ellipse(cycle).a= double([s.MajorAxisLength(index,:)]/2);
        Ellipse(cycle).b = double([s.MinorAxisLength(index,:)]/2);
        Ellipse(cycle).Xc = double(center(1));
        Ellipse(cycle).Yc = double(center(2));
        Ellipse(cycle).phi = double(deg2rad(-s.Orientation(index,:)));
        %Ellipse(cycle).x_cI = round(int32(sum_mx/sum(sum(ROI)))); 
        Ellipse(cycle).x_cI = round(int32(sum_mx/sum_all));
        %Ellipse(cycle).y_cI = round(int32(sum_my/sum(sum(ROI))));
        Ellipse(cycle).y_cI = round(int32(sum_my/sum_all));
        
        % Ricavo la luminosita' totale dell'area individuata
        %Ellipse(cycle).Total_intensity = round(sum(sum(ROI)));
        Ellipse(cycle).Total_intensity = round(sum_all);
        
        %filtering values of major and minor axis in the last 10 frames
        a_filt = mean([Ellipse.a]);
        b_filt = mean([Ellipse.b]);
        Ellipse(cycle).Area = pi*a_filt*b_filt;
        
        % Calcolo il rapporto tra area totale dell'ellisse e intensita' luminosa totale
        % Ratio TotalIntensity/TotalArea obtaining an average intensity
        Ellipse(cycle).ratio_I_area = Ellipse(cycle).Total_intensity/Ellipse(cycle).Area;
    else
        Ellipse(1) =[];
        Ellipse(10).a = double([s.MajorAxisLength(index,:)]/2);
        Ellipse(10).b = double([s.MinorAxisLength(index,:)]/2);
        Ellipse(10).Xc = double(center(1));
        Ellipse(10).Yc = double(center(2));
        Ellipse(10).phi = double(deg2rad(-s.Orientation(index,:)));
        %Ellipse(10).x_cI = round(int32(sum_mx/sum(sum(ROI)))); 
        Ellipse(10).x_cI = round(int32(sum_mx/sum_all));
        %Ellipse(10).y_cI = round(int32(sum_my/sum(sum(ROI))));
        Ellipse(10).y_cI = round(int32(sum_my/sum_all));
        
    % Ricavo la luminosita' totale dell'area individuata
        %Ellipse(10).Total_intensity = round(sum(sum(ROI)));
        Ellipse(10).Total_intensity = round(sum_all);
   
        a_filt = mean([Ellipse.a]);
        b_filt = mean([Ellipse.b]);
        Ellipse(10).Area = pi*a_filt*b_filt;
        Ellipse(10).ratio_I_area = Ellipse(10).Total_intensity/Ellipse(10).Area;
    end

    %AVERAGING the last data for PLOTTING the ELLIPSE
        Xc_filt = mean([Ellipse.Xc]);
        Yc_filt = mean([Ellipse.Yc]);
        phi_filt = mean([Ellipse.phi]);
        % usare come paramtro
        x_cI_filt = mean([Ellipse.x_cI]);
        y_cI_filt = mean([Ellipse.y_cI]);
      
    center_filt = round(int32([Xc_filt Yc_filt]));
    weighted_center_filt = round(int32([x_cI_filt y_cI_filt]));
   
    
 % Plot ellisse utilizzando equazioni
 % Plot ellipse using equations
    for t=0:2*pi/500:2*pi
        x = Xc_filt + a_filt*cos(t)*cos(phi_filt) - b_filt*sin(t)*sin(phi_filt);         
        y = Yc_filt + a_filt*cos(t)*sin(phi_filt) + b_filt*sin(t)*cos(phi_filt);
        x=round(max(min(x,1920),1));
        ymin=round(max(min(y-1,1080),1));
        ymax=round(max(min(y+1,1080),1));
        
        FrameOr(ymin:ymax,x,1)=uint8(255);
        FrameOr(ymin:ymax,x,2)=uint8(255);
        FrameOr(ymin:ymax,x,3)=uint8(0);
    end
    
    %plotting in the frame with lines and vehicles the weighted center of
    %the ellipse
    FrameOr=insertShape(FrameOr,'Circle',[weighted_center_filt 10],'Color','White');
    %imshow(FrameOr)
        end

%STRUCTURE of "Ellipse":
%a: major axis length
%b: minor axis length
%Xc: x-coordinate of the centre of mass of the ellipse taking into account 
     %only the shape (not the intensity)
%Yc: y-coordinate of the centre of mass of the ellipse taking into account 
     %only the shape (not the intensity)
%phi: angle between the x-axis and the major axis of the ellipse that
      %"better approximate" the connected component
%x_cI:x-coordinate of the centre of mass of the ellipse taking into account 
     %also the intensity
%y_cI:y-coordinate of the centre of mass of the ellipse taking into account 
     %also the intensity
%Total_intensity: sum of all the intensity values of al the connected
                 %component pixels
%Area: total area of the connected componed 
%ratio_I_area: ratio between the total intensity and the total area to
               %obtain an average intensity 