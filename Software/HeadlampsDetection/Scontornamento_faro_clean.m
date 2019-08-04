clear all
close all
clc

%% Importo il frame RGB dalla cartella

% Video 1

% cd('/Users/Dario/Dropbox/Ducati/Adaptive Lights/Prove/2016_1111_083900_006');

% Video 2

cd('/Users/Dario/Dropbox/Ducati/Adaptive Lights/Prove/2016_1111_082700_002');

list = dir('*.jpg');

%% Importo i parametri di calibrazione della camera

load('/Users/Dario/Dropbox/Ducati/Adaptive Lights/Software/2017_06_25 Scontornamento faro/cameraParams_final.mat');

%% Definisco il video che andro' a salvare

v = VideoWriter('v','MPEG-4'); 
v.FrameRate = 30; 

open(v);

cycle = 0;

%% Ciclo su tutte le immagini

% 720-1200 frames

for i = 3050:1:3700
    
    clear s s_high_I max index k poly_v poly ROIcropped thresh_percent good_pixels black_pixels thresh
    
    cycle = cycle +1;
    
    FrameRGB = imread(list(i).name);       % Estraggo frame RGB
    
    [FrameRGB,newOrigin] = undistortImage(FrameRGB,cameraParams_final);    % Tolgo la distorsione della lente

    %% Image pre-processing (filtro gaussiano per rendere piu' smooth il perimetro identificato)
    
    % Passo in scala di grigi
    
    FrameGray = rgb2gray(FrameRGB);          % Per trovare le regioni troppo luminose
        
    %% Applico Otsu Thresholding
    
    n = 4;
    
    thresh = multithresh(FrameGray,n);    % Soglia Otsu adattativa a n livelli
    
    FrameQuantized = imquantize(FrameGray, thresh);        % Immagine quantizzata
    FrameQuantizedRGB = label2rgb(FrameQuantized);         % Immagine quantizzata in RGB per visualizzazione

    %% Scelgo la threshold che mi scontorna il faro e converto in immagine binaria
    
    BW = imbinarize(FrameGray,double(thresh(1))/255);                       % Immagine binaria threshold pi? bassa (individuazione faro)
   
    n_thresh_max = 3;

    BW_high_I = imbinarize(FrameGray,double(thresh(n_thresh_max))/255);     % Immagine binaria threshold alta luminosit?
        
    %% Controllo la regione intorno alle zone ad alta intensita'
    
    % Trovo le bounding box degli oggetti piu' luminosi
    
    BW1_high_I = bwconncomp(BW_high_I);              % Trovo le componenti connesse
    
    s_high_I = regionprops('table',BW1_high_I,'centroid','Area','BoundingBox');    % Estraggo le features che mi interessano
    
    index = find(([s_high_I.Area] >= 1));            % Tengo solo le componenti connesse abbastanza grandi

    BW2_high_I = ismember(labelmatrix(BW1_high_I), index);

    % Definizione nuove buonding box ingrandite di un certo fattore di scala per eliminare il rumore
    
    scale_factor = 1.1;
    
    ScaledBoundingBox = [s_high_I.BoundingBox];
    
    ScaledBoundingBox(:,1:2) = ScaledBoundingBox(:,1:2) - ScaledBoundingBox(:,3:4)*0.5*(scale_factor - 1); % metto a posto il left corner

    ScaledBoundingBox(:,3:4) = ScaledBoundingBox(:,3:4)*scale_factor; % scalo altezza e larghezza
  
    % Controllo quanti sono in percentuale i pixel sotto la soglia nelle bounding box
    
    for j = 1:length(index)

        ROICropped{j} = imcrop(FrameGray,ScaledBoundingBox(index(j),:));   % Prendo solo l'interno del bounding box
        
        for k = 1:(length(thresh)+1)
            
            if k == 1
                
                thresh_percent{j}(k) = length(find(ROICropped{j} < thresh(k)))/numel(ROICropped{j});   % Percentuali pixel per ogni threshold
                
            elseif k == length(thresh)+1
                
                thresh_percent{j}(k) = length(find(ROICropped{j} >= thresh(k-1)))/numel(ROICropped{j});

            elseif k ~= 1 & k ~= length(thresh)+1
                
                thresh_percent{j}(k) = length(find((ROICropped{j} < thresh(k)) & (ROICropped{j} >= thresh(k-1))))/numel(ROICropped{j});

            end;
                        
        end;
        
        good_pixels(j) = sum(thresh_percent{j}(2:n_thresh_max));            % Sommo percentuali pixel 'buoni'
        black_pixels(j) = thresh_percent{j}(1);                             % Percentuale pixel 'neri'

    end;
    

    
    % Elimino dalla lista gli oggetti luminosi nella cui bounding box sono contenuti troppi pixel 'buoni' e pochi 'neri'
    
    index(find(good_pixels > 0.4 & black_pixels < 0.3)) = [];

    % Plot degli oggetti da eliminare
    
    for j = 1:length(index)
        
        FrameQuantizedRGB=insertObjectAnnotation(FrameQuantizedRGB,'rectangle',ScaledBoundingBox(index(j),:),sprintf('%d',j),...
                                                 'LineWidth',4,'Color','yellow');
    end
    
    
    %% Elimino gli oggetti identificati come troppo luminosi
    
    ROI_clear = FrameGray;      % Immagine con solo le regioni di interesse
    
    % Definizione bounding box per l'eliminazione dei cartelli
    
    scale_factor_2 = 1.5;       % Elimino un boundingbox che ? 1.5 per il boundingbox originale
    
    DelBoundingBox = [s_high_I.BoundingBox];
    
    DelBoundingBox(:,1:2) = DelBoundingBox(:,1:2) - DelBoundingBox(:,3:4)*0.5*(scale_factor_2 - 1); % metto a posto il left corner

    DelBoundingBox(:,3:4) = DelBoundingBox(:,3:4)*scale_factor_2; % scalo altezza e larghezza
    
    % Converto i bounding box degli oggetti da eliminare in poligoni in coordinate cartesiane, creo le maschere e le elimino dall'immagine 
    
    for j = 1:length(index)
        
        x_base = DelBoundingBox(index(j),1);
        y_base = DelBoundingBox(index(j),2);

        BB_poly_x =[x_base, x_base+DelBoundingBox(index(j),3), x_base+DelBoundingBox(index(j),3), x_base, x_base];
        BB_poly_y =[y_base, y_base, y_base+DelBoundingBox(index(j),4), y_base+DelBoundingBox(index(j),4), y_base];
        
        mask = poly2mask(BB_poly_x,BB_poly_y,length(FrameGray(:,1)),length(FrameGray(1,:)));
        
        mask3 = cat(3, mask, mask, mask);
        
        ROI_clear(mask==1) = 0;
        FrameQuantizedRGB(mask3) = 0;
        
    end
    
    %% Scelgo l'area con la componente connessa maggiore
    
    % Filtraggio Gaussiano
    
    FrameBlurClear = imgaussfilt(ROI_clear, 3);
    
    BW_clear = imbinarize(FrameBlurClear,double(thresh(1))/255);

    BW1 =  bwconncomp(BW_clear);      % connected components (restituisce una struct, non un'immagine)
    
    % Trovo le features di interesse
    
    s = regionprops('table',BW1,'centroid','Area','BoundingBox','Eccentricity','MajorAxisLength','MinorAxisLength',...
                    'Orientation');  % 'ConvexHull'
    
    % Scelgo solo la regione con l'area maggiore
    
    A = s.Area;
    
    clear index
    
    [max,index] = max(A);
    
    BW2 = ismember(labelmatrix(BW1), index);
    
    % trovo il perimetro
    
    BW3= imfill(BW2,'holes');
    BWperim = bwperim(BW3);
        
    RGBperim = double(cat(3, BWperim, BWperim, BWperim));

    %% Trovo il centro dell'area illuminata e il poligono di contorno
    
    center = round(int32([s.Centroid(index,:)]));
    
    %% Trovo il centroide calcolato in base alla luminosita' dei pixel
    
    % Prendo come region of interest solo l'area scontornata
    
    ROI = FrameBlurClear;

    ROI(BW3==0) = 0;

    sum_mx = 0;
    sum_my = 0;
    
    for r = 1:length(ROI(:,1))
        for c = 1:length(ROI(1,:))
            
            sum_mx = sum_mx + double(ROI(r,c))*c;
            sum_my = sum_my + double(ROI(r,c))*r;
            
        end
    end
            
    x_cI(cycle) = round(int32(sum_mx/sum(sum(ROI))));   
    y_cI(cycle) = round(int32(sum_my/sum(sum(ROI))));
    
    % Ricavo la luminosita' totale dell'area individuata
    
    Total_intensity(cycle) = round(sum(sum(ROI)));

    %% Plot del perimetro della luce e del centro individuato sull'immagine originale
    
    [B,L] = bwboundaries(BW3);

    FinalImage = imshowpair(FrameRGB,RGBperim,'blend');
    
    yellow = uint8([255 255 0]);
    green  = uint8([0   255 0]);
    red    = uint8([255   0 0]);
    
    shapeInserter1 = vision.ShapeInserter('Shape','Circles','BorderColor','Custom','CustomBorderColor',red);
%     shapeInserter2 = vision.ShapeInserter('Shape','Polygons','BorderColor','Custom','CustomBorderColor',green);
    shapeInserter3 = vision.ShapeInserter('Shape','Circles','BorderColor','Custom','CustomBorderColor',yellow);

    % Ricavo tutti i parametri dell'ellisse frame dopo frame
    
    a(cycle) = double([s.MajorAxisLength(index,:)]/2);
    b(cycle) = double([s.MinorAxisLength(index,:)]/2);
    Xc(cycle) = double(center(1));
    Yc(cycle) = double(center(2));
    phi(cycle) = double(deg2rad(-s.Orientation(index,:)));
    
    % Filtro con media mobile su 10 frame
    
    if cycle<10
        
        a_filt = mean(a);
        b_filt = mean(b);
        Xc_filt = mean(Xc);
        Yc_filt = mean(Yc);
        x_cI_filt(cycle) = mean(x_cI);
        y_cI_filt(cycle) = mean(y_cI);
        phi_filt = mean(phi);

    else 
        
        a_filt = mean(a(cycle-9:cycle));
        b_filt = mean(b(cycle-9:cycle));
        Xc_filt = mean(Xc(cycle-9:cycle));
        Yc_filt = mean(Yc(cycle-9:cycle));
        x_cI_filt(cycle) = mean(x_cI(cycle-9:cycle));
        y_cI_filt(cycle) = mean(y_cI(cycle-9:cycle));
        phi_filt = mean(phi(cycle-9:cycle));
        
    end
    
    center_filt = round(int32([Xc_filt Yc_filt]));
    weighted_center_filt = round(int32([x_cI_filt(cycle) y_cI_filt(cycle)]));
    
    area_ellipse = pi*a_filt*b_filt;
    %area = sum(sum(BW3));

    % Calcolo il rapporto tra area totale dell'ellisse e intensita' luminosa totale
    
    ratio_I_area(cycle) = Total_intensity(cycle)/area_ellipse;
    
    % Plot dell'ellisse
    
    FinalImage = step(shapeInserter3,FinalImage.CData,[weighted_center_filt 10]);    % Plot centro ellisse
    %FinalImage = step(shapeInserter1,FinalImage,[center_filt 10]);
        
    figure(21);
    imshow(FinalImage);
    hold on
        
    t = linspace(0,2*pi,50);

    x = Xc_filt + a_filt*cos(t)*cos(phi_filt) - b_filt*sin(t)*sin(phi_filt);         % Plot ellisse utilizzando equazioni
    y = Yc_filt + a_filt*cos(t)*sin(phi_filt) + b_filt*sin(t)*cos(phi_filt);
    plot(x,y,'r','Linewidth',2)
    hold off
    
    FinalImage = getframe(gcf);                                                      % Ricavo il frame dalla figura
    
    close(gcf);

    % Inserisco le posizioni dei due centri, l'intensita' totale e il rapporto area/intensita' all'interno dell'immagine
    
    %text_ellipse = ['Xe: ' num2str(Xc_filt,'%0.2f') ' Ye: ' num2str(Yc_filt,'%0.2f')];
    text_cI = ['Xc: ' num2str(x_cI_filt(cycle),'%0.2f') ' Yc: ' num2str(y_cI_filt(cycle),'%0.2f')];
    text_total_intensity = ['I_tot: ' num2str(Total_intensity(cycle),'%0.2f') ]
    text_ratio_I_area = ['r: ' num2str(ratio_I_area(cycle),'%0.2f') ]

    text_position = [1700 42;1700 86;1700 130];
    box_color = {'yellow','red'};
    
    FinalImage = insertText(FinalImage.cdata,text_position(1,:),text_cI,'FontSize',24,'BoxColor',...
    box_color{1},'BoxOpacity',0.4,'TextColor','white');
    
    FinalImage = insertText(FinalImage,text_position(2,:),text_total_intensity,'FontSize',24,'BoxColor',...
    box_color{2},'BoxOpacity',0.4,'TextColor','white');

    FinalImage = insertText(FinalImage,text_position(3,:),text_ratio_I_area,'FontSize',24,'BoxColor',...
    box_color{1},'BoxOpacity',0.4,'TextColor','white');

%     imshow(FinalImage);
       
    %% Creo il video

    writeVideo(v,FinalImage);

end;

close(v);
