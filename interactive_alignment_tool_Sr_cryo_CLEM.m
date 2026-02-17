function interactive_alignment_tool(rgb_img_path, em_img_path, em_tomo_img_path)
% Interactive alignment tool for RGB (reflection + fluorescence), EM, EM tomo, 
% and zoomed-in SMLM images
% It allows flip, rotate, shift, scale with live overlay visualization
% Includes intensity range sliders and toggle buttons for showing/hiding each image

% --------------------------
% Load images


% SMLM zoom out 
rgb_img = imread('/dataPath/wholeFOV_SMLM.tif');    
% low mag. EM 
em_img = imread('/dataPath/lowMag_EM.tif');      
% TOMO EM 
em_tomo_img=imread('/dataPath/highMag_Tomo.tif');
%SMLM zoom in 
FL_image=imread('/dataPath/zoomedIn_SMLM.tif');

% Convert EM images to double for processing
em_img_double = double(em_img);
em_tomo_double = double(em_tomo_img);

% Get min/max for EM images for normalization
em_min = min(em_img_double(:));
em_max = max(em_img_double(:));

rgb_double_raw = double(rgb_img);
rgb_min = prctile(rgb_double_raw(:), 1);   % 1st percentile
rgb_max = prctile(rgb_double_raw(:), 99);  % 99th percentile

em_tomo_min = min(em_tomo_double(:));
em_tomo_max = max(em_tomo_double(:));

[emH, emW] = size(em_img);

% --------------------------
% Initial transformation parameters

% --- Initialize FL image parameters ---


angle = 0;
shift_x = 0;
shift_y = 0;
scale_factor = 1.0;
do_flip = false;

% EM tomo transform params (start with approximate scale ~0.1 for example)
em_tomo_angle = 0;
em_tomo_shift_x = 0;
em_tomo_shift_y = 0;
em_tomo_scale = 0.1; % because tomo is ~10x smaller

% Intensity windows normalized 0-1
em_low = 0;
em_high = 1;

em_tomo_low = 0;
em_tomo_high = 1;

rgb_low = 0;
rgb_high = 1;

% --------------------------
% Create figure and GUI controls

%hFig = figure('Name', 'Interactive RGB–EM Alignment with EM Tomo', 'NumberTitle', 'off', ...
            %  'Position', [100, 100, 1350, 1200]); % enlarged for sliders
% --------------------------
% Create figure (screen-aware)
screenSize = get(0, 'ScreenSize');  % [x y width height]
figWidth  = min(1350, screenSize(3) - 50);
figHeight = min(1200, screenSize(4) - 100);
hFig = figure('Name','Interactive RGB–EM Alignment with EM Tomo','NumberTitle','off','Position',[50 50 figWidth figHeight]);

% Axes for overlay
hAx = axes('Parent', hFig, 'Position', [0.35, 0.1, 0.6, 0.85]);

% Initial Y position and control size
xCtrl = 30; 
%yStart = 1150;
yStart = figHeight - 50;   % start near top of figure
%gap = 30;
gap = 20; 
wCtrl = 260; 
%hCtrl = 20;
hCtrl = 18;
i = 0;  % Counter to manage vertical placement

% --- RGB Intensity Low ---
uicontrol(hFig, 'Style', 'text', 'String', 'RGB Intensity Low', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sRGBLow = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', rgb_low, ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- RGB Intensity High ---
uicontrol(hFig, 'Style', 'text', 'String', 'RGB Intensity High', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sRGBHigh = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', rgb_high, ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- RGB Rotation ---
uicontrol(hFig, 'Style', 'text', 'String', 'RGB Rotation (deg)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sRot = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -180, 'Max', 180, 'Value', angle, ...
          'SliderStep', [0.5/360 5/360], ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;


% --- RGB Shift X ---
uicontrol(hFig, 'Style', 'text', 'String', 'RGB Shift X (px)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;

sX = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -1000, 'Max', 1000, 'Value', shift_x, ...
          'SliderStep', [1/2000 10/2000], ...   % small step = 1px, big step = 10px
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- RGB Shift Y ---
uicontrol(hFig, 'Style', 'text', 'String', 'RGB Shift Y (px)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;

sY = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -1000, 'Max', 1000, 'Value', shift_y, ...
          'SliderStep', [1/2000 10/2000], ...   % same fine step
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;


% --- RGB Scale ---
uicontrol(hFig, 'Style', 'text', 'String', 'RGB Scale (zoom)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
    sScale = uicontrol(hFig, 'Style', 'slider', ...
    'Min', 0.2, 'Max', 10.0, 'Value', scale_factor, ...
    'SliderStep', [0.00067 0.0067], ...   % 5× finer zoom
    'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
    'Callback', @(~,~) update_display());i = i + 1; 

% --- Flip toggle ---
btnFlipH = uicontrol(hFig, 'Style', 'togglebutton', ...
    'String', 'Flip RGB H', ...
    'Position', [xCtrl, yStart-i*gap, 120, 30], ...
    'Callback', @(~,~) update_display());

btnFlipV = uicontrol(hFig, 'Style', 'togglebutton', ...
    'String', 'Flip RGB V', ...
    'Position', [xCtrl+130, yStart-i*gap, 120, 30], ...
    'Callback', @(~,~) update_display());

i = i + 1;


% --- EM Intensity Low ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Intensity Low', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sEMLow = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', em_low, ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Intensity High ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Intensity High', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sEMHigh = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', em_high, ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Tomo Intensity Low ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Tomo Intensity Low', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sEMTomoLow = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', em_tomo_low, ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Tomo Intensity High ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Tomo Intensity High', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sEMTomoHigh = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', em_tomo_high, ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Tomo Rotation ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Tomo Rotation (deg)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sETRot = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -180, 'Max', 180, 'Value', em_tomo_angle, ...
          'SliderStep', [0.1/360 1/360], ... % finer control
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Tomo Shift X ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Tomo Shift X (px)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sETX = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -2000, 'Max', 2000, 'Value', em_tomo_shift_x, ...
          'SliderStep', [1/4000 10/4000], ... % ~1 px per arrow, 10 px per trough
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Tomo Shift Y ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Tomo Shift Y (px)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sETY = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -2000, 'Max', 2000, 'Value', em_tomo_shift_y, ...
          'SliderStep', [1/4000 10/4000], ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- EM Tomo Scale ---
uicontrol(hFig, 'Style', 'text', 'String', 'EM Tomo Scale (zoom)', ...
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl]); i=i+1;
sETScale = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0.1, 'Max', 5, 'Value', em_tomo_scale, ...
          'SliderStep', [0.01/4.9 0.1/4.9], ... % finer scale changes
          'Position', [xCtrl, yStart-i*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display()); i=i+1;


% --------------------------
% --- FL IMAGE CONTROLS (Right Column)
% --------------------------

xCtrl_FL = xCtrl + 320;   % second column of sliders
i_FL = 0;

% --- Show FL toggle ---
uicontrol(hFig, 'Style', 'checkbox', 'String', 'Show FL Image', ...
          'Value', true, ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, 25], ...
          'Callback', @(~,~) update_display());
i_FL = i_FL + 1;

% --- FL Intensity Low ---
uicontrol(hFig, 'Style', 'text', 'String', 'FL Intensity Low', ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl]);
i_FL = i_FL + 1;
sFLLow = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 1, 'Value', 0, ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display());
i_FL = i_FL + 1;

% --- FL Intensity High ---
uicontrol(hFig, 'Style', 'text', 'String', 'FL Intensity High', ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl]);
i_FL = i_FL + 1;
sFLHigh = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0, 'Max', 2, 'Value', 2, ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display());
i_FL = i_FL + 1;

% --- FL Rotation ---
uicontrol(hFig, 'Style', 'text', 'String', 'FL Rotation (deg)', ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl]);
i_FL = i_FL + 1;
sFLRot = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -180, 'Max', 180, 'Value', 0, ...
          'SliderStep', [0.1/360 1/360], ... % fine control
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display());
i_FL = i_FL + 1;

% --- FL Shift X ---
uicontrol(hFig, 'Style', 'text', 'String', 'FL Shift X (px)', ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl]);
i_FL = i_FL + 1;
sFLX = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -2000, 'Max', 2000, 'Value', 0, ...
          'SliderStep', [1/4000 10/4000], ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display());
i_FL = i_FL + 1;

% --- FL Shift Y ---
uicontrol(hFig, 'Style', 'text', 'String', 'FL Shift Y (px)', ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl]);
i_FL = i_FL + 1;
sFLY = uicontrol(hFig, 'Style', 'slider', ...
          'Min', -2000, 'Max', 2000, 'Value', 0, ...
          'SliderStep', [1/4000 10/4000], ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display());

i_FL = i_FL + 1;

% --- FL Scale ---
uicontrol(hFig, 'Style', 'text', 'String', 'FL Scale (zoom)', ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl]);
i_FL = i_FL + 1;
sFLScale = uicontrol(hFig, 'Style', 'slider', ...
          'Min', 0.1, 'Max', 10, 'Value', 1, ...
          'SliderStep', [0.01/9.9 0.1/9.9], ...
          'Position', [xCtrl_FL, yStart - i_FL*gap, wCtrl, hCtrl], ...
          'Callback', @(~,~) update_display());
i_FL = i_FL + 1;

% --- Flip toggle (for FL image) ---
btnFLFlipH = uicontrol(hFig, 'Style', 'togglebutton', ...
    'String', 'Flip FL H', ...
    'Position', [xCtrl_FL, yStart - i_FL*gap, 120, 30], ...
    'Callback', @(~,~) update_display());

btnFLFlipV = uicontrol(hFig, 'Style', 'togglebutton', ...
    'String', 'Flip FL V', ...
    'Position', [xCtrl_FL+130, yStart - i_FL*gap, 120, 30], ...
    'Callback', @(~,~) update_display());

i_FL = i_FL + 1;


% --- Toggles (placed at bottom to avoid breaking layout) ---
i = i + 1;
uicontrol(hFig, 'Style', 'checkbox', 'String', 'Show RGB Image', ...
          'Value', true, 'Position', [xCtrl, yStart-i*gap, wCtrl, 25], ...
          'Callback', @(~,~) update_display()); i=i+1;
uicontrol(hFig, 'Style', 'checkbox', 'String', 'Show EM Image', ...
          'Value', true, 'Position', [xCtrl, yStart-i*gap, wCtrl, 25], ...
          'Callback', @(~,~) update_display()); i=i+1;
uicontrol(hFig, 'Style', 'checkbox', 'String', 'Show EM Tomo Image', ...
          'Value', true, 'Position', [xCtrl, yStart-i*gap, wCtrl, 25], ...
          'Callback', @(~,~) update_display()); i=i+1;

% --- Export button ---
i = i + 1;
uicontrol(hFig, 'Style', 'pushbutton', 'String', 'Export Aligned Images', ...
          'Position', [xCtrl, yStart-i*gap, 220, 35], ...
          'Callback', @export_images);

% Only once when creating GUI
%hImg = imshow(zeros(emH, emW, 3), 'Parent', hAx);
%axis(hAx, 'manual');  % prevent auto-rescaling

hImg = imshow(zeros(emH, emW, 3), 'Parent', hAx, 'InitialMagnification', 100, 'Border', 'tight');
axis(hAx, 'image');  % preserve aspect ratio

% --------------------------
% Initialize display
update_display();

% --------------------------
% --- Nested Functions ---


function export_images(~,~)
    % Export current aligned images exactly as seen in GUI

    [file,path] = uiputfile({'*.tif','TIFF Image (*.tif)'}, 'Save aligned images as');
    if isequal(file,0)
        disp('Export cancelled');
        return;
    end

    %% --- Read UI values ---
    angle = sRot.Value;
    shift_x = sX.Value;
    shift_y = sY.Value;
    scale_factor = sScale.Value;
    do_flip_h = btnFlipH.Value;
    do_flip_v = btnFlipV.Value;

    em_tomo_angle = sETRot.Value;
    em_tomo_shift_x = sETX.Value;
    em_tomo_shift_y = sETY.Value;
    em_tomo_scale = sETScale.Value;

    em_low = sEMLow.Value;
    em_high = sEMHigh.Value;
    em_tomo_low = sEMTomoLow.Value;
    em_tomo_high = sEMTomoHigh.Value;
    rgb_low = sRGBLow.Value;
    rgb_high = sRGBHigh.Value;

    fl_low = sFLLow.Value;
    fl_high = sFLHigh.Value;
    fl_angle = sFLRot.Value;
    fl_shift_x = sFLX.Value;
    fl_shift_y = sFLY.Value;
    fl_scale = sFLScale.Value;
    fl_flip_h = btnFLFlipH.Value;
    fl_flip_v = btnFLFlipV.Value;

    %% --- Transform RGB ---
    rgb_out = rgb_img;
    if do_flip_h, rgb_out = fliplr(rgb_out); end
    if do_flip_v, rgb_out = flipud(rgb_out); end
    rgb_out = imrotate(rgb_out, angle, 'bilinear', 'crop');
    rgb_out = imresize(rgb_out, scale_factor);

    % Apply shifts and pad/crop to EM size
    rgb_canvas = zeros(emH, emW, size(rgb_out,3));
    [hR, wR, ~] = size(rgb_out);
    x_pos = round(shift_x*3); % same as GUI shift scale
    y_pos = round(shift_y*3);
    x_start = max(1,1+x_pos); y_start = max(1,1+y_pos);
    x_img_start = max(1,1-x_pos); y_img_start = max(1,1-y_pos);
    x_end = min(emW, x_start+wR-x_img_start); y_end = min(emH, y_start+hR-y_img_start);
    width = x_end-x_start+1; height = y_end-y_start+1;
    if width>0 && height>0
        rgb_canvas(y_start:y_end, x_start:x_end, :) = ...
            rgb_out(y_img_start:y_img_start+height-1, x_img_start:x_img_start+width-1, :);
    end
    rgb_out = rgb_canvas;

    % Normalize RGB to [0,1] using sliders
    rgb_double = (double(rgb_out)-rgb_min)/(rgb_max-rgb_min);
    rgb_double = max(min((rgb_double - rgb_low)/(rgb_high - rgb_low),1),0);

    %% --- Transform EM ---
    em_norm_out = (em_img_double - em_min)/(em_max - em_min);
    em_norm_out = max(min((em_norm_out - em_low)/(em_high - em_low),1),0);

    %% --- Transform EM Tomo ---
    if em_tomo_max==em_tomo_min
        em_tomo_norm = zeros(size(em_tomo_double));
    else
        em_tomo_norm = (em_tomo_double-em_tomo_min)/(em_tomo_max-em_tomo_min);
    end
    em_tomo_norm = max(min((em_tomo_norm - em_tomo_low)/(em_tomo_high - em_tomo_low),1),0);

    em_tomo_scaled = imresize(em_tomo_norm, em_tomo_scale, 'bicubic', 'Antialiasing', true);

    % --- Rotate with 'bilinear' and 'loose' (keeps full rotated image) ---
    em_tomo_rotated = imrotate(em_tomo_scaled, em_tomo_angle, 'bilinear', 'loose');

    % Place EM Tomo on canvas with shift
    canvas = zeros(emH, emW);
    [hRT, wRT] = size(em_tomo_rotated);
    x_pos = round(em_tomo_shift_x); y_pos = round(em_tomo_shift_y);
    x_start = max(1,1+x_pos); y_start = max(1,1+y_pos);
    x_img_start = max(1,1-x_pos); y_img_start = max(1,1-y_pos);
    x_end = min(emW, x_start+wRT-x_img_start); y_end = min(emH, y_start+hRT-y_img_start);
    width = x_end-x_start+1; height = y_end-y_start+1;
    if width>0 && height>0
        canvas(y_start:y_end, x_start:x_end) = ...
            em_tomo_rotated(y_img_start:y_img_start+height-1, x_img_start:x_img_start+width-1);
    end
    em_tomo_out = canvas;

    %% --- Transform FL image ---
    % --- Transform FL image for export ---
fl_transformed = im2double(FL_image);
fl_transformed = (fl_transformed - fl_low) / (fl_high - fl_low); % normalize
fl_transformed = max(min(fl_transformed, 1), 0);

if fl_flip_h, fl_transformed = fliplr(fl_transformed); end
if fl_flip_v, fl_transformed = flipud(fl_transformed); end

fl_transformed = imrotate(fl_transformed, fl_angle, 'bilinear', 'crop');
fl_transformed = imresize(fl_transformed, fl_scale);
fl_transformed = imtranslate(fl_transformed, [fl_shift_x*3, fl_shift_y*3], 'FillValues',0);

% **Force exact size to EM**
% --- FORCE FL TO GRAYSCALE ---
if ndims(fl_transformed) == 3
    fl_gray = rgb2gray(fl_transformed);
else
    fl_gray = fl_transformed;
end

fl_resized = imresize(fl_gray, [emH, emW]);

nColors = 256;
redhotMap = hot(nColors);
fl_idx = round(fl_resized*(nColors-1)) + 1;


fl_idx(fl_idx < 1) = 1;
fl_idx(fl_idx > nColors) = nColors;

% Convert indices to RGB safely
fl_rgb = zeros(emH, emW, 3);

fl_rgb = reshape(redhotMap(fl_idx(:), :), emH, emW, 3);




    %% --- Save aligned images ---
    imwrite(im2uint8(rgb_double), fullfile(path, ['aligned_rgb_' file]));
    imwrite(uint16(em_norm_out*65535), fullfile(path, ['aligned_em_' file]), 'Compression','none');
    imwrite(uint16(em_tomo_out*65535), fullfile(path, ['aligned_em_tomo_' file]), 'Compression','none');
    imwrite(im2uint8(fl_rgb), fullfile(path, ['aligned_fluorescence_' file]));

    %% --- Create overlay (RGB+EM+EM Tomo+FL) ---
    overlay_img = zeros(emH, emW, 3);
    overlay_img(:,:,1) = max(max(em_norm_out, em_tomo_out), rgb_double(:,:,1));
    overlay_img(:,:,2) = max(max(em_norm_out, em_tomo_out), rgb_double(:,:,2));
    overlay_img(:,:,3) = max(max(em_norm_out, em_tomo_out), rgb_double(:,:,3));

    mask = fl_resized>0;
    mask3 = repmat(mask,[1 1 3]);
    overlay_img(mask3) = max(overlay_img(mask3), fl_rgb(mask3));

    imwrite(im2uint8(overlay_img), fullfile(path, ['aligned_overlay_' file]));

    disp('All aligned images exported successfully.');
end



function update_display()

%% ================= READ UI VALUES =================
angle = sRot.Value;
shift_x = sX.Value;
shift_y = sY.Value;
scale_factor = sScale.Value;

do_flip_h = btnFlipH.Value;
do_flip_v = btnFlipV.Value;

em_tomo_angle   = sETRot.Value;
em_tomo_shift_x = sETX.Value;
em_tomo_shift_y = sETY.Value;
em_tomo_scale   = sETScale.Value;

em_low      = sEMLow.Value;
em_high     = sEMHigh.Value;
em_tomo_low = sEMTomoLow.Value;
em_tomo_high= sEMTomoHigh.Value;
rgb_low     = sRGBLow.Value;
rgb_high    = sRGBHigh.Value;

% --- FL controls ---
fl_low     = sFLLow.Value;
fl_high    = sFLHigh.Value;
fl_angle   = sFLRot.Value;
fl_shift_x = sFLX.Value;
fl_shift_y = sFLY.Value;
fl_scale   = sFLScale.Value;
fl_flip_h  = btnFLFlipH.Value;
fl_flip_v  = btnFLFlipV.Value;

FL_SHIFT_SCALE = 3;

show_rgb     = findobj(hFig,'String','Show RGB Image').Value;
show_em      = findobj(hFig,'String','Show EM Image').Value;
show_em_tomo = findobj(hFig,'String','Show EM Tomo Image').Value;
show_fl      = findobj(hFig,'String','Show FL Image').Value;

%% ================= NORMALIZE EM =================
em_norm = (em_img_double - em_min) / (em_max - em_min);
em_norm = max(min((em_norm - em_low) / (em_high - em_low),1),0);

%% ================= NORMALIZE EM TOMO =================
if em_tomo_max == em_tomo_min
    em_tomo_norm = zeros(size(em_tomo_double));
else
    em_tomo_norm = (em_tomo_double - em_tomo_min) / (em_tomo_max - em_tomo_min);
end
em_tomo_norm = max(min((em_tomo_norm - em_tomo_low) / ...
                       (em_tomo_high - em_tomo_low),1),0);

% Scale → rotate → place on EM canvas
em_tomo_scaled  = imresize(em_tomo_norm, em_tomo_scale);
em_tomo_rotated = imrotate(em_tomo_scaled, em_tomo_angle,'bilinear','loose');

canvas = zeros(emH, emW);
[hRT,wRT] = size(em_tomo_rotated);

x_pos = round(em_tomo_shift_x);
y_pos = round(em_tomo_shift_y);

x_start = max(1,1+x_pos);
y_start = max(1,1+y_pos);
x_img   = max(1,1-x_pos);
y_img   = max(1,1-y_pos);

x_end = min(emW, x_start + wRT - x_img);
y_end = min(emH, y_start + hRT - y_img);

if x_end>=x_start && y_end>=y_start
    canvas(y_start:y_end,x_start:x_end) = ...
        em_tomo_rotated(y_img:y_img+(y_end-y_start), ...
                        x_img:x_img+(x_end-x_start));
end

em_tomo_transformed = canvas;

%% ================= RGB IMAGE =================
%% ================= RGB IMAGE (FIXED) =================

rgb_trans = rgb_img;

% Flip
if do_flip_h, rgb_trans = fliplr(rgb_trans); end
if do_flip_v, rgb_trans = flipud(rgb_trans); end

% Rotate and scale
rgb_trans = imrotate(rgb_trans, angle, 'bilinear', 'crop');
rgb_trans = imresize(rgb_trans, scale_factor);

% Create EM-sized canvas
rgb_canvas = zeros(emH, emW, size(rgb_trans,3));
[hRGB, wRGB, ~] = size(rgb_trans);

% Scale slider shifts (fine control + large travel)
RGB_SHIFT_SCALE = 3;
dx = round(shift_x * RGB_SHIFT_SCALE);
dy = round(shift_y * RGB_SHIFT_SCALE);

% Safe placement with cropping
x_start = max(1, 1 + dx);
y_start = max(1, 1 + dy);
x_img_start = max(1, 1 - dx);
y_img_start = max(1, 1 - dy);

x_end = min(emW, x_start + wRGB - x_img_start);
y_end = min(emH, y_start + hRGB - y_img_start);

if x_end >= x_start && y_end >= y_start
    rgb_canvas(y_start:y_end, x_start:x_end, :) = ...
        rgb_trans(y_img_start:y_img_start + (y_end-y_start), ...
                  x_img_start:x_img_start + (x_end-x_start), :);
end

% Convert to grayscale
rgb_gray = im2double(rgb_canvas);
if ndims(rgb_gray) == 3
    rgb_gray = rgb_gray(:,:,1);
end

% Normalize using sliders
rgb_gray = (rgb_gray - rgb_min) / (rgb_max - rgb_min);
rgb_gray = max(min((rgb_gray - rgb_low) / (rgb_high - rgb_low), 1), 0);

if ndims(rgb_gray)==3
    rgb_gray = rgb_gray(:,:,1);
end

%% ================= BASE OVERLAY =================
overlay_img = zeros(emH,emW,3);

if show_rgb
    overlay_img(:,:,2) = max(overlay_img(:,:,2), rgb_gray);
end

if show_em
    overlay_img = max(overlay_img, repmat(em_norm,[1 1 3]));
end

if show_em_tomo
    overlay_img = max(overlay_img, repmat(em_tomo_transformed,[1 1 3]));
end

%% ================= FL IMAGE (FIXED, SINGLE PIPELINE) =================
if show_fl

    % --- FORCE GRAYSCALE (FIXES 3 COPIES BUG) ---
    if ndims(FL_image)==3
        fl_gray = rgb2gray(FL_image);
    else
        fl_gray = FL_image;
    end
    fl_gray = im2double(fl_gray);

    % Normalize
    fl_norm = (fl_gray - fl_low)/(fl_high-fl_low);
    fl_norm = max(min(fl_norm,1),0);

    if fl_flip_h, fl_norm = fliplr(fl_norm); end
    if fl_flip_v, fl_norm = flipud(fl_norm); end

    fl_rot  = imrotate(fl_norm, fl_angle,'bilinear','crop');
    fl_res  = imresize(fl_rot, fl_scale);

    [hFL,wFL] = size(fl_res);
    fl_canvas = zeros(emH,emW);

    xc = floor((emW-wFL)/2)+1 + round(fl_shift_x*FL_SHIFT_SCALE);
    yc = floor((emH-hFL)/2)+1 + round(fl_shift_y*FL_SHIFT_SCALE);

    xs = max(1,xc); ys = max(1,yc);
    xe = min(emW,xc+wFL-1);
    ye = min(emH,yc+hFL-1);

    xi = 1 + max(0,1-xc);
    yi = 1 + max(0,1-yc);

    if xe>=xs && ye>=ys
        fl_canvas(ys:ye,xs:xe) = ...
            fl_res(yi:yi+(ye-ys),xi:xi+(xe-xs));
    end

    % Hot colormap
    cmap = hot(256);
    idx = round(fl_canvas*255)+1;
    idx = min(max(idx,1),256);
    fl_rgb = reshape(cmap(idx,:),[emH emW 3]);

    mask = fl_canvas>0;
    overlay_img(repmat(mask,[1 1 3])) = ...
        max(overlay_img(repmat(mask,[1 1 3])), fl_rgb(repmat(mask,[1 1 3])));
end

%% ================= DISPLAY =================
set(hImg,'CData',overlay_img);
drawnow;

end

end
