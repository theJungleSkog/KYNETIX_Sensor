%% KYNETIX FSR insole: ground contact time (GCT) check from ONE video  (version 3)
% The video shows the shoe stomping AND the laptop running ground_contact_live.html.
% Press Run. Pop-up instructions walk you through it:
%   1. Click a few regions once (laptop screen, foot map, big number, a floor mark,
%      the shoe area, and the orange liner on the bottom of the sole).
%   2. MATLAB follows the screen, the floor and the orange sole liner through the video
%      and finds every touchdown and toe-off AUTOMATICALLY (no frame-by-frame marking).
%   3. You review each stomp on a strip of zoomed frames and fix it by +/-1 frame if needed.
%   4. You type the GCT number the website showed for each stomp.
% Output: a formatted Excel report (summary, per-stomp table, charts), PNG charts and a CSV.
%
% Needs base MATLAB only. On Windows with Excel installed, the report gets formatting and
% charts inside the workbook; otherwise you still get the .xlsx data + PNG charts.
% With the Computer Vision Toolbox, the website number is pre-filled (you can correct it).
%
% Accuracy: at 30 fps one frame = 33 ms, so each touchdown/toe-off is known to about
% +/-1 frame. Treat this as a check that the system works, not the formal validation.

clear; close all; clc;

%% ======================= SETTINGS =======================
VIDEO_FILE     = 'IMG_1797.MOV';
START_S        = 6;       % ignore footage before this time (s)
END_S          = Inf;     % stop at this time (s); Inf = end of video
TRIAL_ID       = 'FSR_video_trial1';
LONG_STOMP_MS  = 400;     % stomps longer than this are reported as "long holds"
% --- tracking (usually no need to change) ---
CACHE_MB       = 400;     % memory for the zoomed shoe frames used in the review step
SEARCH_PX      = 60;      % max movement of the screen / floor mark between frames (pixels)
DS_SCREEN      = 4;       % downsampling for screen tracking
DS_FLOOR       = 2;       % downsampling for floor tracking
MIN_TRACK      = 0.40;    % tracking score below this = target lost in that frame
GLOW_ON_FRAC   = 0.35;    % website contact ON when foot-map glow > this x typical peak
GLOW_OFF_FRAC  = 0.20;    % website contact OFF when glow < this x typical peak
MIN_ON_FRAMES  = 3;       % ignore website "contacts" shorter than this (frames)
SOLE_HUE_TOL   = 0.05;    % how close a pixel's colour must be to the clicked liner colour (0-1 hue)
SOLE_MIN_PIX   = 8;       % pixels of liner colour a row needs to count as the sole's bottom edge
DIP_MIN_PX     = 40;      % a stomp = the sole drops at least this many pixels and comes back up
CONTACT_TOL_PX = 12;      % "on the ground" = within this many pixels of the stomp's lowest point
WEB_LAG_S      = 0.5;     % the website display can lag the shoe by up to this much (pairing only)
% =========================================================

marksFile = [TRIAL_ID '_review.mat'];
v = VideoReader(VIDEO_FILE);
fps = v.FrameRate; frameMs = 1000 / fps;
if ~isfinite(END_S) || END_S > v.Duration, END_S = v.Duration; end

uiwait(msgbox({ ...
    'This tool checks the website''s ground contact time against the video.', '', ...
    'STEP 1  Click 6 things in one frame (instructions appear in the window title).', ...
    'STEP 2  Wait while MATLAB follows the screen, the floor and the shoe''s orange sole liner.', ...
    '            It finds every touchdown and toe-off for you.', ...
    'STEP 3  Review each stomp on a strip of zoomed frames; fix it by a frame if needed.', ...
    'STEP 4  Type the GCT number the website showed for each stomp.', '', ...
    'You then get an Excel report with a summary, a per-stomp table and charts.'}, ...
    'GCT video check', 'help'));

%% STEP 1-2: regions + tracking (repeat until the tracking looks right)
v.CurrentTime = START_S;
f0 = readFrame(v);
ROT = askRotation(f0);
f0 = rotFrame(f0, ROT);
[H, W, ~] = size(f0);

accepted = false;
while ~accepted
    R1 = selectRegions(f0);
    scrW = R1.scr(3) - R1.scr(1) + 1; scrH = R1.scr(4) - R1.scr(2) + 1;
    Ts0 = blockDown(toGray(f0(R1.scr(2):R1.scr(4), R1.scr(1):R1.scr(3), :)), DS_SCREEN);
    Tf0 = blockDown(toGray(f0(R1.floor(2):R1.floor(4), R1.floor(1):R1.floor(3), :)), DS_FLOOR);
    scx = mean(R1.scr([1 3])); scy = mean(R1.scr([2 4])); ssc = 1.0;
    fcx0 = mean(R1.floor([1 3])); fcy0 = mean(R1.floor([2 4])); fcx = fcx0; fcy = fcy0;
    shoeW = R1.shoe(3) - R1.shoe(1) + 1; shoeH = R1.shoe(4) - R1.shoe(2) + 1;

    nEst = ceil((END_S - START_S) * fps) + 5;
    zs = min(1, sqrt(CACHE_MB * 1e6 / (nEst * shoeW * shoeH * 3)));
    zr = round(linspace(1, shoeH, max(10, round(shoeH * zs))));
    zc = round(linspace(1, shoeW, max(10, round(shoeW * zs))));
    zoom = zeros(numel(zr), numel(zc), 3, nEst, 'uint8');

    tF = nan(nEst,1); glow = nan(nEst,1); sScore = nan(nEst,1); fScore = nan(nEst,1);
    strk = nan(nEst,3); ftrk = nan(nEst,2); soleY = nan(nEst,1); numCrop = cell(nEst,1);
    v.CurrentTime = START_S;
    wb = waitbar(0, 'Step 2: following the screen, the floor and the shoe...');
    k = 0;
    while hasFrame(v) && v.CurrentTime <= END_S && k < nEst
        k = k + 1;
        tF(k) = v.CurrentTime;
        fr = rotFrame(readFrame(v), ROT);
        Gf = toGray(fr);

        % laptop screen -> foot-map glow + number crop
        [ok, scx, scy, ssc, sScore(k)] = trackTemplate(blockDown(Gf, DS_SCREEN), Ts0, scx, scy, ssc, DS_SCREEN, SEARCH_PX, MIN_TRACK, true);
        if ok
            strk(k,:) = [scx scy ssc];
            foot = cropRel(fr, R1.foot, scx, scy, ssc, scrW, scrH);
            if ~isempty(foot)
                hsv = rgb2hsv(foot);
                glow(k) = mean(mean(hsv(:,:,2) > 0.43 & hsv(:,:,3) > 0.35));
            end
            numCrop{k} = cropRel(fr, R1.numb, scx, scy, ssc, scrW, scrH);
        end

        % floor mark -> camera movement near the shoe
        [ok, fcx, fcy, ~, fScore(k)] = trackTemplate(blockDown(Gf, DS_FLOOR), Tf0, fcx, fcy, 1, DS_FLOOR, SEARCH_PX, MIN_TRACK, false);
        if ok, ftrk(k,:) = [fcx fcy]; end

        % shoe area moves with the floor; find the sole's bottom edge (orange liner)
        ox = round(R1.shoe(1) + fcx - fcx0); oy = round(R1.shoe(2) + fcy - fcy0);
        sh = cropPad(fr, ox, oy, shoeW, shoeH);
        zoom(:, :, :, k) = sh(zr, zc, :);
        hsv = rgb2hsv(sh);
        dh = abs(hsv(:,:,1) - R1.hue); dh = min(dh, 1 - dh);
        m = dh < SOLE_HUE_TOL & hsv(:,:,2) > R1.sMin & hsv(:,:,3) > R1.vMin;
        rowsOK = find(sum(m, 2) >= SOLE_MIN_PIX, 1, 'last');
        if ~isempty(rowsOK), soleY(k) = oy + rowsOK - 1; end

        if mod(k, 20) == 0, waitbar(min(1, (tF(k) - START_S) / (END_S - START_S)), wb); end
    end
    close(wb);
    nF = k; keep = 1:nF;
    tF = tF(keep); glow = glow(keep); sScore = sScore(keep); fScore = fScore(keep);
    strk = strk(keep,:); ftrk = ftrk(keep,:); soleY = soleY(keep); numCrop = numCrop(keep);
    zoom = zoom(:, :, :, keep);

    % sole height above the floor mark (bigger = shoe higher)
    fy = fillMissing(ftrk(:,2));
    hgt = medfilt3pt(fillMissing(fy - soleY));
    accepted = checkTracking(f0, R1, v, START_S, tF, strk, ftrk, soleY, fcx0, fcy0, scrW, scrH, ROT, hgt, glow);
end
fprintf('Tracked %d frames. Screen lost in %d, floor lost in %d, sole not found in %d.\n', ...
        nF, sum(isnan(strk(:,1))), sum(isnan(ftrk(:,1))), sum(isnan(soleY)));

%% Website contacts (from the foot-map glow) and shoe contacts (from the sole height)
g = glow; g(isnan(g)) = 0;
web = glowContacts(g, GLOW_ON_FRAC, GLOW_OFF_FRAC, MIN_ON_FRAMES);
auto = findStomps(hgt, DIP_MIN_PX, CONTACT_TOL_PX);        % rows: [touchdownFrame toeOffFrame]
fprintf('Found %d stomps in the video and %d contacts on the website.\n', size(auto,1), size(web,1));

% website contacts with no matching shoe stomp get a proposal too, so nothing is missed
cand = [auto, ones(size(auto,1),1)];                         % 3rd column: 1 = found by tracker, 2 = website only
lagF = round(WEB_LAG_S * fps);
for i = 1:size(web, 1)
    hit = any(auto(:,1) <= web(i,2) & auto(:,2) + lagF >= web(i,1));
    if ~hit
        cand(end+1, :) = [max(1, web(i,1) - 3), min(nF, max(web(i,1) - 1, web(i,2) - lagF)), 2]; %#ok<AGROW>
    end
end
cand = sortrows(cand, 1);

%% STEP 3: review each stomp
if exist(marksFile, 'file')
    a = questdlg('You reviewed this video before. Load that review?', 'Saved review found', 'Yes, load it', 'No, review again', 'Yes, load it');
    if strcmp(a, 'Yes, load it')
        L = load(marksFile);
        if isfield(L, 'cand') && isfield(L, 'status') && L.nF == nF, cand = L.cand; status = L.status; end
    end
end
if ~exist('status', 'var') || numel(status) ~= size(cand, 1), status = zeros(size(cand,1), 1); end  % 0 = unchecked, 1 = accepted, -1 = rejected

uiwait(msgbox({ ...
    'STEP 3: REVIEW EACH STOMP', '', ...
    'MATLAB found the stomps for you. For each one you see two rows of zoomed frames:', ...
    '   TOP row    = around TOUCHDOWN.  GREEN frames = shoe on the ground.', ...
    '   BOTTOM row = around TOE-OFF.    The first RED frame after green = shoe has left the floor.', ...
    'The cyan line shows where MATLAB found the bottom of the sole.', '', ...
    'If it looks right, click  "Correct, next"  (or press Enter).', ...
    'If touchdown or toe-off is off by a frame, use the arrow buttons to shift it.', ...
    'If it is not a real stomp (foot just hovering), click  "Not a stomp".', '', ...
    'Keyboard: A / S = touchdown earlier / later,  K / L = toe-off earlier / later,', ...
    '          Enter = correct,  X = not a stomp,  B = back to previous stomp.'}, 'How to review', 'help'));

j = find(status == 0, 1); if isempty(j), j = 1; end
rv = figure('Name', 'Step 3: review stomps', 'NumberTitle', 'off', 'Units', 'normalized', ...
            'Position', [0.03 0.05 0.94 0.88], 'Color', 'w', 'KeyPressFcn', @reviewKey, ...
            'CloseRequestFcn', @(s,e) setAct(s, 'quit'));
makeReviewButtons(rv);
while j >= 1 && j <= size(cand, 1)
    drawReview(rv, zoom, tF, cand(j,:), j, size(cand,1), soleY, ftrk, fcy0, R1, zr, nF, frameMs, status(j));
    setappdata(rv, 'act', '');
    uiwait(rv);
    if ~ishandle(rv), break; end
    act = getappdata(rv, 'act');
    switch act
        case 'tdE', cand(j,1) = max(1, cand(j,1) - 1);
        case 'tdL', cand(j,1) = min(cand(j,2) - 1, cand(j,1) + 1);
        case 'toE', cand(j,2) = max(cand(j,1) + 1, cand(j,2) - 1);
        case 'toL', cand(j,2) = min(nF, cand(j,2) + 1);
        case 'ok',   status(j) = 1;  j = j + 1;
        case 'rej',  status(j) = -1; j = j + 1;
        case 'back', j = max(1, j - 1);
        case 'quit'
            a = questdlg('Stop reviewing? Stomps you have not checked will be left out.', 'Stop?', 'Keep reviewing', 'Stop and continue', 'Keep reviewing');
            if strcmp(a, 'Stop and continue'), break; end
    end
    save(marksFile, 'cand', 'status', 'nF');
end
if ishandle(rv), delete(rv); end
save(marksFile, 'cand', 'status', 'nF');

steps = cand(status == 1, 1:2);
src = cand(status == 1, 3);
nS = size(steps, 1);
fprintf('%d stomps accepted after review.\n', nS);
if nS == 0
    uiwait(msgbox('No stomps were accepted, so there is nothing to compare. Run the script again to review.', 'Nothing to compare', 'warn'));
    return;
end

%% Pair each accepted stomp with the website contact (website display may lag the shoe)
webIdx = nan(nS, 1);
for jj = 1:nS
    best = 0;
    for i = 1:size(web, 1)
        ov = min(steps(jj,2) + lagF, web(i,2)) - max(steps(jj,1), web(i,1));
        if ov > best, best = ov; webIdx(jj) = i; end
    end
end
usedWeb = unique(webIdx(~isnan(webIdx)));
extraWeb = setdiff(1:size(web,1), usedWeb);
rejectedOverlap = false(size(extraWeb));
for q = 1:numel(extraWeb)                       % website contacts the reviewer rejected as "not a stomp"
    i = extraWeb(q);
    rej = cand(status == -1, :);
    rejectedOverlap(q) = any(rej(:,1) <= web(i,2) & rej(:,2) + lagF >= web(i,1));
end

%% STEP 4: read the website number for each stomp
uiwait(msgbox({'STEP 4: READ THE WEBSITE NUMBER', '', ...
               'For each stomp you will see the big GCT number from the screen, just after the stomp.', ...
               'Type the number you can read (ms). If it is blurry or missing, leave it blank.'}, ...
              'Read the website number', 'help'));
haveOCR = exist('ocr', 'file') == 2;
webNum = nan(nS, 1);
prevNum = '';
for jj = 1:nS
    if isnan(webIdx(jj)), continue; end
    offF = web(webIdx(jj), 2);
    cf = offF + round([0.15 0.3 0.5 0.8] * fps); cf = cf(cf <= nF);
    if isempty(cf), continue; end
    hn = figure('Name', sprintf('Stomp %d of %d: website number', jj, nS), 'NumberTitle', 'off', ...
                'Units', 'normalized', 'Position', [0.15 0.35 0.7 0.35]);
    guess = '';
    for q = 1:numel(cf)
        subplot(1, numel(cf), q);
        im = numCrop{cf(q)};
        if ~isempty(im)
            image(im); axis image off; title(sprintf('%.2f s after the stomp', tF(cf(q)) - tF(offF)));
            if haveOCR && isempty(guess)
                try
                    o = ocr(im, 'CharacterSet', '0123456789', 'TextLayout', 'Word');
                    w = regexp(o.Text, '\d{2,4}', 'match', 'once');
                    if ~isempty(w), guess = w; end
                catch
                end
            end
        end
    end
    sgtitle(sprintf('Stomp %d of %d (video GCT %.0f ms): what number does the website show?', ...
            jj, nS, (tF(steps(jj,2)) - tF(steps(jj,1))) * 1000), 'FontWeight', 'bold');
    a = inputdlg({sprintf(['Stomp %d of %d: type the GCT number shown (ms).\n' ...
                  'Leave blank if unreadable. (Last number you typed: %s)'], jj, nS, prevNum)}, ...
                 'Website GCT', 1, {guess});
    close(hn);
    if ~isempty(a) && ~isempty(strtrim(a{1}))
        webNum(jj) = str2double(a{1}); prevNum = strtrim(a{1});
    end
end

%% Results
gctVideo = (tF(steps(:,2)) - tF(steps(:,1))) * 1000;
diffMs = webNum - gctVideo;
isLong = gctVideo >= LONG_STOMP_MS;
srcTxt = repmat({'tracker'}, nS, 1); srcTxt(src == 2) = {'website only (set in review)'};
withinTxt = repmat({''}, nS, 1);
for jj = 1:nS
    if isnan(diffMs(jj)),                    withinTxt{jj} = 'no website number';
    elseif abs(diffMs(jj)) <= frameMs + 1,   withinTxt{jj} = 'within 1 video frame';
    elseif abs(diffMs(jj)) <= 2*frameMs + 1, withinTxt{jj} = 'within 2 video frames';
    else,                                    withinTxt{jj} = 'more than 2 frames off';
    end
end
typeTxt = repmat({'quick stomp'}, nS, 1); typeTxt(isLong) = {'long hold'};

Stomp = (1:nS)';
Type = typeTxt;
Touchdown_s = round(tF(steps(:,1)) * 100) / 100;
ToeOff_s = round(tF(steps(:,2)) * 100) / 100;
Video_GCT_ms = round(gctVideo);
Website_GCT_ms = webNum;
Website_minus_video_ms = round(diffMs);
Agreement = withinTxt;
Found_by = srcTxt;
Tres = table(Stomp, Type, Touchdown_s, ToeOff_s, Video_GCT_ms, Website_GCT_ms, Website_minus_video_ms, Agreement, Found_by);
disp(Tres);
writetable(Tres, [TRIAL_ID '_results.csv']);

% plain-language summary
okN = ~isnan(webNum);
nMissed = sum(isnan(webIdx));
nExtra = sum(~rejectedOverlap);
S = {};
S{end+1} = sprintf('Video: %d stomps accepted (%d quick stomps, %d long holds).', nS, sum(~isLong), sum(isLong));
S{end+1} = sprintf('The website registered %d of these %d stomps.', nS - nMissed, nS);
if nMissed > 0, S{end+1} = sprintf('   It missed %d stomp(s) (see the table, "Website_GCT_ms" is empty).', nMissed); end
if nExtra > 0,  S{end+1} = sprintf('   It showed %d extra contact(s) the video did not confirm as a stomp.', nExtra); end
S{end+1} = '';
S = [S, groupSummary('Quick stomps', diffMs(okN & ~isLong), gctVideo(okN & ~isLong), frameMs)];
S = [S, groupSummary('Long holds',   diffMs(okN & isLong),  gctVideo(okN & isLong),  frameMs)];
S{end+1} = sprintf(['For scale: 1 video frame = %.0f ms. Each video touchdown/toe-off is known to about +/-1 frame, ' ...
                    'so differences up to ~%.0f ms are within what this video can measure.'], frameMs, frameMs);
S{end+1} = 'What matters here is the GCT value itself; the screen''s display delay does not affect it.';
fprintf('\n%s\n', strjoin(S, newline));

% charts
figs = makeCharts(gctVideo, webNum, isLong, frameMs, tF, hgt, steps, web, g, TRIAL_ID);
for q = 1:numel(figs), exportgraphics(figs(q).h, figs(q).file, 'Resolution', 150); end

% Excel report
xlsxFile = fullfile(pwd, [TRIAL_ID '_report.xlsx']);
writeReport(xlsxFile, Tres, S, figs, TRIAL_ID, VIDEO_FILE, START_S, END_S, fps);

uiwait(msgbox([S, {'', sprintf('Saved: %s_report.xlsx (summary, table, charts), %s_results.csv and the chart PNGs.', TRIAL_ID, TRIAL_ID)}], 'Results', 'help'));

%% ======================= LOCAL FUNCTIONS =======================
function ROT = askRotation(f)
    hf = figure('Name', 'Orientation check', 'NumberTitle', 'off');
    image(f); axis image off; title('Is this picture upright?', 'FontSize', 14);
    a = questdlg('Is the picture behind this box upright (shoe at the bottom, laptop above it)?', 'Orientation', ...
                 'Yes, it is upright', 'Rotate 90 deg clockwise', 'Rotate 90 deg counter-clockwise', 'Yes, it is upright');
    close(hf);
    switch a
        case 'Rotate 90 deg clockwise',         ROT = -1;
        case 'Rotate 90 deg counter-clockwise', ROT = 1;
        otherwise,                              ROT = 0;
    end
end

function R = selectRegions(f0)
    hf = figure('Name', 'Step 1: click the regions', 'NumberTitle', 'off', 'Units', 'normalized', 'Position', [0.1 0.04 0.8 0.9]);
    done = false;
    while ~done
        clf(hf); image(f0); axis image off;
        R.scr = clickBox(f0, {'STEP 1 (1 of 6): LAPTOP SCREEN', 'Click the TOP-LEFT corner of the laptop screen, then the BOTTOM-RIGHT corner'});
        scr = f0(R.scr(2):R.scr(4), R.scr(1):R.scr(3), :);
        clf(hf); image(scr); axis image off;
        R.foot = clickBox(scr, {'STEP 1 (2 of 6): FOOT MAP (zoomed screen)', 'Click TOP-LEFT, then BOTTOM-RIGHT of the insole picture'});
        hold on; rectangle('Position', [R.foot(1) R.foot(2) R.foot(3)-R.foot(1) R.foot(4)-R.foot(2)], 'EdgeColor', 'g', 'LineWidth', 2);
        R.numb = clickBox(scr, {'STEP 1 (3 of 6): BIG GCT NUMBER', 'Click TOP-LEFT, then BOTTOM-RIGHT of the big number (e.g. "340 ms")'});
        clf(hf); image(f0); axis image off;
        R.floor = clickBox(f0, {'STEP 1 (4 of 6): FLOOR MARK', ...
            'Click TOP-LEFT, then BOTTOM-RIGHT of a mark on the FLOOR near the shoe (e.g. the blue tape). It must stay visible.'});
        hold on; rectangle('Position', [R.floor(1) R.floor(2) R.floor(3)-R.floor(1) R.floor(4)-R.floor(2)], 'EdgeColor', 'b', 'LineWidth', 2);
        R.shoe = clickBox(f0, {'STEP 1 (5 of 6): SHOE AREA', ...
            'Click TOP-LEFT, then BOTTOM-RIGHT of a GENEROUS box: from above the highest lift down to below the floor, full shoe width'});
        rectangle('Position', [R.shoe(1) R.shoe(2) R.shoe(3)-R.shoe(1) R.shoe(4)-R.shoe(2)], 'EdgeColor', 'c', 'LineWidth', 2);
        title({'STEP 1 (6 of 6): SOLE COLOUR', 'Click once ON the orange liner at the bottom of the sole'}, 'FontSize', 13);
        [xc, yc] = ginput(1); xc = round(xc); yc = round(yc);
        p = f0(max(1,yc-2):min(end,yc+2), max(1,xc-2):min(end,xc+2), :);
        hsvp = rgb2hsv(p); hh = hsvp(:,:,1); ss = hsvp(:,:,2); vv = hsvp(:,:,3);
        R.hue = angle(mean(exp(2i*pi*hh(:)))) / (2*pi); if R.hue < 0, R.hue = R.hue + 1; end
        R.sMin = max(0.25, 0.55 * median(ss(:))); R.vMin = max(0.25, 0.5 * median(vv(:)));

        % show the liner detection on this frame so the user can judge it
        sh = f0(R.shoe(2):R.shoe(4), R.shoe(1):R.shoe(3), :);
        hsv = rgb2hsv(sh); dh = abs(hsv(:,:,1) - R.hue); dh = min(dh, 1 - dh);
        m = dh < 0.05 & hsv(:,:,2) > R.sMin & hsv(:,:,3) > R.vMin;
        [yy, xx] = find(m);
        clf(hf); image(f0); axis image off; hold on;
        plot(xx + R.shoe(1) - 1, yy + R.shoe(2) - 1, '.', 'Color', [0 1 0], 'MarkerSize', 4);
        rectangle('Position', [R.scr(1) R.scr(2) R.scr(3)-R.scr(1) R.scr(4)-R.scr(2)], 'EdgeColor', 'y', 'LineWidth', 2);
        rectangle('Position', [R.floor(1) R.floor(2) R.floor(3)-R.floor(1) R.floor(4)-R.floor(2)], 'EdgeColor', 'b', 'LineWidth', 2);
        rectangle('Position', [R.shoe(1) R.shoe(2) R.shoe(3)-R.shoe(1) R.shoe(4)-R.shoe(2)], 'EdgeColor', 'c', 'LineWidth', 2);
        title({'Yellow = screen, blue = floor mark, cyan = shoe area', 'GREEN dots = pixels matching the sole liner colour (should trace the bottom of the sole)'}, 'FontSize', 13);
        a = questdlg({'Check the picture:', '- the boxes are in the right places', ...
                      '- the GREEN dots follow the orange liner along the bottom of the sole', '', ...
                      '(A few green dots elsewhere are fine, as long as none are BELOW the sole.)'}, ...
                     'Step 1: check', 'Yes, start tracking', 'No, redo the clicks', 'Yes, start tracking');
        done = ~strcmp(a, 'No, redo the clicks');
    end
    close(hf);
end

function b = clickBox(img, ttl)
    title(ttl, 'FontSize', 13);
    [x, y] = ginput(2);
    b = [max(1, round(min(x))) max(1, round(min(y))) min(size(img,2), round(max(x))) min(size(img,1), round(max(y)))];
end

function ok = checkTracking(f0, R1, v, START_S, tF, strk, ftrk, soleY, fcx0, fcy0, scrW, scrH, ROT, hgt, glow) %#ok<INUSL>
    nF = numel(tF);
    hc = figure('Name', 'Step 2: tracking check', 'NumberTitle', 'off', 'Units', 'normalized', 'Position', [0.03 0.05 0.94 0.88]);
    idx = round(linspace(1, nF, 6));
    vc = VideoReader(fullfile(v.Path, v.Name));
    for i = 1:6
        jj = idx(i);
        vc.CurrentTime = tF(jj); fr = rotFrame(readFrame(vc), ROT);
        subplot(3, 6, [i i+6]); image(fr); axis image off; hold on;
        title(sprintf('t = %.1f s', tF(jj)));
        if all(isfinite(strk(jj,:)))
            drawRel(strk(jj,:), [0 0 scrW scrH], scrW, scrH, 'y');
        end
        if all(isfinite(ftrk(jj,:)))
            fw = R1.floor(3) - R1.floor(1); fh = R1.floor(4) - R1.floor(2);
            rectangle('Position', [ftrk(jj,1)-fw/2 ftrk(jj,2)-fh/2 fw fh], 'EdgeColor', 'b', 'LineWidth', 1.5);
            ox = R1.shoe(1) + ftrk(jj,1) - fcx0; oy = R1.shoe(2) + ftrk(jj,2) - fcy0;
            rectangle('Position', [ox oy R1.shoe(3)-R1.shoe(1) R1.shoe(4)-R1.shoe(2)], 'EdgeColor', 'c', 'LineWidth', 1);
            if isfinite(soleY(jj)), plot([ox ox + R1.shoe(3) - R1.shoe(1)], soleY(jj)*[1 1], 'g-', 'LineWidth', 2); end
        end
    end
    subplot(3, 6, 13:18);
    yyaxis left; plot(tF, hgt, 'Color', [0.1 0.5 0.9], 'LineWidth', 1); ylabel('Sole height above floor (px)');
    yyaxis right; g = glow; g(isnan(g)) = 0; plot(tF, g, 'Color', [0.95 0.45 0.1]); ylabel('Website foot-map glow');
    xlabel('Time (s)'); grid on; xlim([tF(1) tF(end)]);
    title('Blue = how high the sole is (dips = stomps). Orange = website lighting up. They should line up.');
    sgtitle({'Yellow = screen, blue = floor mark, cyan = shoe area, GREEN line = bottom of the sole found by MATLAB'}, 'FontWeight', 'bold');
    a = questdlg({'Look at the window behind this box:', ...
                  '- the boxes stay on the screen / floor mark in all 6 pictures', ...
                  '- the GREEN line sits on the bottom of the sole', ...
                  '- each blue dip in the graph lines up with an orange website bump', '', ...
                  '(A few frames at the very end may be lost; that is fine.)'}, ...
                 'Step 2: is the tracking right?', 'Yes, continue', 'No, redo the clicks', 'Yes, continue');
    close(hc);
    ok = ~strcmp(a, 'No, redo the clicks');
end

function [ok, cx, cy, sc, best] = trackTemplate(G, T0, cx, cy, sc, DS, SEARCH_PX, MIN_TRACK, allowScale)
    if allowScale, scales = sc * [0.97 1 1.03]; else, scales = sc; end
    best = -Inf; bs = sc; bx = cx; by = cy;
    for st = scales
        if st == 1, T = T0; else, T = resizeBase(T0, st); end
        [th, tw] = size(T);
        Rr = SEARCH_PX / DS;
        wx0 = max(1, round(cx/DS - tw/2 - Rr)); wx1 = min(size(G,2), round(cx/DS + tw/2 + Rr));
        wy0 = max(1, round(cy/DS - th/2 - Rr)); wy1 = min(size(G,1), round(cy/DS + th/2 + Rr));
        Wn = G(wy0:wy1, wx0:wx1);
        if size(Wn,1) < th || size(Wn,2) < tw, continue; end
        [mv, r, c] = nccBest(Wn, T);
        if mv > best
            best = mv; bs = st;
            bx = (wx0 + c - 1 + (tw-1)/2) * DS;
            by = (wy0 + r - 1 + (th-1)/2) * DS;
        end
    end
    ok = best >= MIN_TRACK;
    if ok, cx = bx; cy = by; sc = min(max(bs, 0.6), 1.6); end
end

function web = glowContacts(g, onF, offF, minOn)
    gs = sort(g); ref = max(gs(max(1, round(0.95 * numel(gs)))), eps);
    onThr = onF * ref; offThr = offF * ref;
    web = zeros(0, 2); inC = false; s0 = 1;
    for i = 1:numel(g)
        if ~inC && g(i) > onThr
            inC = true; s0 = i;
        elseif inC && g(i) < offThr
            inC = false;
            if i - s0 >= minOn, web(end+1, :) = [s0 i]; end %#ok<AGROW>
        end
    end
end

function st = findStomps(h, minDepth, tol)
    % stomps = dips in sole height. Touchdown = first frame within tol of the dip's lowest point,
    % toe-off = first frame after that window (shoe has left the floor).
    n = numel(h); st = zeros(0, 2); lastEnd = 0;
    for p = 2:n-1
        if ~(h(p) <= h(p-1) && h(p) < h(p+1)), continue; end
        k = p - 1; mL = h(p);
        while k >= 1 && h(k) >= h(p), mL = max(mL, h(k)); k = k - 1; end
        k = p + 1; mR = h(p);
        while k <= n && h(k) >= h(p), mR = max(mR, h(k)); k = k + 1; end
        if min(mL, mR) - h(p) < minDepth, continue; end
        s = p; while s > 1 && h(s-1) <= h(p) + tol, s = s - 1; end
        e = p; while e < n && h(e+1) <= h(p) + tol, e = e + 1; end
        if s <= lastEnd, continue; end
        st(end+1, :) = [s min(n, e + 1)]; %#ok<AGROW>
        lastEnd = e;
    end
end

function makeReviewButtons(fig)
    b = {'<< Touchdown earlier (A)', 'tdE', 0.02; 'Touchdown later >> (S)', 'tdL', 0.15; ...
         '<< Toe-off earlier (K)', 'toE', 0.30; 'Toe-off later >> (L)', 'toL', 0.43; ...
         'Correct, next (Enter)', 'ok', 0.60; 'Not a stomp (X)', 'rej', 0.75; 'Back (B)', 'back', 0.88};
    for i = 1:size(b, 1)
        uicontrol(fig, 'Style', 'pushbutton', 'String', b{i,1}, 'Units', 'normalized', ...
                  'Position', [b{i,3} 0.015 0.12 0.055], 'FontSize', 11, 'FontWeight', 'bold', ...
                  'Callback', @(s, ~) setAct(ancestor(s, 'figure'), b{i,2}), 'KeyPressFcn', @reviewKey);
    end
    uicontrol(fig, 'Style', 'text', 'Tag', 'info', 'Units', 'normalized', 'Position', [0.02 0.075 0.96 0.05], ...
              'FontSize', 12, 'BackgroundColor', 'w', 'HorizontalAlignment', 'center');
end

function reviewKey(src, evt)
    fig = ancestor(src, 'figure');
    switch lower(evt.Key)
        case 'a', setAct(fig, 'tdE');
        case 's', setAct(fig, 'tdL');
        case 'k', setAct(fig, 'toE');
        case 'l', setAct(fig, 'toL');
        case {'return', 'enter'}, setAct(fig, 'ok');
        case {'x', 'delete'}, setAct(fig, 'rej');
        case 'b', setAct(fig, 'back');
    end
end

function setAct(fig, act)
    setappdata(fig, 'act', act);
    uiresume(fig);
end

function drawReview(fig, zoom, tF, c, j, nC, soleY, ftrk, fcy0, R1, zr, nF, frameMs, st)
    figure(fig);
    delete(findobj(fig, 'Type', 'axes'));
    delete(findall(fig, 'Tag', 'hdr'));
    td = c(1); to = c(2);
    rows = {td-3:td+4, to-4:to+3};
    labels = {'TOUCHDOWN', 'TOE-OFF'};
    zsY = (numel(zr) - 1) / max(1, (R1.shoe(4) - R1.shoe(2)));
    for r = 1:2
        for q = 1:8
            k = rows{r}(q);
            ax = axes('Parent', fig, 'Position', [0.01 + (q-1)*0.1235 0.555 - (r-1)*0.42 0.12 0.36]);
            if k < 1 || k > nF, axis(ax, 'off'); continue; end
            image(ax, zoom(:, :, :, k)); axis(ax, 'image'); axis(ax, 'off'); hold(ax, 'on');
            onG = k >= td && k < to;
            if onG, col = [0 0.75 0]; else, col = [0.85 0 0]; end
            xl = xlim(ax); yl = ylim(ax);
            rectangle(ax, 'Position', [xl(1) yl(1) diff(xl) diff(yl)], 'EdgeColor', col, 'LineWidth', 4);
            if isfinite(soleY(k)) && all(isfinite(ftrk(k,:)))
                oy = R1.shoe(2) + ftrk(k,2) - fcy0;
                yy = (soleY(k) - oy) * zsY + 1;
                plot(ax, xl, [yy yy], 'c-', 'LineWidth', 1);
            end
            lab = sprintf('%.2f s', tF(k));
            if k == td, lab = [lab '  TOUCHDOWN']; end
            if k == to, lab = [lab '  TOE-OFF']; end
            title(ax, lab, 'FontSize', 9, 'Color', col);
        end
        annotation(fig, 'textbox', [0.0 0.92 - (r-1)*0.42 1 0.03], 'String', ...
                   sprintf('%s row: green = on the ground, red = in the air', labels{r}), ...
                   'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Tag', 'hdr');
    end
    srcTxt = ''; if c(3) == 2, srcTxt = '   (MATLAB did not find this one; the website did. Set it or mark "Not a stomp")'; end
    if st == 1, stTxt = 'accepted'; elseif st == -1, stTxt = 'marked not a stomp'; else, stTxt = 'not checked yet'; end
    info = findobj(fig, 'Tag', 'info');
    set(info, 'String', sprintf('Stomp %d of %d   |   touchdown %.2f s, toe-off %.2f s   |   GCT = %.0f ms (%d frames)   |   status: %s%s', ...
        j, nC, tF(td), tF(to), (tF(to) - tF(td)) * 1000, to - td, stTxt, srcTxt));
    drawnow;
end

function L = groupSummary(name, d, gv, frameMs)
    L = {};
    n = numel(d);
    if n == 0, return; end
    bias = mean(d); mae = mean(abs(d));
    w1 = sum(abs(d) <= frameMs + 1); w2 = sum(abs(d) <= 2*frameMs + 1);
    if bias >= 0, word = 'longer'; else, word = 'shorter'; end
    L{end+1} = sprintf('%s (%d with a website number, average video GCT %.0f ms):', name, n, mean(gv));
    L{end+1} = sprintf('   On average the website''s GCT was %.0f ms %s than the video.', abs(bias), word);
    L{end+1} = sprintf('   Typical error per stomp: %.0f ms (%.0f%% of the GCT).', mae, 100 * mae / mean(gv));
    L{end+1} = sprintf('   %d of %d within 1 video frame (%.0f ms), %d of %d within 2 frames.', w1, n, frameMs, w2, n);
    L{end+1} = '';
end

function figs = makeCharts(gv, wn, isLong, frameMs, tF, hgt, steps, web, g, id)
    figs = struct('h', {}, 'file', {}, 'title', {});
    n = numel(gv); ok = ~isnan(wn); d = wn - gv;
    cB = [0.12 0.47 0.85]; cO = [0.95 0.45 0.10];

    % 1) per-stomp comparison: video vs website, quick stomps and long holds side by side
    h1 = figure('Name', 'Video vs website GCT', 'NumberTitle', 'off', 'Color', 'w', 'Position', [80 80 1200 520]);
    groups = {~isLong, isLong}; names = {'Quick stomps', 'Long holds'};
    for p = 1:2
        idx = find(groups{p});
        subplot(1, 2, p); hold on; box on; grid on;
        if isempty(idx), title([names{p} ': none']); axis off; continue; end
        y = 1:numel(idx);
        for q = 1:numel(idx)
            i = idx(q);
            if ok(i), plot([gv(i) wn(i)], [y(q) y(q)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 3); end
        end
        hv = plot(gv(idx), y, 'o', 'MarkerSize', 9, 'MarkerFaceColor', cB, 'MarkerEdgeColor', 'w');
        hw = plot(wn(idx), y, 'd', 'MarkerSize', 9, 'MarkerFaceColor', cO, 'MarkerEdgeColor', 'w');
        for q = 1:numel(idx)
            i = idx(q);
            if ok(i)
                text(max(gv(i), wn(i)), y(q), sprintf('  %+.0f ms', d(i)), 'FontSize', 9, 'VerticalAlignment', 'middle');
            end
        end
        set(gca, 'YTick', y, 'YTickLabel', arrayfun(@(i) sprintf('Stomp %d', i), idx, 'UniformOutput', false), 'YDir', 'reverse');
        xr = [gv(idx); wn(idx(ok(idx)))]; pad = max(30, 0.25 * (max(xr) - min(xr)));
        xlim([max(0, min(xr) - pad) max(xr) + 2*pad]); ylim([0.4 numel(idx) + 0.6]);
        xlabel('Ground contact time (ms)');
        title(sprintf('%s: video (blue) vs website (orange)', names{p}));
        if p == 1 || isempty(find(groups{1}, 1)), legend([hv hw], {'video (truth)', 'website'}, 'Location', 'southeast'); end
    end
    sgtitle('Each row is one stomp. The closer the orange diamond is to the blue circle, the better.', 'FontWeight', 'bold');
    figs(end+1) = struct('h', h1, 'file', [id '_chart_per_stomp.png'], 'title', 'Video vs website GCT per stomp');

    % 2) error per stomp against the video's own precision
    h2 = figure('Name', 'Website error per stomp', 'NumberTitle', 'off', 'Color', 'w', 'Position', [80 620 1200 420]);
    hold on; box on;
    patch([0.4 n+0.6 n+0.6 0.4], [-frameMs -frameMs frameMs frameMs], [0.3 0.75 0.4], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    patch([0.4 n+0.6 n+0.6 0.4], [frameMs frameMs 2*frameMs 2*frameMs], [0.95 0.75 0.2], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    patch([0.4 n+0.6 n+0.6 0.4], [-2*frameMs -2*frameMs -frameMs -frameMs], [0.95 0.75 0.2], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    for i = 1:n
        if ~ok(i), continue; end
        if abs(d(i)) <= frameMs + 1, col = [0.2 0.65 0.3]; elseif abs(d(i)) <= 2*frameMs + 1, col = [0.9 0.65 0.1]; else, col = [0.85 0.2 0.2]; end
        bar(i, d(i), 0.6, 'FaceColor', col, 'EdgeColor', 'none');
        text(i, d(i), sprintf('%+.0f', d(i)), 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', ternary(d(i) >= 0, 'bottom', 'top'), 'FontSize', 9);
        if isLong(i), text(i, 0, 'hold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontSize', 8, 'Color', [0.4 0.4 0.4]); end
    end
    yline(0, 'k-');
    xlim([0.4 n+0.6]); set(gca, 'XTick', 1:n);
    xlabel('Stomp'); ylabel('Website minus video (ms)'); grid on;
    title(sprintf(['How far off the website was on each stomp.  Green band = within 1 video frame (%.0f ms), ' ...
                   'yellow = within 2 frames. Bars above 0 = website longer.'], frameMs));
    figs(end+1) = struct('h', h2, 'file', [id '_chart_error.png'], 'title', 'Website error per stomp');

    % 3) timeline
    h3 = figure('Name', 'Timeline', 'NumberTitle', 'off', 'Color', 'w', 'Position', [80 80 1200 380]);
    yyaxis left; plot(tF, hgt, '-', 'Color', cB); ylabel('Sole height above floor (px)'); hold on;
    yl = ylim;
    for i = 1:size(steps, 1)
        patch(reshape(tF(steps(i,[1 2 2 1])), 1, []), yl([1 1 2 2]), cB, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    end
    yyaxis right; plot(tF, g, '-', 'Color', cO); ylabel('Website foot-map glow'); hold on;
    for i = 1:size(web, 1)
        plot(tF(web(i,:)), max(g) * 1.05 * [1 1], '-', 'Color', cO, 'LineWidth', 4);
    end
    xlim([tF(1) tF(end)]); xlabel('Time (s)'); grid on;
    title('Blue shading = shoe on the ground (video). Orange bars = website showing contact (the screen lags a little).');
    figs(end+1) = struct('h', h3, 'file', [id '_chart_timeline.png'], 'title', 'Timeline');
end

function writeReport(file, Tres, S, figs, id, videoFile, t0, t1, fps)
    if exist(file, 'file'), try, delete(file); catch, end, end
    info = {'KYNETIX FSR insole: ground contact time check'; ''; ...
            ['Trial: ' id]; ['Video: ' videoFile]; sprintf('Analysed: %.1f s to %.1f s at %.0f fps', t0, t1, fps); ...
            ['Created: ' char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'))]; ''; 'SUMMARY'};
    writecell([info; S(:)], file, 'Sheet', 'Summary');
    writetable(Tres, file, 'Sheet', 'Stomps');
    writecell({'Charts'}, file, 'Sheet', 'Charts');
    if ~ispc, return; end
    try
        xl = actxserver('Excel.Application'); xl.Visible = false; xl.DisplayAlerts = false;
        wb = xl.Workbooks.Open(file);
        % Summary sheet
        ws = wb.Worksheets.Item('Summary');
        ws.Range('A1').Font.Size = 16; ws.Range('A1').Font.Bold = true;
        ws.Range('A8').Font.Bold = true; ws.Range('A8').Font.Size = 13;
        ws.Columns.Item(1).ColumnWidth = 120;
        % Stomps sheet
        ws = wb.Worksheets.Item('Stomps');
        nR = height(Tres) + 1; nC = width(Tres);
        last = colLetter(nC);
        hdr = ws.Range(['A1:' last '1']);
        hdr.Font.Bold = true; hdr.Font.Color = rgb2xl([1 1 1]); hdr.Interior.Color = rgb2xl([0.12 0.47 0.85]);
        ws.Range(['A1:' last num2str(nR)]).Borders.LineStyle = 1;
        agCol = find(strcmp(Tres.Properties.VariableNames, 'Agreement'));
        for r = 2:nR
            t = Tres.Agreement{r-1};
            if contains(t, '1 video frame'), c = [0.80 0.93 0.82];
            elseif contains(t, '2 video frames'), c = [1.00 0.93 0.75];
            elseif contains(t, 'more than'), c = [0.98 0.80 0.80];
            else, c = [0.93 0.93 0.93];
            end
            ws.Range([colLetter(agCol) num2str(r)]).Interior.Color = rgb2xl(c);
        end
        ws.Columns.AutoFit;
        % Charts sheet
        ws = wb.Worksheets.Item('Charts');
        ws.Range('A1').Font.Size = 16; ws.Range('A1').Font.Bold = true;
        top = 30;
        for q = 1:numel(figs)
            ws.Shapes.AddPicture(fullfile(pwd, figs(q).file), 0, -1, 10, top, -1, -1);
            pic = ws.Shapes.Item(ws.Shapes.Count);
            pic.LockAspectRatio = -1; pic.Width = 800;
            top = top + pic.Height + 25;
        end
        wb.Worksheets.Item('Summary').Activate;
        wb.Save; wb.Close(false); xl.Quit; delete(xl);
    catch err
        fprintf('Excel formatting/charts skipped (%s). The data is still in the .xlsx and the charts are saved as PNGs.\n', err.message);
        try, xl.Quit; delete(xl); catch, end
    end
end

function s = colLetter(n)
    s = '';
    while n > 0, r = mod(n - 1, 26); s = [char(65 + r) s]; n = floor((n - 1) / 26); end %#ok<AGROW>
end

function c = rgb2xl(rgb)
    rgb = round(rgb * 255); c = rgb(1) + 256 * rgb(2) + 65536 * rgb(3);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function x = fillMissing(x)
    ok = isfinite(x);
    if sum(ok) < 2, return; end
    i = (1:numel(x))';
    x(~ok) = interp1(i(ok), x(ok), i(~ok), 'linear', 'extrap');
end

function y = medfilt3pt(x)
    y = x;
    for k = 2:numel(x) - 1, y(k) = median(x(k-1:k+1)); end
end

function f = rotFrame(f, ROT)
    if ROT ~= 0, f = rot90(f, ROT); end
end

function G = toGray(f)
    f = double(f);
    G = 0.2989 * f(:,:,1) + 0.5870 * f(:,:,2) + 0.1140 * f(:,:,3);
end

function D = blockDown(G, k)
    h = floor(size(G,1) / k) * k; w = floor(size(G,2) / k) * k;
    G = G(1:h, 1:w);
    D = reshape(mean(mean(reshape(G, k, h/k, k, w/k), 1), 3), h/k, w/k);
end

function B = resizeBase(A, s)
    [h, w] = size(A);
    nh = max(4, round(h * s)); nw = max(4, round(w * s));
    [X, Y] = meshgrid(linspace(1, w, nw), linspace(1, h, nh));
    B = interp2(A, X, Y, 'linear');
end

function [best, r, c] = nccBest(Wn, T)
    T = T - mean(T(:)); nT = sqrt(sum(T(:).^2));
    [th, tw] = size(T);
    num = conv2(Wn, rot90(T, 2), 'valid');
    on = ones(th, tw);
    s1 = conv2(Wn, on, 'valid'); s2 = conv2(Wn.^2, on, 'valid');
    den = sqrt(max(s2 - s1.^2 / numel(T), 1e-9)) * nT;
    Rm = num ./ den;
    [best, idx] = max(Rm(:));
    [r, c] = ind2sub(size(Rm), idx);
end

function crop = cropRel(fr, box, cx, cy, sc, scrW, scrH)
    x0 = round(cx + (box(1) - scrW/2) * sc); x1 = round(cx + (box(3) - scrW/2) * sc);
    y0 = round(cy + (box(2) - scrH/2) * sc); y1 = round(cy + (box(4) - scrH/2) * sc);
    x0 = max(1, x0); y0 = max(1, y0); x1 = min(size(fr,2), x1); y1 = min(size(fr,1), y1);
    if x1 <= x0 || y1 <= y0, crop = []; return; end
    crop = fr(y0:y1, x0:x1, :);
end

function c = cropPad(fr, x0, y0, w, h)
    % crop of fixed size w x h starting at (x0, y0); outside the frame is black
    c = zeros(h, w, 3, 'uint8');
    xs = max(1, x0):min(size(fr,2), x0 + w - 1);
    ys = max(1, y0):min(size(fr,1), y0 + h - 1);
    if isempty(xs) || isempty(ys), return; end
    c(ys - y0 + 1, xs - x0 + 1, :) = fr(ys, xs, :);
end

function drawRel(trk, box, scrW, scrH, col)
    cx = trk(1); cy = trk(2); sc = trk(3);
    x0 = cx + (box(1) - scrW/2) * sc; x1 = cx + (box(3) - scrW/2) * sc;
    y0 = cy + (box(2) - scrH/2) * sc; y1 = cy + (box(4) - scrH/2) * sc;
    rectangle('Position', [x0 y0 x1-x0 y1-y0], 'EdgeColor', col, 'LineWidth', 1.5);
end
