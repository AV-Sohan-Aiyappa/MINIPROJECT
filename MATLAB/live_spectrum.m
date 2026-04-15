clc
clear
close all

%% ============================================================
% SERIAL CONNECTIONS — TWO PORTS
% ============================================================

heltec_port = "COM7";   % <<< Heltec USB COM port (S/E frames)
fpga_port   = "COM9";   % <<< FPGA USB COM port   (F/O/H frames)
baud        = 115200;

disp("Connecting to Heltec...")
s_heltec = serialport(heltec_port, baud);
configureTerminator(s_heltec, "LF");
flush(s_heltec);

disp("Connecting to FPGA...")
s_fpga = serialport(fpga_port, baud);
configureTerminator(s_fpga, "LF");
flush(s_fpga);

disp("Both serial ports connected")
pause(2)

%% ============================================================
% LIVE DATA BUFFERS
% ============================================================

rssi_buffer = [];
busy_buffer = [];
occ_buffer  = [];
freq_buffer = [];
fft_bins    = zeros(256,1);
fft_ready   = false;

FFT_WINDOW = 512;
Fs_plot    = 5000;

%% ============================================================
% CREATE UI
% ============================================================

disp("LIVE REAL-TIME MODE STARTED")

SPEC_ROWS = 256;
SPEC_COLS = 200;
specMatrix = -150 * ones(SPEC_ROWS, SPEC_COLS);

figure('Name','Live Spectrum Sensing','NumberTitle','off')
tiledlayout(3,2,'TileSpacing','compact')

ax1 = nexttile;
specImg = imagesc(ax1, specMatrix);
set(ax1,'YDir','normal')
yticks(ax1, linspace(1,SPEC_ROWS,5))
yticklabels(ax1, round(linspace(-Fs_plot/2, Fs_plot/2, 5)))
title(ax1,'Received Spectrogram (FPGA FFT)')
xlabel(ax1,'Time')
ylabel(ax1,'Baseband Frequency (Hz)')
colormap(ax1,jet); colorbar(ax1)

ax2 = nexttile;
specLine = plot(ax2, nan, nan, 'LineWidth', 1.3);
title(ax2,'Received Spectrum (FPGA FFT)')
xlabel(ax2,'Baseband Offset (Hz)')
ylabel(ax2,'Power (dB)')
grid(ax2,'on')

ax3 = nexttile;
rssiLine = plot(ax3, nan, nan);
title(ax3,'Average RSSI (dB)')
xlabel(ax3,'Samples')
ylabel(ax3,'Power (dB)')
grid(ax3,'on')

ax4 = nexttile;
occLine = plot(ax4, nan, nan, 'LineWidth', 1.3);
title(ax4,'Channel Occupancy % (FPGA)')
xlabel(ax4,'Samples')
ylabel(ax4,'Occupancy (%)')
ylim(ax4,[0 100])
grid(ax4,'on')

ax5 = nexttile;
busyLine = plot(ax5, nan, nan);
title(ax5,'Busy / Free Detection')
xlabel(ax5,'Samples')
ylabel(ax5,'State')
ylim(ax5,[-0.2 1.2])
grid(ax5,'on')

ax6 = nexttile;
scanScatter = scatter(ax6, nan, nan, 20, 'filled');
title(ax6,'Channel Scan Power')
xlabel(ax6,'Frequency (MHz)')
ylabel(ax6,'RSSI (dBm)')
xlim(ax6,[865 867])
grid(ax6,'on')

%% ============================================================
% MAIN REAL-TIME LOOP
% ============================================================

while true

    %% -------- READ HELTEC PORT (S frames) --------
    while s_heltec.NumBytesAvailable > 0
        line  = readline(s_heltec);
        parts = split(strtrim(line), ',');
        if isempty(parts), continue; end
        type = parts{1};

        try
            if type == "S" && numel(parts) >= 5
                freq_val = str2double(parts{3});
                rssi_val = str2double(parts{4});
                busy_val = str2double(parts{5});
                rssi_buffer(end+1) = rssi_val;
                busy_buffer(end+1) = busy_val;
                freq_buffer(end+1) = freq_val;
            end
        catch
        end
    end

    %% -------- READ FPGA PORT (F / O / H frames) --------
    while s_fpga.NumBytesAvailable > 0
        line  = readline(s_fpga);
        parts = split(strtrim(line), ',');
        if isempty(parts), continue; end
        type = parts{1};

        try
            if type == "F" && numel(parts) >= 258
                % F,frequency,bin0,bin1,...,bin255
                raw_bins = str2double(parts(3:258));
                fft_bins = raw_bins(:);
                fft_ready = true;

            elseif type == "O" && numel(parts) >= 3
                % O,frequency,percent
                occ_val = str2double(parts{3});
                occ_buffer(end+1) = occ_val;   % already 0-100

            elseif type == "H"
                % Heartbeat — ignore silently
            end
        catch
        end
    end

    pause(0.001)

    %% -------- LIMIT MEMORY --------
    rssi_buffer = rssi_buffer(max(1,end-2000):end);
    busy_buffer = busy_buffer(max(1,end-2000):end);
    occ_buffer  = occ_buffer(max(1,end-2000):end);
    freq_buffer = freq_buffer(max(1,end-2000):end);

    if ~fft_ready
        continue
    end

    %% ========================================================
    % GRAPHICS UPDATE
    % ========================================================

    % ---- Spectrum (FPGA FFT bins) ----
    f = linspace(-Fs_plot/2, Fs_plot/2, 256);
    spec_db = 20*log10(double(fft_bins) + eps);
    set(specLine, 'XData', f, 'YData', spec_db);

    % ---- Waterfall ----
    col = spec_db;
    specMatrix(:, 1:end-1) = specMatrix(:, 2:end);
    specMatrix(:, end)     = col(1:SPEC_ROWS);
    set(specImg, 'CData', specMatrix);

    % ---- RSSI ----
    if ~isempty(rssi_buffer)
        set(rssiLine, ...
            'XData', 1:length(rssi_buffer), ...
            'YData', rssi_buffer);
    end

    % ---- Occupancy ----
    if ~isempty(occ_buffer)
        set(occLine, ...
            'XData', 1:length(occ_buffer), ...
            'YData', occ_buffer);
    end

    % ---- Busy/Free ----
    if ~isempty(busy_buffer)
        set(busyLine, ...
            'XData', 1:length(busy_buffer), ...
            'YData', busy_buffer);
    end

    % ---- Channel Scan ----
    if length(freq_buffer) > 30
        N      = min(200, length(freq_buffer));
        freqs  = freq_buffer(end-N+1:end) / 1e6;
        powers = rssi_buffer(end-N+1:end);
        set(scanScatter, ...
            'XData', freqs, ...
            'YData', powers, ...
            'CData', powers);
    end

    drawnow limitrate
end
