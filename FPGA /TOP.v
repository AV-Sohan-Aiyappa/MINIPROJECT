// =============================================================================
// top_spectrum.v
// Hardware-Aware Spectrum Monitoring and Channel Occupancy Analyzer
// Basys3 (Artix-7 XC7A35T) @ 100 MHz
//
// IP dependency: xfft_512
//   Vivado IP Catalog -> Fast Fourier Transform
//   Component name    : xfft_512
//   Transform length  : 512
//   Architecture      : Pipelined, Streaming I/O
//   Data format       : Fixed Point
//   Input data width  : 16
//   Phase factor width: 16
//   Output ordering   : Natural Order
//   Throttle scheme   : Non Real Time
//   Scaling options   : Unscaled
//   Rounding          : Truncation
//   Data memory       : Block RAM
//   Twiddle memory    : Block RAM
//
// KEY RESOURCE FIX:
//   All large arrays carry (* ram_style = "block" *) so Vivado maps them
//   to BRAM instead of flip-flops.  The 9-channel sample buffer is flattened
//   from 2D [0:8][0:511] to 1D [0:4607] (9*512=4608) because Vivado cannot
//   infer BRAM from 2D arrays.
//
// BRAM budget used (Basys3 has 50 x 18Kb BRAMs = 900 Kb):
//   sbuf  : 4608 x 16b =  72 Kb  -> 4 BRAMs
//   hann  :  512 x 16b =   8 Kb  -> 1 BRAM
//   fifo  : 4096 x  8b =  32 Kb  -> 2 BRAMs
//   mag   :  256 x 16b =   4 Kb  -> 1 BRAM
//   Total :                116 Kb -> ~7 BRAMs  (well within 50)
// =============================================================================

// =============================================================================
// MODULE: top
// =============================================================================
module top #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire        clk,
    input  wire        rst_btn,
    input  wire        esp_rx_pin,
    output wire        uart_txd_in,
    output wire [15:0] led          // Diagnostic LEDs
);

// Synchronous reset - Active High (Basys3 btnC = 1 when pressed)
reg rst_r = 1'b0, rst = 1'b0;
always @(posedge clk) begin
    rst_r <= rst_btn;
    rst   <= rst_r;
end

// ── UART RX ───────────────────────────────────────────────────────────────────
wire [7:0] rx_data;
wire       rx_ready;

uart_rx #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(BAUD_RATE)) u_rx (
    .clk(clk), .rst(rst), .rx_pin(esp_rx_pin),
    .rx_data(rx_data), .rx_ready(rx_ready)
);

// ── TX FIFO 4096 x 8b -> BRAM ─────────────────────────────────────────────────
(* ram_style = "block" *) reg [7:0] fifo_mem [0:4095];
reg [11:0] fifo_wr = 12'd0;
reg [11:0] fifo_rd = 12'd0;
wire fifo_empty = (fifo_wr == fifo_rd);
wire fifo_full  = ((fifo_wr + 12'd1) == fifo_rd);

// ── UART TX ───────────────────────────────────────────────────────────────────
reg [7:0] tx_dat = 8'd0;
reg       tx_go  = 1'b0;
wire      tx_busy;

uart_tx #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(BAUD_RATE)) u_tx (
    .clk(clk), .rst(rst), .tx_start(tx_go), .tx_data(tx_dat),
    .tx_busy(tx_busy), .tx_pin(uart_txd_in)
);

// ── Frame Parser ──────────────────────────────────────────────────────────────
wire        s_valid, e_valid;
wire [3:0]  ch_idx;
wire        busy_out;
wire signed [15:0] rssi_out;
wire [31:0] freq_out;
wire [7:0]  pt_byte;
wire        pt_valid;

frame_parser u_parser (
    .clk(clk), .rst(rst),
    .rx_data(rx_data), .rx_ready(rx_ready),
    .s_valid(s_valid), .e_valid(e_valid),
    .ch_idx(ch_idx), .busy_out(busy_out),
    .rssi_out(rssi_out), .freq_out(freq_out),
    .pt_byte(pt_byte), .pt_valid(pt_valid)
);

// ── Occupancy Counter ─────────────────────────────────────────────────────────
wire        occ_valid;
wire [3:0]  occ_ch;
wire [6:0]  occ_pct;
wire [31:0] occ_freq;

occ_counter u_occ (
    .clk(clk), .rst(rst),
    .s_valid(s_valid), .ch_idx(ch_idx),
    .busy_in(busy_out), .freq_in(freq_out),
    .occ_valid(occ_valid), .occ_ch(occ_ch),
    .occ_pct(occ_pct), .occ_freq(occ_freq)
);

// ── FFT Feeder ────────────────────────────────────────────────────────────────
wire [31:0] fft_in_tdata;
wire        fft_in_tvalid, fft_in_tlast, fft_in_tready;
wire [7:0]  fft_cfg_tdata;
wire        fft_cfg_tvalid, fft_cfg_tready;
wire [31:0] freq_tag;
wire        freq_tag_valid;

fft_feeder u_feeder (
    .clk(clk), .rst(rst),
    .e_valid(e_valid), .rssi_in(rssi_out),
    .freq_in(freq_out), .ch_idx_in(ch_idx),
    .m_tdata(fft_in_tdata), .m_tvalid(fft_in_tvalid),
    .m_tlast(fft_in_tlast), .m_tready(fft_in_tready),
    .cfg_tdata(fft_cfg_tdata), .cfg_tvalid(fft_cfg_tvalid),
    .cfg_tready(fft_cfg_tready),
    .freq_tag_out(freq_tag), .freq_tag_valid(freq_tag_valid)
);

// ── Xilinx FFT IP ─────────────────────────────────────────────────────────────
wire [55:0] fft_out_tdata;
wire        fft_out_tvalid, fft_out_tlast;

xfft_512 u_xfft (
    .aclk                        (clk),
    .s_axis_config_tdata         (fft_cfg_tdata),
    .s_axis_config_tvalid        (fft_cfg_tvalid),
    .s_axis_config_tready        (fft_cfg_tready),
    .s_axis_data_tdata           (fft_in_tdata),
    .s_axis_data_tvalid          (fft_in_tvalid),
    .s_axis_data_tready          (fft_in_tready),
    .s_axis_data_tlast           (fft_in_tlast),
    .m_axis_data_tdata           (fft_out_tdata),
    .m_axis_data_tvalid          (fft_out_tvalid),
    .m_axis_data_tready          (1'b1),
    .m_axis_data_tlast           (fft_out_tlast),
    .event_frame_started         (),
    .event_tlast_unexpected      (),
    .event_tlast_missing         (),
    .event_status_channel_halt   (),
    .event_data_in_channel_halt  (),
    .event_data_out_channel_halt ()
);

// ── FFT Output Stage ──────────────────────────────────────────────────────────
wire [7:0] fft_fmt_byte;
wire       fft_fmt_valid;

fft_output_stage u_fft_out (
    .clk(clk), .rst(rst),
    .fft_tdata(fft_out_tdata),
    .fft_tvalid(fft_out_tvalid),
    .fft_tlast(fft_out_tlast),
    .freq_tag(freq_tag),
    .freq_tag_valid(freq_tag_valid),
    .out_byte(fft_fmt_byte),
    .out_valid(fft_fmt_valid)
);

// ── Occupancy Formatter ───────────────────────────────────────────────────────
wire [7:0] occ_fmt_byte;
wire       occ_fmt_valid;
wire       occ_fmt_busy;

tx_formatter_occ u_fmt_occ (
    .clk(clk), .rst(rst),
    .trigger(occ_valid), .freq_in(occ_freq), .pct_in(occ_pct),
    .out_byte(occ_fmt_byte), .out_valid(occ_fmt_valid), .out_busy(occ_fmt_busy)
);

// ── Heartbeat generator ───────────────────────────────────────────────────────
// Sends "H,ALIVE\n" (8 bytes) every ~1 second when FIFO has room.
// MATLAB ignores unknown frame types so this is safe.
// LED 6 toggles on every heartbeat sent - confirms FPGA->PC link is live.
wire [7:0] hb_byte;
wire       hb_valid;
reg        toggle_hb = 1'b0;

heartbeat u_hb (
    .clk(clk), .rst(rst),
    .out_byte(hb_byte), .out_valid(hb_valid)
);

// ── Single always block: owns fifo_wr, fifo_rd, tx_go, tx_dat ────────────────
always @(posedge clk) begin
    tx_go <= 1'b0;
    if (rst) begin
        fifo_wr <= 12'd0;
        fifo_rd <= 12'd0;
        tx_go   <= 1'b0;
    end else begin
        // Drain FIFO -> UART TX
        if (!fifo_empty && !tx_busy && !tx_go) begin
            tx_dat  <= fifo_mem[fifo_rd];
            fifo_rd <= fifo_rd + 12'd1;
            tx_go   <= 1'b1;
        end
        // Write to FIFO - strict priority, one writer per clock
        if (pt_valid && !fifo_full) begin
            fifo_mem[fifo_wr] <= pt_byte;
            fifo_wr           <= fifo_wr + 12'd1;
        end else if (occ_fmt_valid && !fifo_full) begin
            fifo_mem[fifo_wr] <= occ_fmt_byte;
            fifo_wr           <= fifo_wr + 12'd1;
        end else if (fft_fmt_valid && !fifo_full) begin
            fifo_mem[fifo_wr] <= fft_fmt_byte;
            fifo_wr           <= fifo_wr + 12'd1;
        end else if (hb_valid && !fifo_full) begin
            // Heartbeat has lowest priority - only writes when nothing else is
            fifo_mem[fifo_wr] <= hb_byte;
            fifo_wr           <= fifo_wr + 12'd1;
        end
    end
end

// ── Diagnostic LEDs ───────────────────────────────────────────────────────────
// LED 0  : esp_rx_pin raw  - ON=idle(HIGH), flickers during data
// LED 1  : toggle on every rx_ready - blinks when bytes decoded successfully
// LED 2  : toggle on every s_valid  - blinks when 'S' frames parsed
// LED 3  : toggle on every e_valid  - blinks when 'E' frames parsed
// LED 4  : toggle on every fft_out_tvalid - blinks when FFT producing output
// LED 5  : occ_valid pulse - blinks when occupancy frame sent
// LED 6  : toggle on heartbeat - blinks ~1Hz to confirm FPGA->PC link alive
// LED 15 : uart_txd_in raw - ON=idle, flickers when sending to PC

reg toggle_rx  = 1'b0;
reg toggle_pt  = 1'b0;
reg toggle_e   = 1'b0;
reg toggle_fft = 1'b0;
reg toggle_occ = 1'b0;

always @(posedge clk) begin
    if (rst) begin
        toggle_rx  <= 1'b0;
        toggle_pt  <= 1'b0;
        toggle_e   <= 1'b0;
        toggle_fft <= 1'b0;
        toggle_occ <= 1'b0;
        toggle_hb  <= 1'b0;
    end else begin
        if (rx_ready)        toggle_rx  <= ~toggle_rx;
        if (s_valid)         toggle_pt  <= ~toggle_pt;
        if (e_valid)         toggle_e   <= ~toggle_e;
        if (fft_out_tvalid)  toggle_fft <= ~toggle_fft;
        if (occ_valid)       toggle_occ <= ~toggle_occ;
        if (hb_valid)        toggle_hb  <= ~toggle_hb;
    end
end

assign led[0]  = esp_rx_pin;
assign led[1]  = toggle_rx;
assign led[2]  = toggle_pt;
assign led[3]  = toggle_e;
assign led[4]  = toggle_fft;
assign led[5]  = toggle_occ;
assign led[6]  = toggle_hb;
assign led[14:7] = 8'b0;
assign led[15] = uart_txd_in;

endmodule
//
// Fixes applied:
//   1. sync0/sync1 now reset to 1 (UART idle = HIGH) inside the same
//      always block as the state machine so reset clears everything atomically.
//   2. bit_idx and shreg now explicitly reset.
//   3. Stop-bit validity check: if stop bit is 0 (framing error / noise),
//      discard the byte and return to IDLE instead of reporting bad data.
//   4. START state re-checks that line is still LOW at mid-point; if it
//      went HIGH again it was a glitch - abort back to IDLE.
// =============================================================================
module uart_rx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx_pin,
    output reg  [7:0] rx_data,
    output reg        rx_ready
);
localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // 868
localparam HALF_BIT     = CLKS_PER_BIT / 2;       // 434
localparam ST_IDLE=2'd0, ST_START=2'd1, ST_DATA=2'd2, ST_STOP=2'd3;

reg [1:0]  state   = ST_IDLE;
reg [15:0] cnt     = 16'd0;
reg [2:0]  bit_idx = 3'd0;
reg [7:0]  shreg   = 8'd0;
// Two-flop synchroniser - kept in same always block so rst clears them
reg        sync0   = 1'b1;
reg        sync1   = 1'b1;

always @(posedge clk) begin
    rx_ready <= 1'b0;

    // Synchroniser - always runs
    sync0 <= rx_pin;
    sync1 <= sync0;

    if (rst) begin
        // Reset everything including synchroniser to UART idle (HIGH)
        state   <= ST_IDLE;
        cnt     <= 16'd0;
        bit_idx <= 3'd0;
        shreg   <= 8'd0;
        sync0   <= 1'b1;
        sync1   <= 1'b1;
    end else case (state)

        ST_IDLE: begin
            cnt     <= 16'd0;
            bit_idx <= 3'd0;
            // Start condition: line goes LOW
            if (!sync1) state <= ST_START;
        end

        ST_START: begin
            // Wait to mid-point of start bit
            if (cnt == HALF_BIT - 1) begin
                cnt <= 16'd0;
                // Validate: line must still be LOW at mid-point
                // If it went HIGH it was a glitch - go back to IDLE
                if (!sync1)
                    state <= ST_DATA;
                else
                    state <= ST_IDLE;
            end else begin
                cnt <= cnt + 16'd1;
            end
        end

        ST_DATA: begin
            if (cnt == CLKS_PER_BIT - 1) begin
                cnt <= 16'd0;
                shreg[bit_idx] <= sync1;
                if (bit_idx == 3'd7) begin
                    bit_idx <= 3'd0;
                    state   <= ST_STOP;
                end else begin
                    bit_idx <= bit_idx + 3'd1;
                end
            end else begin
                cnt <= cnt + 16'd1;
            end
        end

        ST_STOP: begin
            if (cnt == CLKS_PER_BIT - 1) begin
                cnt   <= 16'd0;
                state <= ST_IDLE;
                // Only accept byte if stop bit is HIGH (valid framing)
                // If stop bit is LOW it is a framing error - discard
                if (sync1) begin
                    rx_data  <= shreg;
                    rx_ready <= 1'b1;
                end
            end else begin
                cnt <= cnt + 16'd1;
            end
        end

        default: state <= ST_IDLE;

    endcase
end
endmodule


// =============================================================================
// MODULE: uart_tx  (unchanged from original)
// =============================================================================
module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_busy,
    output reg        tx_pin
);
localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
localparam ST_IDLE=2'd0, ST_START=2'd1, ST_DATA=2'd2, ST_STOP=2'd3;

reg [1:0]  state = ST_IDLE;
reg [15:0] cnt   = 16'd0;
reg [2:0]  bidx  = 3'd0;
reg [7:0]  shreg = 8'd0;

always @(posedge clk) begin
    if (rst) begin
        state<=ST_IDLE; tx_pin<=1'b1; tx_busy<=1'b0;
        cnt<=16'd0; bidx<=3'd0;
    end else case (state)
        ST_IDLE:  begin tx_pin<=1'b1; tx_busy<=1'b0; cnt<=16'd0;
                        if (tx_start) begin
                            shreg<=tx_data; tx_busy<=1'b1; state<=ST_START;
                        end end
        ST_START: begin tx_pin<=1'b0;
                        if (cnt==CLKS_PER_BIT-1) begin
                            cnt<=16'd0; bidx<=3'd0; state<=ST_DATA;
                        end else cnt<=cnt+16'd1; end
        ST_DATA:  begin tx_pin<=shreg[bidx];
                        if (cnt==CLKS_PER_BIT-1) begin cnt<=16'd0;
                            if (bidx==3'd7) state<=ST_STOP;
                            else bidx<=bidx+3'd1;
                        end else cnt<=cnt+16'd1; end
        ST_STOP:  begin tx_pin<=1'b1;
                        if (cnt==CLKS_PER_BIT-1) begin
                            cnt<=16'd0; state<=ST_IDLE;
                        end else cnt<=cnt+16'd1; end
    endcase
end
endmodule


// =============================================================================
// MODULE: frame_parser
// =============================================================================
module frame_parser (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  rx_data,
    input  wire        rx_ready,
    output reg         s_valid,
    output reg         e_valid,
    output reg  [3:0]  ch_idx,
    output reg         busy_out,
    output reg  signed [15:0] rssi_out,
    output reg  [31:0] freq_out,
    output reg  [7:0]  pt_byte,
    output reg         pt_valid
);

// Line buffer - plain registers (NOT distributed RAM) for reliable
// single-cycle read access inside the combinatorial parse loop.
// 64 x 8b = 512 bits = 32 LUTs, perfectly acceptable.
reg [7:0] lbuf [0:63];
reg [5:0] llen = 6'd0;
reg       is_s = 1'b0;

integer i;

always @(posedge clk) begin
    s_valid  <= 1'b0;
    e_valid  <= 1'b0;
    pt_valid <= 1'b0;

    if (rst) begin
        llen <= 6'd0;
        is_s <= 1'b0;
    end else if (rx_ready) begin

        if (llen == 6'd0)
            is_s <= (rx_data == 8'h53); // 'S'

        if (is_s || (llen == 6'd0 && rx_data == 8'h53)) begin
            pt_byte  <= rx_data;
            pt_valid <= 1'b1;
        end

        if (rx_data == 8'h0A) begin // '\n'
            begin : PARSE
                reg [7:0]  ftype;
                reg [31:0] p_freq;
                reg signed [15:0] p_rssi;
                reg        p_busy;
                reg [31:0] acc;
                reg        neg;
                reg [2:0]  fld;

                ftype  = lbuf[0];
                p_freq = 32'd0;
                p_rssi = 16'sd0;
                p_busy = 1'b0;
                acc    = 32'd0;
                neg    = 1'b0;
                fld    = 3'd0;

                for (i = 0; i < 64; i = i + 1) begin
                    if (i < llen) begin
                        if (lbuf[i] == 8'h2C) begin // ','
                            if (fld == 3'd2) p_freq = acc;
                            if (fld == 3'd3) p_rssi = neg ? -(acc[15:0]) : acc[15:0];
                            if (fld == 3'd4) p_busy = acc[0];
                            fld = fld + 3'd1;
                            acc = 32'd0;
                            neg = 1'b0;
                        end else if (lbuf[i] == 8'h2D) begin // '-'
                            neg = 1'b1;
                        end else if (lbuf[i] >= 8'h30 && lbuf[i] <= 8'h39) begin
                            acc = (acc * 32'd10) + ({24'b0,lbuf[i]} - 32'd48);
                        end
                    end
                end
                // Commit last field
                if (fld == 3'd2) p_freq = acc;
                if (fld == 3'd3) p_rssi = neg ? -(acc[15:0]) : acc[15:0];
                if (fld == 3'd4) p_busy = acc[0];

                freq_out <= p_freq;
                rssi_out <= p_rssi;
                busy_out <= p_busy;

                case (p_freq - 32'd865_100_000)
                    32'd0:           ch_idx <= 4'd0;
                    32'd200_000:     ch_idx <= 4'd1;
                    32'd400_000:     ch_idx <= 4'd2;
                    32'd600_000:     ch_idx <= 4'd3;
                    32'd800_000:     ch_idx <= 4'd4;
                    32'd1_000_000:   ch_idx <= 4'd5;
                    32'd1_200_000:   ch_idx <= 4'd6;
                    32'd1_400_000:   ch_idx <= 4'd7;
                    32'd1_600_000:   ch_idx <= 4'd8;
                    default:         ch_idx <= 4'd0;
                endcase

                if (ftype == 8'h53) s_valid <= 1'b1; // 'S'
                if (ftype == 8'h45) e_valid <= 1'b1; // 'E'
            end
            llen <= 6'd0;
            is_s <= 1'b0;
        end else begin
            if (llen < 6'd63) begin
                lbuf[llen] <= rx_data;
                llen       <= llen + 6'd1;
            end
        end
    end
end
endmodule


// =============================================================================
// MODULE: occ_counter
// =============================================================================
module occ_counter (
    input  wire        clk,
    input  wire        rst,
    input  wire        s_valid,
    input  wire [3:0]  ch_idx,
    input  wire        busy_in,
    input  wire [31:0] freq_in,
    output reg         occ_valid,
    output reg  [3:0]  occ_ch,
    output reg  [6:0]  occ_pct,
    output reg  [31:0] occ_freq
);

// Small arrays - distributed RAM is fine (9 entries)
reg [7:0]  busy_cnt  [0:8];
reg [7:0]  total_cnt [0:8];
reg [31:0] ch_freq   [0:8];

integer i;
initial begin
    for (i = 0; i < 9; i = i + 1) begin
        busy_cnt[i]  = 8'd0;
        total_cnt[i] = 8'd0;
        ch_freq[i]   = 32'd0;
    end
end

always @(posedge clk) begin
    occ_valid <= 1'b0;
    if (!rst && s_valid && ch_idx <= 4'd8) begin
        ch_freq[ch_idx] <= freq_in;
        if (busy_in)
            busy_cnt[ch_idx] <= busy_cnt[ch_idx] + 8'd1;
        total_cnt[ch_idx] <= total_cnt[ch_idx] + 8'd1;
        if (total_cnt[ch_idx] == 8'd199) begin
            occ_pct   <= {1'b0, busy_cnt[ch_idx][7:1]};
            occ_ch    <= ch_idx;
            occ_freq  <= ch_freq[ch_idx];
            occ_valid <= 1'b1;
            busy_cnt[ch_idx]  <= 8'd0;
            total_cnt[ch_idx] <= 8'd0;
        end
    end
end
endmodule


// =============================================================================
// MODULE: fft_feeder
//
// BRAM FIX: sbuf flattened from [0:8][0:511] to [0:4607] (9*512=4608 entries)
//           Index = ch_idx * 512 + sample_pos
//           Both sbuf and hann carry (* ram_style = "block" *)
// =============================================================================
module fft_feeder (
    input  wire        clk,
    input  wire        rst,
    input  wire        e_valid,
    input  wire signed [15:0] rssi_in,
    input  wire [31:0] freq_in,
    input  wire [3:0]  ch_idx_in,
    output reg  [31:0] m_tdata,
    output reg         m_tvalid,
    output reg         m_tlast,
    input  wire        m_tready,
    output reg  [7:0]  cfg_tdata,
    output reg         cfg_tvalid,
    input  wire        cfg_tready,
    output reg  [31:0] freq_tag_out,
    output reg         freq_tag_valid
);

// Flattened 9-channel x 512-sample buffer -> BRAM
// Access: sbuf[ch * 512 + sample_idx]
(* ram_style = "block" *) reg signed [15:0] sbuf [0:4607];

// Hann window ROM -> BRAM
(* ram_style = "block" *) reg [15:0] hann [0:511];

// Per-channel metadata (small, stays as registers)
reg [8:0]  scount [0:8];   // fill count per channel
reg [31:0] sfreq  [0:8];   // last seen frequency per channel

integer k;
initial begin
    for (k = 0; k < 512; k = k + 1)
        hann[k] = $rtoi(32767.0 * 0.5 *
                  (1.0 - $cos(2.0 * 3.14159265358979 * k / 512.0)));
    for (k = 0; k < 9; k = k + 1) begin
        scount[k] = 9'd0;
        sfreq[k]  = 32'd0;
    end
end

localparam ST_IDLE   = 2'd0,
           ST_CONFIG = 2'd1,
           ST_SEND   = 2'd2;

reg [1:0]  state     = ST_IDLE;
reg [3:0]  active_ch = 4'd0;
reg [8:0]  send_idx  = 9'd0;
reg        cfg_sent  = 1'b0;

// BRAM read address for sbuf and hann (registered = 1-cycle latency)
reg [12:0] sbuf_raddr = 13'd0;   // up to 4607
reg [8:0]  hann_raddr = 9'd0;    // up to 511
wire signed [15:0] sbuf_rdata = sbuf[sbuf_raddr];
wire        [15:0] hann_rdata = hann[hann_raddr];

// Windowed multiply (registered output, accounts for BRAM read latency)
reg signed [31:0] windowed_r  = 32'sd0;
reg signed [31:0] windowed_r2 = 32'sd0;

always @(posedge clk) begin
    // Pipeline stage 1: issue BRAM read addresses
    sbuf_raddr <= {active_ch, send_idx};   // ch*512 + send_idx (concatenation)
    hann_raddr <= send_idx[8:0];
    // Pipeline stage 2: multiply (data arrives from BRAM)
    windowed_r  <= ($signed({1'b0, hann_rdata}) * sbuf_rdata) >>> 15;
    // Pipeline stage 3: register once more so AXI output is stable
    windowed_r2 <= windowed_r;
end

// Write address for sbuf
wire [12:0] sbuf_waddr = {ch_idx_in[3:0], scount[ch_idx_in][8:0]};

always @(posedge clk) begin
    m_tvalid       <= 1'b0;
    m_tlast        <= 1'b0;
    cfg_tvalid     <= 1'b0;
    freq_tag_valid <= 1'b0;

    if (rst) begin
        state    <= ST_IDLE;
        cfg_sent <= 1'b0;
        for (k = 0; k < 9; k = k + 1)
            scount[k] <= 9'd0;
    end else begin

        // Always accumulate incoming E samples into BRAM
        if (e_valid && ch_idx_in <= 4'd8) begin
            sbuf[sbuf_waddr]  <= rssi_in;
            sfreq[ch_idx_in]  <= freq_in;
            if (scount[ch_idx_in] == 9'd511)
                scount[ch_idx_in] <= 9'd0;
            else
                scount[ch_idx_in] <= scount[ch_idx_in] + 9'd1;
        end

        case (state)

            ST_IDLE: begin
                // Detect any channel just completing its 512th sample
                begin : FILL_CHECK
                    integer j;
                    for (j = 0; j < 9; j = j + 1) begin
                        if (e_valid && ch_idx_in == j[3:0] &&
                            scount[j] == 9'd511) begin
                            active_ch <= j[3:0];
                            send_idx  <= 9'd0;
                            state     <= cfg_sent ? ST_SEND : ST_CONFIG;
                        end
                    end
                end
            end

            ST_CONFIG: begin
                cfg_tdata  <= 8'h00;   // bit0=0 -> FFT forward
                cfg_tvalid <= 1'b1;
                if (cfg_tready) begin
                    cfg_sent <= 1'b1;
                    state    <= ST_SEND;
                end
            end

            ST_SEND: begin
                if (m_tready) begin
                    // Output windowed sample (2-cycle BRAM+multiply pipeline)
                    m_tdata  <= {16'b0, windowed_r2[15:0]};
                    m_tvalid <= 1'b1;
                    m_tlast  <= (send_idx == 9'd511);
                    if (send_idx == 9'd511) begin
                        freq_tag_out   <= sfreq[active_ch];
                        freq_tag_valid <= 1'b1;
                        state          <= ST_IDLE;
                    end else begin
                        send_idx <= send_idx + 9'd1;
                    end
                end
            end

            default: state <= ST_IDLE;

        endcase
    end
end
endmodule


// =============================================================================
// MODULE: fft_output_stage
//
// BRAM FIX: mag_buf carries (* ram_style = "block" *)
// IP output: tdata[55:0] = {im[27:0], re[27:0]}
// =============================================================================
module fft_output_stage (
    input  wire        clk,
    input  wire        rst,
    input  wire [55:0] fft_tdata,
    input  wire        fft_tvalid,
    input  wire        fft_tlast,
    input  wire [31:0] freq_tag,
    input  wire        freq_tag_valid,
    output reg  [7:0]  out_byte,
    output reg         out_valid
);

// Magnitude buffer -> BRAM
(* ram_style = "block" *) reg [15:0] mag_buf [0:255];
reg [8:0]  bin_cnt   = 9'd0;
reg [31:0] freq_lat  = 32'd0;
reg        frame_rdy = 1'b0;

always @(posedge clk) begin
    if (freq_tag_valid) freq_lat <= freq_tag;
end

always @(posedge clk) begin
    frame_rdy <= 1'b0;
    if (rst) begin
        bin_cnt <= 9'd0;
    end else if (fft_tvalid) begin
        if (bin_cnt < 9'd256) begin
            begin : MAG
                reg signed [27:0] re28, im28;
                reg [27:0] ar, ai, mx, mn, mg;
                re28 = fft_tdata[27:0];
                im28 = fft_tdata[55:28];
                ar   = re28[27] ? (~re28 + 28'd1) : {1'b0, re28[26:0]};
                ai   = im28[27] ? (~im28 + 28'd1) : {1'b0, im28[26:0]};
                mx   = (ar > ai) ? ar : ai;
                mn   = (ar > ai) ? ai : ar;
                mg   = mx + ((mn * 28'd3) >> 3);
                mag_buf[bin_cnt[7:0]] <= mg[27:12];
            end
        end
        if (fft_tlast) begin
            frame_rdy <= 1'b1;
            bin_cnt   <= 9'd0;
        end else begin
            bin_cnt <= bin_cnt + 9'd1;
        end
    end
end

// ── Serialiser FSM ────────────────────────────────────────────────────────────
localparam FS_IDLE   = 3'd0,
           FS_F      = 3'd1,
           FS_C1     = 3'd2,
           FS_FREQ   = 3'd3,
           FS_BCOMMA = 3'd4,
           FS_DIGIT  = 3'd5,
           FS_NL     = 3'd6;

reg [2:0] fs      = FS_IDLE;
reg [7:0] bidx    = 8'd0;
reg [2:0] didx    = 3'd0;
reg [3:0] fpos    = 4'd8;
reg [3:0] fd [0:8];
reg [3:0] md [0:4];

// BRAM read port for mag_buf (1-cycle latency)
reg  [7:0]  mag_raddr = 8'd0;
wire [15:0] mag_rdata = mag_buf[mag_raddr];
reg  [15:0] mag_lat2  = 16'd0;

always @(posedge clk) begin
    mag_lat2 <= mag_rdata;
end

always @(posedge clk) begin
    out_valid <= 1'b0;
    if (rst) begin
        fs <= FS_IDLE;
    end else case (fs)

        FS_IDLE: begin
            if (frame_rdy) begin
                fd[8] <= (freq_lat / 32'd100_000_000) % 10;
                fd[7] <= (freq_lat / 32'd10_000_000)  % 10;
                fd[6] <= (freq_lat / 32'd1_000_000)   % 10;
                fd[5] <= (freq_lat / 32'd100_000)     % 10;
                fd[4] <= (freq_lat / 32'd10_000)      % 10;
                fd[3] <= (freq_lat / 32'd1_000)       % 10;
                fd[2] <= (freq_lat / 32'd100)         % 10;
                fd[1] <= (freq_lat / 32'd10)          % 10;
                fd[0] <= (freq_lat / 32'd1)           % 10;
                fpos      <= 4'd8;
                bidx      <= 8'd0;
                mag_raddr <= 8'd0;   // preload BRAM read for bin 0
                fs        <= FS_F;
            end
        end

        FS_F: begin
            out_byte <= 8'h46; out_valid <= 1'b1; fs <= FS_C1;
        end

        FS_C1: begin
            out_byte <= 8'h2C; out_valid <= 1'b1; fs <= FS_FREQ;
        end

        FS_FREQ: begin
            out_byte  <= 8'h30 + fd[fpos];
            out_valid <= 1'b1;
            if (fpos == 4'd0) begin
                // mag_buf[0] already being read (1 cycle latency)
                // Capture it next state via mag_lat2
                fs <= FS_BCOMMA;
            end else begin
                fpos <= fpos - 4'd1;
            end
        end

        FS_BCOMMA: begin
            // Capture mag value from BRAM (registered 1 cycle ago)
            md[4] <= (mag_lat2 / 16'd10_000) % 10;
            md[3] <= (mag_lat2 / 16'd1_000)  % 10;
            md[2] <= (mag_lat2 / 16'd100)    % 10;
            md[1] <= (mag_lat2 / 16'd10)     % 10;
            md[0] <= (mag_lat2)              % 10;
            didx  <= 3'd4;
            out_byte  <= 8'h2C;
            out_valid <= 1'b1;
            fs        <= FS_DIGIT;
        end

        FS_DIGIT: begin
            out_byte  <= 8'h30 + md[didx];
            out_valid <= 1'b1;
            if (didx == 3'd0) begin
                if (bidx == 8'd255) begin
                    fs <= FS_NL;
                end else begin
                    begin : NXT
                        reg [7:0] nb;
                        nb        = bidx + 8'd1;
                        bidx      <= nb;
                        mag_raddr <= nb;   // issue BRAM read for next bin
                    end
                    fs <= FS_BCOMMA;
                end
            end else begin
                didx <= didx - 3'd1;
            end
        end

        FS_NL: begin
            out_byte <= 8'h0A; out_valid <= 1'b1; fs <= FS_IDLE;
        end

        default: fs <= FS_IDLE;

    endcase
end
endmodule


// =============================================================================
// MODULE: tx_formatter_occ
// Builds "O,FFFFFFFFF,PPP\n" - 16 bytes fixed, zero-padded.
// =============================================================================
module tx_formatter_occ (
    input  wire        clk,
    input  wire        rst,
    input  wire        trigger,
    input  wire [31:0] freq_in,
    input  wire [6:0]  pct_in,
    output reg  [7:0]  out_byte,
    output reg         out_valid,
    output reg         out_busy
);

reg [7:0] frame [0:15];
reg [3:0] fidx    = 4'd0;
reg       sending = 1'b0;

wire [3:0] fd0 = (freq_in / 32'd1)           % 10;
wire [3:0] fd1 = (freq_in / 32'd10)          % 10;
wire [3:0] fd2 = (freq_in / 32'd100)         % 10;
wire [3:0] fd3 = (freq_in / 32'd1_000)       % 10;
wire [3:0] fd4 = (freq_in / 32'd10_000)      % 10;
wire [3:0] fd5 = (freq_in / 32'd100_000)     % 10;
wire [3:0] fd6 = (freq_in / 32'd1_000_000)   % 10;
wire [3:0] fd7 = (freq_in / 32'd10_000_000)  % 10;
wire [3:0] fd8 = (freq_in / 32'd100_000_000) % 10;
wire [3:0] pd0 = (pct_in  / 7'd1)            % 10;
wire [3:0] pd1 = (pct_in  / 7'd10)           % 10;
wire [3:0] pd2 = (pct_in  / 7'd100)          % 10;

always @(posedge clk) begin
    out_valid <= 1'b0;
    if (rst) begin
        sending <= 1'b0; fidx <= 4'd0; out_busy <= 1'b0;
    end else begin
        if (trigger && !sending) begin
            frame[0]  <= 8'h4F;
            frame[1]  <= 8'h2C;
            frame[2]  <= 8'h30 + fd8;
            frame[3]  <= 8'h30 + fd7;
            frame[4]  <= 8'h30 + fd6;
            frame[5]  <= 8'h30 + fd5;
            frame[6]  <= 8'h30 + fd4;
            frame[7]  <= 8'h30 + fd3;
            frame[8]  <= 8'h30 + fd2;
            frame[9]  <= 8'h30 + fd1;
            frame[10] <= 8'h30 + fd0;
            frame[11] <= 8'h2C;
            frame[12] <= 8'h30 + pd2;
            frame[13] <= 8'h30 + pd1;
            frame[14] <= 8'h30 + pd0;
            frame[15] <= 8'h0A;
            fidx     <= 4'd0;
            sending  <= 1'b1;
            out_busy <= 1'b1;
        end
        if (sending) begin
            out_byte  <= frame[fidx];
            out_valid <= 1'b1;
            if (fidx == 4'd15) begin
                sending  <= 1'b0;
                out_busy <= 1'b0;
                fidx     <= 4'd0;
            end else begin
                fidx <= fidx + 4'd1;
            end
        end
    end
end
endmodule

// =============================================================================
// MODULE: heartbeat
//
// Sends "H,ALIVE\n" (8 bytes) once every ~1 second (100MHz / 100_000_000).
// Frame type 'H' is unknown to MATLAB parser - silently ignored.
// out_valid pulses for exactly 8 consecutive clocks (one byte each).
// =============================================================================
module heartbeat (
    input  wire       clk,
    input  wire       rst,
    output reg  [7:0] out_byte,
    output reg        out_valid
);

// 1-second counter at 100 MHz
localparam PERIOD = 32'd100_000_000;

// "H,ALIVE\n" = 0x48 0x2C 0x41 0x4C 0x49 0x56 0x45 0x0A
localparam [63:0] MSG = 64'h480_02C_041_04C_049_056_045_00A;
// Store as individual bytes in a small array
reg [7:0] msg_rom [0:7];
initial begin
    msg_rom[0] = 8'h48; // 'H'
    msg_rom[1] = 8'h2C; // ','
    msg_rom[2] = 8'h41; // 'A'
    msg_rom[3] = 8'h4C; // 'L'
    msg_rom[4] = 8'h49; // 'I'
    msg_rom[5] = 8'h56; // 'V'
    msg_rom[6] = 8'h45; // 'E'
    msg_rom[7] = 8'h0A; // '\n'
end

reg [31:0] timer   = 32'd0;
reg [2:0]  byte_idx = 3'd0;
reg        sending  = 1'b0;

always @(posedge clk) begin
    out_valid <= 1'b0;

    if (rst) begin
        timer    <= 32'd0;
        byte_idx <= 3'd0;
        sending  <= 1'b0;
    end else begin
        if (sending) begin
            out_byte  <= msg_rom[byte_idx];
            out_valid <= 1'b1;
            if (byte_idx == 3'd7) begin
                sending  <= 1'b0;
                byte_idx <= 3'd0;
            end else begin
                byte_idx <= byte_idx + 3'd1;
            end
        end else begin
            if (timer == PERIOD - 1) begin
                timer   <= 32'd0;
                sending <= 1'b1;
            end else begin
                timer <= timer + 32'd1;
            end
        end
    end
end
endmodule
