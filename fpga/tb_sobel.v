`timescale 1ns/1ps

module tb_sobel;

    // ============================================================
    // PARAMETERS
    // ============================================================
    parameter WIDTH  = 512;
    parameter HEIGHT = 512;

    // ============================================================
    // SIGNALS
    // ============================================================
    reg CLOCK_50;
    reg rst_n;
    reg [31:0] pixel_in;
    reg        data_valid;
    wire [7:0] pixel_out;

    // Image memories
    reg [7:0] image_mem  [0:WIDTH*HEIGHT-1];
    reg [7:0] output_mem [0:WIDTH*HEIGHT-1];

    integer row, col;
    integer file_check;

    // ============================================================
    // DUT - 2-reg synchronizer version
    // ============================================================
    edge_vision_sobel DUT (
        .CLOCK_50   (CLOCK_50),
        .rst_n      (rst_n),
        .pixel_in   (pixel_in),
        .data_valid (data_valid),
        .pixel_out  (pixel_out)
    );

    // ============================================================
    // CLOCK GENERATION (50 MHz -> 20ns period)
    // ============================================================
    always #10 CLOCK_50 = ~CLOCK_50;

    // ============================================================
    // TASK: send one column for 2-reg synchronizer
    //
    // 2-reg rising edge = dv_ff1 & ~dv_ff2
    //
    // Cycle T+0: data_valid=1 asserted
    //            dv_ff1 latches data_valid (was 0) -> dv_ff1 still 0 this cycle
    // Cycle T+1: dv_ff1=1, dv_ff2=0 -> data_valid_rise FIRES
    //            buffer shifts, Sobel computed (non-blocking, takes effect next)
    // Cycle T+2: pixel_out_reg updated and stable -> safe to read pixel_out
    //
    // So we need 3 clocks total before reading pixel_out
    // ============================================================
    task send_column;
        input [31:0] col_data;
        begin
            pixel_in   = col_data;
            data_valid = 1;
            @(posedge CLOCK_50); #1;   // clock T:   dv_ff1 will latch 1
            @(posedge CLOCK_50); #1;   // clock T+1: dv_ff1=1, dv_ff2=0 -> rise fires
            data_valid = 0;
            @(posedge CLOCK_50); #1;   // clock T+2: pixel_out_reg settled -> read
        end
    endtask

    // ============================================================
    // MAIN
    // ============================================================
    initial begin

        CLOCK_50   = 0;
        rst_n      = 0;
        pixel_in   = 32'd0;
        data_valid = 0;

        for (row = 0; row < WIDTH*HEIGHT; row = row + 1)
            output_mem[row] = 0;

        // Check input file exists
        file_check = $fopen("sat_input.hex", "r");
        if (file_check == 0) begin
            $display("ERROR: sat_input.hex not found!");
            $stop;
        end
        $fclose(file_check);

        // Load image
        $readmemh("Physical_18.hex", image_mem);
        $display("First pixel     = %h (should be 25)", image_mem[0]);
        $display("Pixel[100*512+100] = %h (should be 29)", image_mem[100*WIDTH+100]);

        // Reset
        #100;
        rst_n = 1;
        repeat(5) @(posedge CLOCK_50);

        // ============================================================
        // STREAM IMAGE
        // Border pixels (row 0, row 511, col 0, col 511) stay 0
        // ============================================================
        for (row = 1; row < HEIGHT-1; row = row + 1) begin
            for (col = 1; col < WIDTH-1; col = col + 1) begin

                send_column({
                    8'd0,
                    image_mem[(row-1)*WIDTH + col],   // top    pixel
                    image_mem[(row  )*WIDTH + col],   // centre pixel
                    image_mem[(row+1)*WIDTH + col]    // bottom pixel
                });

                output_mem[row*WIDTH + col] = pixel_out;
            end
        end

        // ============================================================
        // PRINT ROW 100 FOR COMPARISON (cols 1-19)
        // ============================================================
        $display("\nRow 100 output (cols 1-19):");
        for (col = 1; col < 20; col = col + 1)
            $write(" %3d", output_mem[100*WIDTH + col]);
        $display("");

        // Save output
        $writememh("2flop_Physical_18.hex", output_mem);
        $display("Done. Output written to 2flop_Physical_18.hex");

        #100;
        $stop;
    end

    // ============================================================
    // WAVEFORM DUMP
    // ============================================================
    initial begin
        $dumpfile("vision_2reg.vcd");
        $dumpvars(0, tb_sobel);
    end

endmodule
