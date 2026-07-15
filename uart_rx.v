`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/09/2026 04:27:35 PM
// Design Name: 
// Module Name: uart_rx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module uart_rx (
    input wire clk,       // 100 MHz system clock from Basys 3
    input wire rx,        // UART RX line from ESP32
    output reg [7:0] rx_data, // Received 8-bit data
    output reg rx_ready   // High for one clock cycle when data is valid and fully assembled
);

    // Fact: Clocks per bit = System Clock (100MHz) / Target Baud Rate (115200)
    // 100,000,000 / 115200 = 868.06 (Rounded to 868)
    parameter CLKS_PER_BIT = 868; // For 115200 baud with 100 MHz clock

    // State Machine definition using one-hot/binary encoding
    localparam IDLE         = 3'b000;
    localparam RX_START_BIT = 3'b001;
    localparam RX_DATA_BITS = 3'b010;
    localparam RX_STOP_BIT  = 3'b011;
    localparam CLEANUP      = 3'b100;

    reg [2:0] state = IDLE;
    reg [13:0] clock_count = 0; // 14-bit register is sufficient to hold up to 16383
    reg [2:0] bit_index = 0;    // 3-bit register to count from 0 to 7 (8 bits total)
    reg [7:0] data_buffer = 0;

    always @(posedge clk) begin
        case (state)
            IDLE: begin
                rx_ready <= 1'b0;
                clock_count <= 0;
                bit_index <= 0;
                
                if (rx == 1'b0) begin // Start bit detected (falling edge on RX line)
                    state <= RX_START_BIT;
                end else begin
                    state <= IDLE;
                end
            end

            RX_START_BIT: begin
                // Wait until the middle of the start bit to sample, preventing edge noise corruption
                if (clock_count == (CLKS_PER_BIT / 2)) begin
                    if (rx == 1'b0) begin // Verify it is still a valid start bit
                        clock_count <= 0;
                        state <= RX_DATA_BITS;
                    end else begin
                        state <= IDLE; // False alarm (glitch), return to IDLE
                    end
                end else begin
                    clock_count <= clock_count + 1;
                    state <= RX_START_BIT;
                end
            end

            RX_DATA_BITS: begin
                // Wait for the duration of one full bit
                if (clock_count < CLKS_PER_BIT - 1) begin
                    clock_count <= clock_count + 1;
                    state <= RX_DATA_BITS;
                end else begin
                    clock_count <= 0;
                    data_buffer[bit_index] <= rx; // Sample the incoming data bit
                    
                    if (bit_index < 7) begin
                        bit_index <= bit_index + 1;
                        state <= RX_DATA_BITS;
                    end else begin
                        bit_index <= 0;
                        state <= RX_STOP_BIT;
                    end
                end
            end

            RX_STOP_BIT: begin
                // Wait for the duration of the stop bit
                if (clock_count < CLKS_PER_BIT - 1) begin
                    clock_count <= clock_count + 1;
                    state <= RX_STOP_BIT;
                end else begin
                    rx_ready <= 1'b1;        // Trigger validation pulse
                    rx_data <= data_buffer;  // Push buffer to output register
                    clock_count <= 0;
                    state <= CLEANUP;
                end
            end

            CLEANUP: begin
                rx_ready <= 1'b0; // Pull validation pulse low after 1 clock cycle
                state <= IDLE;
            end

            default: begin
                state <= IDLE;
            end
        endcase
    end
endmodule
