module ultrasonic_sensor(
    input clk,              // 100 MHz clock
    input echo,             // Echo pulse from HC-SR04
    output reg trig,        // Trigger pulse to HC-SR04
    output reg [15:0] distance // Calculated distance in cm
);
    reg [19:0] trig_counter = 0;
    reg [19:0] echo_counter = 0;
    reg echo_old = 0;

    // Generate ~10us Trigger pulse every 60ms
    always @(posedge clk) begin
        if (trig_counter < 6000000) begin
            trig_counter <= trig_counter + 1;
            trig <= (trig_counter < 1000) ? 1'b1 : 1'b0; // 10us high
        end else begin
            trig_counter <= 0;
        end
    end

    // Measure Echo width
    always @(posedge clk) begin
        echo_old <= echo;
        if (echo) begin
            echo_counter <= echo_counter + 1;
        end else if (echo_old && !echo) begin
            // Distance = (Time * Speed of Sound) / 2
            // At 100MHz, 1 count = 10ns. Formula simplifies roughly to:
            distance <= echo_counter / 16'd5800; 
            echo_counter <= 0;
        end
    end
endmodule