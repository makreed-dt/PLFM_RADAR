module cic_decimator_4x_enhanced (
    input wire clk,                 // 400MHz input clock
    input wire reset_n,
    input wire signed [17:0] data_in,  // 18-bit input
    input wire data_valid,
    output reg signed [17:0] data_out, // 18-bit output
    output reg data_out_valid,       // Valid at 100MHz
    // Enhanced monitoring outputs
    output reg saturation_detected,  // Latched saturation indicator
    output reg [7:0] max_value_monitor, // For gain control
    input wire reset_monitors        // Clear saturation and max value
);

parameter STAGES = 5;
parameter DECIMATION = 4;
parameter COMB_DELAY = 1;

// Accumulator width: input_width + N*log2(R) = 18 + 5*2 = 28 bits
// (36-bit was over-provisioned; 28 bits is mathematically exact for R=4, N=5)
localparam ACC_WIDTH = 28;

reg signed [ACC_WIDTH-1:0] integrator [0:STAGES-1];
reg signed [ACC_WIDTH-1:0] comb [0:STAGES-1];
reg signed [ACC_WIDTH-1:0] comb_delay [0:STAGES-1][0:COMB_DELAY-1];

// Enhanced control and monitoring
reg [1:0] decimation_counter;
reg data_valid_delayed;
reg data_valid_comb;
reg [7:0] output_counter;
reg [ACC_WIDTH-1:0] max_integrator_value;
reg overflow_detected;
reg overflow_latched;  // Latched overflow indicator

// Diagnostic registers
reg [7:0] saturation_event_count;
reg [31:0] sample_count;

// Comb-stage saturation flags (separate from integrator block to avoid multi-driven nets)
reg comb_overflow_latched;
reg comb_saturation_detected;
reg [7:0] comb_saturation_event_count;

// Temporary signals for calculations
reg signed [ACC_WIDTH-1:0] abs_integrator_value;
reg signed [ACC_WIDTH-1:0] temp_scaled_output;
reg signed [17:0] temp_output;  // Temporary output for proper range checking

// Pipeline stage for saturation comparison — breaks CARRY4 chain from timing path
reg sat_pos;            // temp_scaled_output > 131071 (registered)
reg sat_neg;            // temp_scaled_output < -131072 (registered)
reg signed [17:0] temp_output_pipe;  // Registered passthrough value
reg data_out_valid_pipe; // Delayed valid for pipelined output

integer i, j;

// Initialize
initial begin
    for (i = 0; i < STAGES; i = i + 1) begin
        integrator[i] = 0;
        comb[i] = 0;
        for (j = 0; j < COMB_DELAY; j = j + 1) begin
            comb_delay[i][j] = 0;
        end
    end
    decimation_counter = 0;
    data_valid_delayed = 0;
    data_valid_comb = 0;
    output_counter = 0;
    max_integrator_value = 0;
    overflow_detected = 0;
    overflow_latched = 0;
    saturation_detected = 0;
    saturation_event_count = 0;
    sample_count = 0;
    max_value_monitor = 0;
    data_out = 0;
    data_out_valid = 0;
    abs_integrator_value = 0;
    temp_scaled_output = 0;
    temp_output = 0;
    sat_pos = 0;
    sat_neg = 0;
    temp_output_pipe = 0;
    data_out_valid_pipe = 0;
    comb_overflow_latched = 0;
    comb_saturation_detected = 0;
    comb_saturation_event_count = 0;
end

// Enhanced integrator section with proper saturation monitoring
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (i = 0; i < STAGES; i = i + 1) begin
            integrator[i] <= 0;
        end
        decimation_counter <= 0;
        data_valid_delayed <= 0;
        max_integrator_value <= 0;
        overflow_detected <= 0;
        sample_count <= 0;
        abs_integrator_value <= 0;
        overflow_latched <= 0;
        saturation_detected <= 0;
        saturation_event_count <= 0;
        max_value_monitor <= 0;
        output_counter <= 0;
    end else begin
        // Monitor control - clear latched saturation on reset_monitors
        // (must be inside else branch so Vivado sees a clean async-reset FF template)
        if (reset_monitors) begin
            overflow_latched <= 0;
            saturation_detected <= 0;
            max_integrator_value <= 0;
            max_value_monitor <= 0;
            saturation_event_count <= 0;
        end

        if (data_valid) begin
            sample_count <= sample_count + 1;
            
            // Integrator stages — standard CIC uses wrapping (modular) arithmetic.
            // Saturation clamping is removed because CIC math relies on wrap-around;
            // the comb stages difference successive integrator values, canceling wraps.
            integrator[0] <= integrator[0] + {{(ACC_WIDTH-18){data_in[17]}}, data_in};
            
            // Calculate absolute value for monitoring
            abs_integrator_value <= (integrator[0][ACC_WIDTH-1]) ? -integrator[0] : integrator[0];
            
            // Track maximum integrator value for gain monitoring (absolute value)
            if (abs_integrator_value > max_integrator_value) begin
                max_integrator_value <= abs_integrator_value;
                max_value_monitor <= abs_integrator_value[ACC_WIDTH-5:ACC_WIDTH-12];
            end
            
            // Remaining integrator stages — pure accumulation, no saturation
            for (i = 1; i < STAGES; i = i + 1) begin
                integrator[i] <= integrator[i] + integrator[i-1];
            end
            
            // Enhanced decimation control
            if (decimation_counter == DECIMATION - 1) begin
                decimation_counter <= 0;
                data_valid_delayed <= 1;
                output_counter <= output_counter + 1;
            end else begin
                decimation_counter <= decimation_counter + 1;
                data_valid_delayed <= 0;
            end
        end else begin
            data_valid_delayed <= 0;
            overflow_detected <= 1'b0;  // Clear immediate detection when no data
        end
    end
end

// Pipeline the valid signal for comb section
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        data_valid_comb <= 0;
    end else begin
        data_valid_comb <= data_valid_delayed;
    end
end

// Enhanced comb section with FIXED scaling and saturation monitoring
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (i = 0; i < STAGES; i = i + 1) begin
            comb[i] <= 0;
            for (j = 0; j < COMB_DELAY; j = j + 1) begin
                comb_delay[i][j] <= 0;
            end
        end
        data_out <= 0;
        data_out_valid <= 0;
        temp_scaled_output <= 0;
        temp_output <= 0;
        sat_pos <= 0;
        sat_neg <= 0;
        temp_output_pipe <= 0;
        data_out_valid_pipe <= 0;
        comb_overflow_latched <= 0;
        comb_saturation_detected <= 0;
        comb_saturation_event_count <= 0;
    end else begin
        // Monitor control - clear latched comb saturation on reset_monitors
        // (inside else branch so Vivado sees clean async-reset FF template)
        if (reset_monitors) begin
            comb_overflow_latched <= 0;
            comb_saturation_detected <= 0;
            comb_saturation_event_count <= 0;
        end

        if (data_valid_comb) begin
            // Comb processing — raw subtraction only (no saturation check needed;
            // comb is a differencing stage, cannot overflow if integrators are bounded)
            for (i = 0; i < STAGES; i = i + 1) begin
                if (i == 0) begin
                    comb[0] <= integrator[STAGES-1] - comb_delay[0][COMB_DELAY-1];
                    
                    // Update delay line for first stage
                    for (j = COMB_DELAY-1; j > 0; j = j - 1) begin
                        comb_delay[0][j] <= comb_delay[0][j-1];
                    end
                    comb_delay[0][0] <= integrator[STAGES-1];
                end else begin
                    comb[i] <= comb[i-1] - comb_delay[i][COMB_DELAY-1];
                    
                    // Update delay line
                    for (j = COMB_DELAY-1; j > 0; j = j - 1) begin
                        comb_delay[i][j] <= comb_delay[i][j-1];
                    end
                    comb_delay[i][0] <= comb[i-1];
                end
            end
            
            // FIXED: Use proper scaling for 5 stages and decimation by 4
            // Gain = (4^5) = 1024 = 2^10, so scale by 2^10 to normalize
            temp_scaled_output <= comb[STAGES-1] >>> 10;
            
            // FIXED: Extract 18-bit output properly
            temp_output <= temp_scaled_output[17:0];
            
            // Pipeline Stage 2: Register saturation comparison flags
            // This breaks the CARRY4 chain out of the data_out critical path
            sat_pos <= (temp_scaled_output > 131071);
            sat_neg <= (temp_scaled_output < -131072);
            temp_output_pipe <= temp_scaled_output[17:0];
            data_out_valid_pipe <= 1;
        end else begin
            data_out_valid_pipe <= 0;
        end
        
        // Pipeline Stage 3: MUX from registered comparison flags
        if (data_out_valid_pipe) begin
            if (sat_pos) begin
                data_out <= 131071;
                comb_overflow_latched <= 1'b1;
                comb_saturation_detected <= 1'b1;
                comb_saturation_event_count <= comb_saturation_event_count + 1;
                `ifdef SIMULATION
                $display("CIC_OUTPUT_SAT: TRUE Positive saturation, final_out=%d", 131071);
                `endif
            end else if (sat_neg) begin
                data_out <= -131072;
                comb_overflow_latched <= 1'b1;
                comb_saturation_detected <= 1'b1;
                comb_saturation_event_count <= comb_saturation_event_count + 1;
                `ifdef SIMULATION
                $display("CIC_OUTPUT_SAT: TRUE Negative saturation, final_out=%d", -131072);
                `endif
            end else begin
                data_out <= temp_output_pipe;
                comb_overflow_latched <= 1'b0;
                comb_saturation_detected <= 1'b0;
            end
            
            data_out_valid <= 1;
        end else begin
            data_out_valid <= 0;
        end
    end
end

// Continuous monitoring of saturation status
`ifdef SIMULATION
always @(posedge clk) begin
    if (overflow_detected && sample_count < 100) begin
        $display("CIC_OVERFLOW: Immediate detection at sample %0d", sample_count);
    end
end
`endif

// Clear saturation on external reset — handled in integrator always block
// (lines 165-172, using synchronous check of reset_monitors)

endmodule