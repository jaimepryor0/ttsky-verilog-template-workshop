module filter(clk, rst_n, b0, b1, b2, a1, a2, in, out);
// Assume normalised coefficients i.e. a0 = 1
// Second order 
    input clk; 
    input rst_n;
    // signed to make sure multiplication with negative numbers works ok
    input signed [7:0] in; 
    output signed [7:0] out; 
    parameter signed b0, b1, b2, a1, a2;


    reg signed [7:0] s1; // state register 1 
    reg signed [7:0] s2; // state register 2 

    // Assign output 
    assign out = in*b0 + s1; 


    always @(posedge clk) begin // synchronous reset 
        if (~rst_n) begin
            s1 <= 0; // this is what makes this a register 
            s2 <= 0;
        end else begin
            // Update state registers 
            s1 <= in*b1 - out*a1 + s2; // TODO: truncation 
            s2 <=  in*b2 - out*a2;
        end
    end  
endmodule


