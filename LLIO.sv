/* HDL implementation of Low-Latency API protocol
* 
* Copyright 2019 Jamie Dickson aka Kitrinx
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
* documentation files (the "Software"), to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or substantial portions
* of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
* TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

// Feb, 1 2019 - Initial Release

// This module requires the following assigments in the project .qsf file:
// set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to USBIO_RX
// set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to USBIO_TX
// set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to USBIO_RX
// set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to USBIO_TX
// set_location_assignment PIN_AG11 -to USBIO_RX
// set_location_assignment PIN_AH9 -to USBIO_TX

// And the following corresponding ports in the top level:
// inout USBIO_TX,     // Pin AH9
// inout USBIO_RX,     // Pin AG11

// Data payload bit order:

// Byte 0:
// uint8_t Controller Type (or error code)
// Notable ID's are:
// 18 - NES
// 21 - Gen 3 button
// 22 - Gen 6 button
// 27 - SNES
// 28 - NES Zapper
// 41 - Atari Paddles
// 51 - NeGcon
// 65 - PSX Digital
// 11 - PSX DS
// 12 - PSX DS2
// more info: https://docs.google.com/document/d/12XpxrmKYx_jgfEPyw-O2zex1kTQZZ-NSBdLO2RQPRzM/edit

// Bytes 1-2:
// Buttons
// System SNES    Genesis PSX    Gun   Saturn
//  0:     Y       A       □      Click A
//  1:     B       B       ×      Light B
//  2:     X       X       △      NA    X
//  3:     A       Y       ○      NA    Y
//  4:     Select  Mode    Select NA    NA
//  5:     Start   Start   Start  NA    Start
//  6:     LT      NA      L1     NA    L
//  7:     RT      NA      R1     NA    R

//  8:     NA      Z       L2     NA    Z
//  9:     NA      C       R2     NA    C
// 10:     U       U       U      NA    NA
// 11:     D       D       D      NA    NA
// 12:     L       L       L      NA    NA
// 13:     R       R       R      NA    NA
// 14:     0       NA      L3     NA    NA
// 15:     1       NA      R3     NA    NA

// 16:     2       NA      NA     NA    NA
// 17:     3       NA      NA     NA    NA
// 18:     4       NA      NA     NA    NA
// 19:     5       NA      NA     NA    NA
// 20:     6       NA      NA     NA    NA
// 21:     7       NA      NA     NA    NA
// 22:     8       NA      NA     NA    NA
// 23: This bit can be the "special" trigger button for retroarch
// more info: https://docs.google.com/spreadsheets/d/1Bk3j5kaKfV1tOfzCq3GLKFsff027RdmwOuSfBLM3Ims/edit#gid=0

// Bytes 4-12: (uint8_t's)
// 04: Axis 1 X
// 05: Axis 1 Y
// 06: Axis 1 Z
// 07: Axis 2 X
// 08: Axis 2 Y
// 09: Axis 2 Z
// 10: Slider
// 11: Dial
// 12: Hat

// Get mode status:
// 0: autopause state
// 1:
// 2: 
// 3: Hotswap Disabled
// 4: UDLR mode
// 5: Analog to D-pad
// 6: autopause dis.
// 7: d-pad only mode

module LLIO
(
	input         CLK_50M,
	input         ENABLE,        // If 0, module will be disabled and pins will be set to Z
	inout         IO_LATCH,      // TX/D- top level IO pin
	inout         IO_DATA,       // RX/D+ top level IO pin
	input         LLIO_SYNC,     // Pos edge corresponds with when the core needs the data, from core
	output        LLIO_EN,       // High when device is communicating, passed to core
	output [7:0]  LLIO_TYPE,     // Enumerated controller type, passed to core
	output [23:0] LLIO_BUTTONS,  // Vector of buttons, 1 == pressed, passed to core
	output [47:0] LLIO_ANALOG    // Unsigned 8 bit vector of analog axis, passed to core
);

assign LLIO_TYPE = lljs_type;
assign LLIO_BUTTONS = lljs_buttons;
assign LLIO_ANALOG = lljs_analog;
assign LLIO_EN = enable;

// Commands
localparam LLIO_POLL               = 32'h00; // Holds latch low until done
localparam LLIO_STATUS             = 32'h01; // Returns 13 bytes, see above
localparam LLIO_PRESSURE_STATUS    = 32'h02; // Returns ?? bytes
localparam LLIO_SET_MODES          =  8'h2F;
localparam LLIO_GET_MODES          = 32'h3F; // Returns bit field of active modes, 1 byte. see above
// Rumble
localparam LLIO_RUMBLE_CONST_START_FROM_PARMS = 32'h11; //must set parms first
localparam LLIO_RUBMLE_CONST_END              = 32'h12;
localparam LLIO_RUMBLE_SINE_START_FROM_PARMS  = 32'h14; //must set parms first
localparam LLIO_RUMBLE_SINE_END               = 32'h18;
localparam LLIO_RUMBLE_CONST_JOLT             = 32'h1A;
localparam LLIO_RUMBLE_SINE_JOLT              = 32'h1B;
localparam LLIO_RUMBLE_PARMS                  = 32'h1C; //followed by 2 bytes of data containing the parms (rumbleLevel and then rumbleLoop)
// Errors
localparam LLIO_ERROR_NODATA       = 'h00;
localparam LLIO_ERROR_AP_NO_REPORT = 'hFF;
// States
localparam STATE_IDLE        = 4'h0;
localparam STATE_WRITE_START = 4'h1;
localparam STATE_WRITE       = 4'h2;
localparam STATE_WRITE_END   = 4'h3;
localparam STATE_READ_START  = 4'h4;
localparam STATE_READ_WAIT   = 4'h5;
localparam STATE_READ        = 4'h6;
localparam STATE_READ_END    = 4'h7;
localparam STATE_POLL        = 4'h8;
// Read types
localparam READ_IDLE           = 3'h0;
localparam READ_POLL           = 3'h1;
localparam WRITE_STATUS        = 3'h2;
localparam READ_STATUS         = 3'h3;
localparam WRITE_SETUP         = 3'h4;
localparam WRITE_MODES         = 3'h5;
localparam READ_MODES          = 3'h6;
// Timing (one 50mhz cycle == 0.02us)
localparam TIME_POLL   =  21'd820000; // 16.4ms - default polling period if no sync is used
localparam TIME_SETTLE = 'd150;    // 3us - for bi-directional IO pins to settle
localparam TIME_LEADIN = 'd75;     // 1.5us - at start of new transactions
localparam TIME_BIT_H  = 'd110;    // 2.2us - always-high first bit-half
localparam TIME_BIT_R  = 'd115;    // 2.3us - variable second bit-half
localparam TIME_SYNC_H = 'd50;     // 1us - sync pulse between bytes high time
localparam TIME_SYNC_L = 'd50;     // 1us - sync pulse between bytes low time

logic [20:0]  cycle, count, poll_offset, poll_counter, sync_counter;
logic [31:0]  write_buffer;
logic [3:0]   write_length;
logic [2:0]   read_bit; // Relies on overflow
logic [3:0]   read_byte;
logic [7:0]   read_buffer[13];
logic [3:0]   read_length;

logic [23:0]  lljs_buttons;
logic [47:0]  lljs_analog;
logic [7:0]   lljs_type;
logic [7:0]   lljs_modes;

reg   [2:0]   read_type = READ_IDLE;
reg   [3:0]   state = STATE_IDLE;
reg   [20:0]  poll_time = TIME_POLL;

logic [1:0] latch;
logic is_latched;
logic data_in, data_out;
logic enable;
logic old_sync, new_sync, old_data;

always_comb begin
	case (latch)
		2'b00: begin
			IO_LATCH <= 1'bZ;
			IO_DATA <= 1'bZ;
		end

		2'b01: begin
			IO_LATCH <= 1'b0;
			IO_DATA <= data_out;
		end

		2'b10: begin
			IO_LATCH <= 1'b1;
			IO_DATA <= 1'b1;
		end

		default: begin
			IO_LATCH <= 1'bZ;
			IO_DATA <= 1'bZ;
		end
	endcase
end

always_ff @(posedge CLK_50M) begin
	if (ENABLE) begin
	old_sync <= new_sync;
	new_sync <= LLIO_SYNC;
	cycle <= cycle + 1'b1;

	if (~latch) begin
		is_latched <= ~IO_LATCH;
		old_data <= data_in;
		data_in <= IO_DATA;
	end

	if (~old_sync && new_sync) begin
		sync_counter <= 0;
		poll_time <= poll_counter - 21'd9500; // about 3 scanlines of wobble room
		poll_counter <= 0;
	end else begin
		sync_counter <= sync_counter + 1'b1;
		poll_counter <= poll_counter + 1'b1;
	end

	if (read_type == READ_POLL || read_type == READ_STATUS)
		poll_offset <= poll_offset + 1'b1;

	case (state)
		STATE_IDLE: begin // Idle
			latch <= (~enable && sync_counter > 'd4000) ? 2'b10 : 2'b00; // Assist weak pull up resistors put device in LL mode
			if (is_latched) begin // remote device wants to talk
				enable <= 1'b1;
				cycle <= 0;
				count <= 0;
				read_byte <= 0;
				state <= (read_type == READ_POLL) ? STATE_POLL : STATE_READ_START;
			end else if (read_type == WRITE_STATUS) begin // We just got a poll result
				cycle <= 0;
				state <= STATE_WRITE_START;
				read_type <= READ_STATUS;
				read_length <= 'd13;
				write_buffer <= LLIO_STATUS;
			end else if (read_type == WRITE_MODES) begin
				cycle <= 0;
				read_type <= READ_MODES;
				read_length <= 1'd1;
				state <= STATE_WRITE_START;
				write_buffer <= LLIO_GET_MODES;
			end else if (sync_counter >= (((poll_offset < (poll_time >> 1)) && enable) ?
				(poll_time - poll_offset) : poll_time)) begin // Trigger timed device poll. Offset can not be > half the poll time.
				if (read_type != READ_IDLE) begin // IO timeout, device disconnect/defunct
					enable <= 1'b0;
					poll_offset <= 0;
					poll_time <= TIME_POLL;
					lljs_buttons <= 24'h0;
					lljs_type <= 8'h0;
					lljs_analog <= 48'h808080808080;
				end
				poll_offset <= 0;
				sync_counter <= 0;
				cycle <= 0;
				state <= STATE_WRITE_START;
				read_type <= READ_POLL;
				write_buffer <= LLIO_POLL;
			end
		end

		STATE_WRITE_START: begin
			if (cycle == 0) begin
				data_out <= 1'b0; 
				latch <= 2'b01; // Take control
				write_length <= 'h8; // Always 8 for now
			end else if (cycle >= TIME_LEADIN) begin
				cycle <= 0;
				state <= STATE_WRITE;
			end
		end

		STATE_WRITE: begin
			if (write_length == 0) begin
				state <= STATE_WRITE_END;
				cycle <= 0;
			end else if (cycle == 'd0) begin // high half-bit
				data_out <= 1'b1;
			end else if (cycle == TIME_BIT_H) begin // data half-bit
				data_out <= ~write_buffer[0];
				write_buffer <= {1'b0, write_buffer[31:1]};
			end else if (cycle > (TIME_BIT_R + TIME_BIT_H)) begin
				write_length <= write_length - 1'b1;
				cycle <= 0;
			end
		end

		STATE_WRITE_END: begin
			if (cycle == 0)
				data_out <= 1'b1;
			else if (cycle == TIME_SYNC_H)
				data_out <= 1'b0;
			else if (cycle == (TIME_SYNC_H + TIME_SYNC_L)) begin
				latch <= 2'b10; // Bump latch and data to ensure proper sync
			end else if (cycle >= (TIME_SYNC_H + TIME_SYNC_L + TIME_SETTLE)) begin
				latch <= 2'b00;
				state <= STATE_IDLE;
				cycle <= 0;
			end
		end

		STATE_READ_START: begin
			if (cycle > TIME_LEADIN) begin
				cycle <= 0;
				read_bit <= 3'h0;
				state <= STATE_READ; // Helps reduce noise and errors
			end
		end

		STATE_READ_WAIT: begin // the space between the bytes
			if (cycle > (TIME_SYNC_H + TIME_SYNC_L + 'd5) ||
				((cycle > 'd25) && ~old_data && data_in)) begin // Re align during wait if needed
				cycle <= 0;
				read_bit <= 3'h0;
				state <= STATE_READ;
			end
		end

		STATE_READ: begin
			if (~is_latched) begin //XXX: Accounts from random "blip" on latch before data. Remove if possible.
				cycle <= 0;
				state <= STATE_IDLE;
			end if (cycle == (TIME_BIT_H + (TIME_BIT_R >> 1))) begin // latch in the middle of the data window
				read_buffer[read_byte][read_bit] <= ~data_in;
				read_bit <= read_bit + 1'b1;
			end else if (cycle > (TIME_BIT_H + TIME_BIT_R) ||
				(cycle > TIME_BIT_H + 'd20 && ~old_data && data_in)) begin
				//XXX: account for shorter low bit-halves, remove if able
				cycle <= 0;
				if (read_bit == 3'h0) begin // This relies on overflow behavior
					if (read_byte == 0) begin
						if (read_type == READ_STATUS) begin
							lljs_type <= read_buffer[0];
						end else if (read_type == READ_MODES) begin
							lljs_modes <= read_buffer[0];
						end
					end else if (read_byte == 'd3)
						lljs_buttons[23:0] <= {read_buffer[3], read_buffer[2], read_buffer[1]};
					else if (read_byte == 'd12)
						lljs_analog[47:0] <=
							{read_buffer[11], read_buffer[10], read_buffer[8],
							read_buffer[7], read_buffer[5], read_buffer[4]};
					if (read_byte >= read_length - 1'd1) begin
						state <= STATE_READ_END;
					end else begin
						read_byte <= read_byte + 1'd1;
						state <= STATE_READ_WAIT;
					end
				end
			end
		end

		STATE_READ_END: begin
			// latch data to correct registers based on read_type
			if (cycle >= TIME_SETTLE + TIME_SYNC_H + TIME_SYNC_L) begin
				if (read_type == READ_STATUS) begin
					if (lljs_modes[7]) begin
						lljs_buttons[10] <= lljs_buttons[10] | (LLIO_ANALOG[15:8] < 'd97);
						lljs_buttons[11] <= lljs_buttons[10] | (LLIO_ANALOG[15:8] > 'd157);
						lljs_buttons[12] <= lljs_buttons[10] | (LLIO_ANALOG[7:0]  < 'd97);
						lljs_buttons[13] <= lljs_buttons[10] | (LLIO_ANALOG[7:0]  > 'd157);
					end
					read_type <= READ_IDLE; //XXX: WRITE_MODES -- NYI;
				end else begin
					read_type <= READ_IDLE;
				end
				cycle <= 0;
				state <= STATE_IDLE;
			end
		end

		STATE_POLL: begin // Polling hold timer
			// Allow to settle, then idle while the device is polling the controller
			if (cycle >= TIME_SETTLE && ~is_latched) begin
				count <= count + 1'b1;
			end
			if (count > TIME_SETTLE) begin
				cycle <= 0;
				count <= 0;
				read_type <= WRITE_STATUS;
				state <= STATE_IDLE;
			end
		end

		default: begin
			cycle <= 0;
			count <= 0;
			read_type <= READ_IDLE;
			state <= STATE_IDLE;
		end
	endcase
	end else
		latch <= 2'b00;
		enable <= 1'b0;
		lljs_analog <= 0;
		lljs_buttons <= 0;
		lljs_modes <= 0;
		lljs_type <= 0;
		state <= STATE_IDLE;
		read_type <= READ_IDLE;
	end
end

endmodule