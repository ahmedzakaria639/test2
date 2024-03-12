////////////////////////////////////////////////////////////////////////////////////////////////////
// Block: control_fsm
//
// Author: Ahmed Zakaria
//
// Description: It oversees the exchange of parameters during the CLd state using transactions. 
//              It also handles the transmission of the appropriate ordered set in the lanes while 
//              ensuring the correct state is maintained. Once the link is trained successfully, 
//              the control unit enables the transmission of data packets.
//
////////////////////////////////////////////////////////////////////////////////////////////////////


`default_nettype none

module control_fsm
(
  input  wire        fsm_clk, 
  input  wire        reset_n,
  input  wire [3:0]  os_in, //received from data bus indicating which os received
  input  wire        disconnect_sbrx, //disconnection requested from other lane
  input  wire        s_write_i,
  input  wire        s_read_i,
  input  wire [7:0]  s_address_i,
  input  wire [23:0] payload_in, //data in received transaction
  input  wire        trans_error, //error in transaction received
  input  wire        t_valid, //pulse received from transactions fsm indicating transaction received
  input  wire        os_sent, //pulse received from data bus indicating os is sent
  input  wire [31:0] c_data_in,
  input  wire        lane_disable,
  input  wire        sync_busy, //shows if pulse synchronizer busy
  input  wire        tdisabled_min,
  input  wire        ttraining_error_timeout,
  input  wire        tgen4_ts1_timeout,
  input  wire        tgen4_ts2_timeout, 
  output reg  [2:0]  trans_sel,
  output reg         disconnect_sbtx, //send zeros in sbtx to complete disconnection
  output reg         fsm_disabled, //indicating fsm is DISABLED
  output reg         fsm_training, //indicating fsm is training
  output reg         ts1_gen4_s, //indicating fsm is sending GEN4 ts1 os
  output reg         ts2_gen4_s, //indicating fsm is sending GEN4 ts2 os
  output reg  [3:0]  d_sel, //selection line to data bus to send required os
  output reg  [1:0]  gen_speed, //GEN4, GEN3 or GEN2
  output reg  [31:0] c_data_out,
  output reg  [7:0]  c_address,
  output reg         c_read_write,
  output reg  [7:0]  s_data_o,
  output reg  [7:0]  s_address_o,
  output reg         s_read_o,
  output reg         s_write_o
);

localparam DISABLED            = 'b0000, //DISABLED state

           //cld sub-state machine
           CLD_CABLE_PROP      = 'b0001,
           CLD_DET_DEVICE      = 'b0010,
           CLD_PARAMETERS      = 'b0011, 
           CLD_CLK_SWITCH      = 'b0100, 
		   
		   //training GEN4 sub-state machine
           TRAINING_GEN4_TS1   = 'b0101,
           TRAINING_GEN4_TS2   = 'b0110,
           TRAINING_GEN4_TS3   = 'b0111,
           TRAINING_GEN4_TS4   = 'b1000,
		   
		   //training GEN3 and GEN2 sub-state machine
           TRAINING_GEN3_SLOS1 = 'b1001,
           TRAINING_GEN3_SLOS2 = 'b1010,
           TRAINING_GEN3_TS1   = 'b1011,
           TRAINING_GEN3_TS2   = 'b1100,
		   
           CL0                 = 'b1101; //data sent normally


localparam GEN4 = 'b00,
           GEN3 = 'b01,
		   GEN2 = 'b10;

reg [3:0]  cs, ns;
	   
reg [5:0]  os_sent_cnt;
reg        enough_os_sent;
reg        os_received;
		   
reg [1:0]  gen_speed_reg,
           cable_gen,
		   opp_adapter_gen;

reg        is_usb4,
           c_address_sent_flag,
	       c_data_received_flag,
	       AT_req_trans_send_flag,
		   AT_req_trans_sent_flag;


always @(posedge fsm_clk or negedge reset_n)
  begin
    if(!reset_n)
      begin
        cs <= DISABLED ;
      end
    else
      begin
        cs <= ns ;
      end
  end


always @(*)
  begin
    case (cs)
    DISABLED : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
		else if (!lane_disable && !tdisabled_min) //won't leave DISABLED state before a min timing
		  ns = DISABLED;
        else 
		  ns = CLD_CABLE_PROP; 
      end
    
    CLD_CABLE_PROP : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (is_usb4) 
		  ns = CLD_DET_DEVICE;
        else 
		  ns = CLD_CABLE_PROP; 
      end
	  
    CLD_DET_DEVICE : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (!disconnect_sbrx) 
		  ns = CLD_PARAMETERS;
        else 
		  ns = CLD_DET_DEVICE; 
      end
	  
    CLD_PARAMETERS : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
        else if (t_valid && !s_read_i && !s_write_i) //AT response received with no errors
		  ns = CLD_CLK_SWITCH;
        else 
		  ns = CLD_PARAMETERS; 
      end
	  
    CLD_CLK_SWITCH : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
        else if (gen_speed == GEN4) 
		  ns = TRAINING_GEN4_TS1;
        else 
		  ns = TRAINING_GEN3_SLOS1; 
      end
    
    TRAINING_GEN4_TS1 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
		else if (tgen4_ts1_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = TRAINING_GEN4_TS2;
        else 
		  ns = TRAINING_GEN4_TS1; 
      end
	  
    TRAINING_GEN4_TS2 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
		else if (tgen4_ts2_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = TRAINING_GEN4_TS3;
        else 
		  ns = TRAINING_GEN4_TS2; 
      end
	  
    TRAINING_GEN4_TS3 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = TRAINING_GEN4_TS4;
        else 
		  ns = TRAINING_GEN4_TS3; 
      end
	  
    TRAINING_GEN4_TS4 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = CL0;
        else 
		  ns = TRAINING_GEN4_TS4; 
      end
	  
    TRAINING_GEN3_SLOS1 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = TRAINING_GEN3_SLOS2;
        else 
		  ns = TRAINING_GEN3_SLOS1; 
      end
	  
    TRAINING_GEN3_SLOS2 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = TRAINING_GEN3_TS1;
        else 
		  ns = TRAINING_GEN3_SLOS2; 
      end
	  
    TRAINING_GEN3_TS1 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = TRAINING_GEN3_TS2;
        else 
		  ns = TRAINING_GEN3_TS1; 
      end
	  
    TRAINING_GEN3_TS2 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
		else if (ttraining_error_timeout)
		  ns = CLD_PARAMETERS;
        else if (enough_os_sent && os_received) 
		  ns = CL0;
        else 
		  ns = TRAINING_GEN3_TS2; 
      end
	  
    CL0 : 
      begin 
        if (lane_disable) 
		  ns = DISABLED;
        else if (disconnect_sbrx) 
		  ns = CLD_DET_DEVICE;
        else 
		  ns = CL0; 
      end
	  
    default : ns = DISABLED;
    endcase
  end

    
always @ (posedge fsm_clk or negedge reset_n)    
  begin
    if (!reset_n)
	  begin
        d_sel <= 'h9; //zeros in lanes
        gen_speed <= GEN4;
        c_data_out <= 'h0;
        c_address <= 'h0;
        c_read_write <= 1;   
        c_address_sent_flag <= 0;
        c_data_received_flag <= 0;		
	    os_sent_cnt <= 'h0;
        enough_os_sent <= 0;
        os_received <= 0;
        cable_gen <= GEN4;
        opp_adapter_gen <= GEN4;
        is_usb4 <= 0;
		fsm_disabled <= 1;
        fsm_training <= 0;
        ts1_gen4_s <= 0;
        ts2_gen4_s <= 0;
	  end
	  
	else
	  begin
		fsm_disabled <= 1;
        fsm_training <= 0;
        ts1_gen4_s <= 0;
        ts2_gen4_s <= 0;
		
        case (cs)
        DISABLED : 
          begin
            d_sel <= 'h9; //zeros in lanes
            gen_speed <= GEN4;
            c_address <= 'h0;
            c_read_write <= 1;	
            c_address_sent_flag <= 0;
            c_data_received_flag <= 0;			
	        os_sent_cnt <= 'h0;
            enough_os_sent <= 0;
            os_received <= 0;
            cable_gen <= GEN4;
            opp_adapter_gen <= GEN4;
            is_usb4 <= 0;
			fsm_disabled <= 1;
          end
	  
        CLD_CABLE_PROP : 
          begin
            d_sel <= 'h9; //zeros in lanes
            gen_speed <= GEN4;				
	        os_sent_cnt <= 'h0;
            enough_os_sent <= 0;
            os_received <= 0;
		    // reading from cfg spaces
            c_address <= 'd18;
            c_read_write <= 1; //read
			
			if (ns != CLD_CABLE_PROP)
			  begin
			    c_address_sent_flag <= 0;
				c_data_received_flag <= 0;
			  end
			else
			  begin
                c_address_sent_flag <= 1; //raised after entering this state by 1 clk cycle indicating address reached the cng spaces
                c_data_received_flag <= c_address_sent_flag; //if address is sent --> data rec next clock cycle
			  end
			  
            is_usb4 <= (c_data_in[7:0] == 'h40);
			if(c_data_in[21] == 1)
			  cable_gen <= GEN4;
			else if(c_data_in[20] == 1)
			  cable_gen <= GEN3;
			else
			  cable_gen <= GEN2;
          end
		  
        CLD_DET_DEVICE : 
          begin
            d_sel <= 'h9; //zeros in lanes
            gen_speed <= GEN4;				
	        os_sent_cnt <= 'h0;
            enough_os_sent <= 0;
            os_received <= 0;
          end
		  
        CLD_PARAMETERS : 
          begin
	        //deciding which gen speed to operate with
			if (cable_gen == GEN2 || opp_adapter_gen == GEN2)
		      gen_speed <= GEN2;
		    else if (cable_gen == GEN3 || opp_adapter_gen == GEN3)
		      gen_speed <= GEN3;
		    else
		      gen_speed <= GEN4;
			  
			//gen of other adapter extracted from received data in the AT response received
			if(payload_in [18] == 1)
			  opp_adapter_gen <= GEN4;
			else if(payload_in [13] == 1)
			  opp_adapter_gen <= GEN3;
			else
			  opp_adapter_gen <= GEN2;
			  
			d_sel <= 'h9; //zeros in lanes   
	        os_sent_cnt <= 'h0;
            enough_os_sent <= 0;
            os_received <= 0;
          end
		  
        TRAINING_GEN4_TS1 : 
          begin
            fsm_training <= 1;
			ts1_gen4_s <= 1;
			d_sel <= 'h4; //GEN4 TS1 in lanes back-to-back	
			
	        if (ns == TRAINING_GEN4_TS1)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h4)
				  os_received <= 1;
				if (os_sent_cnt == 'd16)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		   
        TRAINING_GEN4_TS2 : 
          begin
            fsm_training <= 1;
			ts2_gen4_s <= 1;
			d_sel <= 'h5; //GEN4 TS2 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN4_TS2)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h5)
				  os_received <= 1;
				if (os_sent_cnt == 'd16)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		  
        TRAINING_GEN4_TS3 : 
          begin
            fsm_training <= 1;
			d_sel <= 'h6; //GEN4 TS3 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN4_TS3)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h6)
				  os_received <= 1;
				if (os_sent_cnt == 'd16)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		  
        TRAINING_GEN4_TS4 : 
          begin
            fsm_training <= 1;
			d_sel <= 'h7; //GEN4 TS4 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN4_TS4)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h7)
				  os_received <= 1;
				if (os_sent_cnt == 'd16)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		  
        TRAINING_GEN3_SLOS1 : 
          begin
            fsm_training <= 1;
			d_sel <= 'h0; //GEN3 SLOS1 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN3_SLOS1)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h0)
				  os_received <= 1;
				if (os_sent_cnt == 'd32)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		  
        TRAINING_GEN3_SLOS2 : 
          begin
            fsm_training <= 1;
			d_sel <= 'h1; //GEN3 SLOS2 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN3_SLOS2)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h1)
				  os_received <= 1;
				if (os_sent_cnt == 'd32)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		  
        TRAINING_GEN3_TS1 : 
          begin
            fsm_training <= 1;
			d_sel <= 'h2; //GEN3 TS1 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN3_TS1)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h2)
				  os_received <= 1;
				if (os_sent_cnt == 'd32)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
		  
        TRAINING_GEN3_TS2 : 
          begin
            fsm_training <= 1;
			d_sel <= 'h3; //GEN3 TS2 in lanes back-to-back			
	        
			if (ns == TRAINING_GEN3_TS2)
			  begin
				if (os_sent)
				  os_sent_cnt <= os_sent_cnt + 1;
				if (os_in == 'h3)
				  os_received <= 1;
				if (os_sent_cnt == 'd32)
				  enough_os_sent <= 1;
			  end
			else 
			  begin
				os_sent_cnt <= 'h0;
                enough_os_sent <= 0;
                os_received <= 0;
			  end
          end
    
        CL0:
		  begin
		    d_sel <= 'h8; //transport layer data in lanes
		  end
		
		default :
          begin
            d_sel <= 'h9; //zeros in lanes
            gen_speed <= GEN4;
            c_address <= 'h0;
            c_read_write <= 1;	
            c_address_sent_flag <= 0;
            c_data_received_flag <= 0;			
	        os_sent_cnt <= 'h0;
            enough_os_sent <= 0;
            os_received <= 0;
            cable_gen <= GEN4;
            opp_adapter_gen <= GEN4;
            is_usb4 <= 0;
          end
        endcase
      end
  end
 

////////////////////////////// Transactions ////////////////////////////////////
always @ (posedge fsm_clk or negedge reset_n)
  begin
    if (!reset_n)
      begin
	    AT_req_trans_send_flag <= 0; //order to send a request AT transaction
		AT_req_trans_sent_flag <= 0; //flag indicating the transaction is sent
	    trans_sel <= 'h0;
		s_write_o <= 0;
	    s_read_o <= 0;
	    s_address_o <= 'b0;
	    s_data_o <= 'b0;
		disconnect_sbtx <= 1; //zeros in sbtx
      end	
	
	else
	  begin
	    if (cs == CLD_PARAMETERS)
	      begin
	        AT_req_trans_send_flag <= 1; //command to send AT transaction next clk cycle to request parameters from other lane
		    AT_req_trans_sent_flag <= (trans_sel == 'h2); //if the tranaction is executed, this flag is raised
	      end
		else
		  begin
	        AT_req_trans_send_flag <= 0;
		    AT_req_trans_sent_flag <= 0; 
		  end
		
  	    if (cs == DISABLED || cs == CLD_CABLE_PROP)
		  begin
		    disconnect_sbtx <= 1; //zeros in sbtx
			trans_sel <= 'h0; 
			s_read_o <= 0;
			s_write_o <= 0;
          end
		
		else if ((AT_req_trans_send_flag && !AT_req_trans_sent_flag) || (cs == CLD_PARAMETERS && trans_error))
		  begin
		    if(!sync_busy)
			  trans_sel <= 'h2; //to send AT transaction to obtain parameters from other lane
			  
			s_read_o <= 0;
			s_write_o <= 0;
			disconnect_sbtx <= 0;
          end			
		  
		else if (t_valid && s_write_i)
          begin
	        s_write_o <= 1;
			s_read_o <= 0;
		    s_address_o <= s_address_i;
	        s_data_o <= payload_in;
			disconnect_sbtx <= 0;
	      end
	  
	    else if (t_valid && s_read_i)
	      begin
			if(!sync_busy)
			  trans_sel <= 'h3;
			  
			s_read_o <= 1;
			s_write_o <= 0;
	        s_address_o <= s_address_i;			  
			disconnect_sbtx <= 0;
	      end
		  
		else
		  begin
		    trans_sel <= 'h0;
			disconnect_sbtx <= 0;
			s_read_o <= 0;
			s_write_o <= 0;
		  end
      end
  end
  
  
endmodule

`resetall