// coding: utf-8

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "can.h"

/* ids used for ECU detection */
#define PCM_ID_REQUEST              0x7e0 
#define PCM_ID_RESPONSE             0x7e8

// -----------------------------------------------------------------------------
/** Set filters and masks.
 *
 * The filters are divided in two groups:
 *
 * Group 0: Filter 0 and 1 with corresponding mask 0.
 * Group 1: Filter 2, 3, 4 and 5 with corresponding mask 1.
 *
 * If a group mask is set to 0, the group will receive all messages.
 *
 * If you want to receive ONLY 11 bit identifiers, set your filters
 * and masks as follows:
 *
 *	uint8_t can_filter[] PROGMEM = {
 *		// Group 0
 *		MCP2515_FILTER(0),				// Filter 0
 *		MCP2515_FILTER(0),				// Filter 1
 *		
 *		// Group 1
 *		MCP2515_FILTER(0),				// Filter 2
 *		MCP2515_FILTER(0),				// Filter 3
 *		MCP2515_FILTER(0),				// Filter 4
 *		MCP2515_FILTER(0),				// Filter 5
 *		
 *		MCP2515_FILTER(0),				// Mask 0 (for group 0)
 *		MCP2515_FILTER(0),				// Mask 1 (for group 1)
 *	};
 *
 *
 * If you want to receive ONLY 29 bit identifiers, set your filters
 * and masks as follows:
 *
 * \code
 *	uint8_t can_filter[] PROGMEM = {
 *		// Group 0
 *		MCP2515_FILTER_EXTENDED(0),		// Filter 0
 *		MCP2515_FILTER_EXTENDED(0),		// Filter 1
 *		
 *		// Group 1
 *		MCP2515_FILTER_EXTENDED(0),		// Filter 2
 *		MCP2515_FILTER_EXTENDED(0),		// Filter 3
 *		MCP2515_FILTER_EXTENDED(0),		// Filter 4
 *		MCP2515_FILTER_EXTENDED(0),		// Filter 5
 *		
 *		MCP2515_FILTER_EXTENDED(0),		// Mask 0 (for group 0)
 *		MCP2515_FILTER_EXTENDED(0),		// Mask 1 (for group 1)
 *	};
 * \endcode
 *
 * If you want to receive both 11 and 29 bit identifiers, set your filters
 * and masks as follows:
 *
* const uint8_t can_filter[] PROGMEM = 
* {
* 	// Group 0
*	MCP2515_FILTER(0),				// Filter 0
*	MCP2515_FILTER(0),				// Filter 1
*	
*	// Group 1
*	MCP2515_FILTER_EXTENDED(0),		// Filter 2
*	MCP2515_FILTER_EXTENDED(0),		// Filter 3
*	MCP2515_FILTER_EXTENDED(0),		// Filter 4
*	MCP2515_FILTER_EXTENDED(0),		// Filter 5
*	
*	MCP2515_FILTER(0),				// Mask 0 (for group 0)
*	MCP2515_FILTER_EXTENDED(0),		// Mask 1 (for group 1)
* };
// You can receive 11 bit identifiers with either group 0 or 1.
* \endcode 
*/
uint8_t can_filter[] PROGMEM = {
    // Group 0
    MCP2515_FILTER(0),				// Filter 0
    MCP2515_FILTER(0),				// Filter 1
    
    // Group 1
    MCP2515_FILTER(0),				// Filter 2
    MCP2515_FILTER(0),				// Filter 3
    MCP2515_FILTER(0),				// Filter 4
    MCP2515_FILTER(0),				// Filter 5
    
    MCP2515_FILTER(0),				// Mask 0 (for group 0)
    MCP2515_FILTER(0),				// Mask 1 (for group 1)
};

int         ect_voltage = 0;
int         ect_temperature = 0;
int         engine_fan_speed = 0;

// -----------------------------------------------------------------------------
// Main loop for receiving and sending messages.

int main(void)
{
	// Initialize MCP2515
	// can_init(BITRATE_500_KBPS);
    can_init(BITRATE_1_MBPS); /* for some retarded reason 1_mbps needs to be setup to get 500 kbps baudrate 
                                 possible AVR oscilator problem / wrong BTR values */
	
	// Load filters and masks
	can_static_filter(can_filter);
	
	// Create a test messsage
	can_t rx_msg;
    can_t tx_msg; 
	
	rx_msg.id = PCM_ID_RESPONSE;
	rx_msg.flags.rtr = 0;
	rx_msg.flags.extended = 0;
	rx_msg.length = 8;

    tx_msg.id = PCM_ID_RESPONSE;
	tx_msg.flags.rtr = 0;
	tx_msg.flags.extended = 0;
	tx_msg.length = 8;

	while (1)
	{
		// Check if a new messag was received
		if (can_check_message())
		{
			// Try to read the message
			if (can_get_message(&rx_msg))
			{

                if (rx_msg.id == PCM_ID_REQUEST)
                {

                    /* Request for data */
                    if (rx_msg.data[1] == 0x22)
                    {
                        /* --- ECU AND VEHICLE IDENTIFICATION --- */

                        /* call PCM */
                        if (rx_msg.data[2] == 0x02)
                        {
                            tx_msg.data[0] = 0x3;
                            tx_msg.data[1] = rx_msg.data[1] + 0x40;
                            tx_msg.data[2] = rx_msg.data[2];
                            tx_msg.data[3] = rx_msg.data[3];
                            tx_msg.data[4] = 0x00;
                            tx_msg.data[5] = 0x00;
                            tx_msg.data[6] = 0x00;
                            tx_msg.data[7] = 0x00;
                            can_send_message(&tx_msg);   
                        }

                        /* --- ECU & Vehicle IDENTIFICATION PART --- */

                        /* System Identification */
                        else if (rx_msg.data[2] ==  0xC9)
                        {
                            if (rx_msg.data[3] == 0x2E)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'S';
                                tx_msg.data[5] = 'I';
                                tx_msg.data[6] = 'M';
                                tx_msg.data[7] = '-';
                                can_send_message(&tx_msg);    
                            }
                            else if (rx_msg.data[3] == 0x2F)
                            {
                                tx_msg.data[0] = 0x5;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '2';
                                tx_msg.data[5] = '8';
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);    
                            }
                        }

                        /* send manufacturng date, ECU Part number, ECU serial number */
                        else if (rx_msg.data[2] ==  0xE2)
                        {
                            /* Send Manufacturing Date -> Ford Focus 2006/57/27 */
                            if (rx_msg.data[3] == 0x00)
                            {
                                tx_msg.data[0] = 0x6;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 57;  
                                tx_msg.data[5] = 27;
                                tx_msg.data[6] = 106; /* Manufacturing year - 1900 */
                                tx_msg.data[7] = 0x00;   /* I gues it's padding */
                                can_send_message(&tx_msg);    
                            }

                            /* ECU Part Number #1 part */
                            else if (rx_msg.data[3] == 0x1A)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '7';
                                tx_msg.data[5] = 'M';
                                tx_msg.data[6] = '5';
                                tx_msg.data[7] = '1';
                                can_send_message(&tx_msg);   
                            }

                            /* ECU Part Number #2 part */
                            else if (rx_msg.data[3] == 0x17)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x12;
                                tx_msg.data[5] = 0xA;
                                tx_msg.data[6] = 0x06;
                                tx_msg.data[7] = 0x50;
                                can_send_message(&tx_msg);   
                            }

                            /* ECU Part Number #3 part */
                            else if (rx_msg.data[3] == 0x1D)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'A';
                                tx_msg.data[5] = 'F';
                                tx_msg.data[6] = 'B';
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);   
                            }


    /*
                            else if (rx_msg.data[3] == 0x18)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'A';
                                tx_msg.data[5] = 'A';
                                tx_msg.data[6] = 'A';
                                tx_msg.data[7] = 'A';
                                can_send_message(&tx_msg);   
                            }


                            else if (rx_msg.data[3] == 0x19)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'B';
                                tx_msg.data[5] = 'B';
                                tx_msg.data[6] = 'B';
                                tx_msg.data[7] = 'B';
                                can_send_message(&tx_msg);   
                            }

                            else if (rx_msg.data[3] == 0x1B)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'D';
                                tx_msg.data[5] = 'D';
                                tx_msg.data[6] = 'D';
                                tx_msg.data[7] = 'D';
                                can_send_message(&tx_msg);   
                            }


                            else if (rx_msg.data[3] == 0x1C)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'E';
                                tx_msg.data[5] = 'E';
                                tx_msg.data[6] = 'E';
                                tx_msg.data[7] = 'E';
                                can_send_message(&tx_msg);   
                            }

                            else if (rx_msg.data[3] == 0x1E)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'G';
                                tx_msg.data[5] = 'G';
                                tx_msg.data[6] = 'G';
                                tx_msg.data[7] = 'G';
                                can_send_message(&tx_msg);   
                            }
    */

                            /* send serial number 270487626 as asii characters.*/
                            /* I have added a 5ms delay between message sending 
                             * but am not sure if it is needed. 1ms will not work  */
                            else if (rx_msg.data[3] == 0x21)
                            {

                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '2';
                                tx_msg.data[5] = '7';
                                tx_msg.data[6] = '0';
                                tx_msg.data[7] = '4';
                                can_send_message(&tx_msg);    
                                _delay_ms(5);

                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '8';
                                tx_msg.data[5] = '7';
                                tx_msg.data[6] = '6';
                                tx_msg.data[7] = '2';
                                can_send_message(&tx_msg);    
                                _delay_ms(5);

                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '6';
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);    
                                _delay_ms(5);
                            }
                        }

                        /* Send VIN */
                        else if (rx_msg.data[2] ==  0xE3)
                        {
                            if (rx_msg.data[3] == 0x00)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x00;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 'W';
                                can_send_message(&tx_msg);    
                            }
                            else if (rx_msg.data[3] == 0x01)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'F';
                                tx_msg.data[5] = '0';
                                tx_msg.data[6] = '5';
                                tx_msg.data[7] = 'X';
                                can_send_message(&tx_msg);    
                            }
                            else if (rx_msg.data[3] == 0x02)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 'X';
                                tx_msg.data[5] = 'G';
                                tx_msg.data[6] = 'C';
                                tx_msg.data[7] = 'D';
                                can_send_message(&tx_msg);    
                            }
                            else if (rx_msg.data[3] == 0x03)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '5';
                                tx_msg.data[5] = '7';
                                tx_msg.data[6] = 'M';
                                tx_msg.data[7] = '2';
                                can_send_message(&tx_msg);    
                            }
                            else if (rx_msg.data[3] == 0x04)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = '2';
                                tx_msg.data[5] = '8';
                                tx_msg.data[6] = '8';
                                tx_msg.data[7] = '8';
                                can_send_message(&tx_msg);    
                            }
                        }

                        if (rx_msg.data[2] == 0x18)
                        {
                            if (rx_msg.data[3] == 0x00)
                            {
                                tx_msg.data[0] = 0x7;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0xFF;
                                tx_msg.data[5] = 0x01;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;  
                                can_send_message(&tx_msg);                              
                            }
                        }

                        /* --- END OF VEHICLE AND ECU IDENTIFICATION --- */

                        /* --- SENSOR DATA SENDING --- */
        
                        /* --- A/C Pressure Switch  ON/OFF --- */
                        if (rx_msg.data[2] == 0x11)
                        {
                            if (rx_msg.data[3] == 0x02)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x01;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- A/C Pressure Switch [V]  ---*/
                        if (rx_msg.data[2] == 0x16)
                        {
                            if (rx_msg.data[3] == 0x38)
                            {
                                tx_msg.data[0] = 0x5;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x01;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- A/C Request Signal ON / OFF -- */
                        if (rx_msg.data[2] == 0x11)
                        {
                            if (rx_msg.data[3] == 0x01)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x01;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }


                        /* --- ABS Yaw Rate Value  Deg / Second ---- */
                        if (rx_msg.data[2] == 0xC9)
                        {
                            if (rx_msg.data[3] == 0x10)
                            {
                                tx_msg.data[0] = 0x5;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x00;
                                tx_msg.data[5] = 0x01;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- ACC Position --- */
                        /* --- NB! for some reason always returning inactive --- */
                        if (rx_msg.data[2] == 0xA4)
                        {
                            if (rx_msg.data[3] == 0x30)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x01;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- Accelerator Pedal Position [%] && Accelerator Pedal Position [V] --- */
                        if (rx_msg.data[2] == 0x13)
                        {
                            if (rx_msg.data[3] == 0x40)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x1F;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- Accelerator Pedal Possition Mode  --- */
                        if (rx_msg.data[2] == 0x11)
                        {
                            if (rx_msg.data[3] == 0x25)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x01;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- Air Conditioning Clutch --- */
                        if (rx_msg.data[2] == 0x11)
                        {
                            if (rx_msg.data[3] == 0x04)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = 0x01;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- ECT voltage value --- */
                        if (rx_msg.data[2] == 0x11)
                        {
                            if (rx_msg.data[3] == 0x4D)
                            {
                                tx_msg.data[0] = 0x5;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = ect_voltage & 0xFF;
                                tx_msg.data[5] = ect_voltage >> 8;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- ECT temperature value 0x28 = 0°C --- */
                        if (rx_msg.data[2] == 0x00)
                        {
                            if (rx_msg.data[3] == 0x05)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = ect_temperature + 40;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }

                        /* --- FAN speed ---- */
                        if (rx_msg.data[2] == 0x16)
                        {
                            if (rx_msg.data[3] == 0xAA)
                            {
                                tx_msg.data[0] = 0x4;
                                tx_msg.data[1] = rx_msg.data[1] + 0x40;
                                tx_msg.data[2] = rx_msg.data[2];
                                tx_msg.data[3] = rx_msg.data[3];
                                tx_msg.data[4] = engine_fan_speed;
                                tx_msg.data[5] = 0x00;
                                tx_msg.data[6] = 0x00;
                                tx_msg.data[7] = 0x00;
                                can_send_message(&tx_msg);
                            }
                        }
                    }

                    /* --- TROUBLE CODES --- */

                    if (rx_msg.data[1] == 0x18)
                    {
                        /* sending P0210 code */
                        tx_msg.data[0] = 0x05;
                        tx_msg.data[1] = rx_msg.data[1] + 0x40;
                        tx_msg.data[2] = 0x01;  // number of error codes
                        tx_msg.data[3] = 0x00;  // first part of error code
                        tx_msg.data[4] = 0x10;  // second part of error code ... the code is P0010
                        tx_msg.data[5] = 0xff;  // end of error codes 
                        tx_msg.data[6] = 0x00;
                        tx_msg.data[7] = 0x00;
                        can_send_message(&tx_msg);
                    }
                    /* --- END OF TROUBLE CODES --- */
                }
			}
		}
	}
	
	return 0;
}
