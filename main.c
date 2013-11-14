
#define F_CPU     16000000

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "init.h"
#include "motor.h"


#define ADDR      0x23 

/**
 *
 */
typedef struct command_request {
  unsigned char current_round;
  void * data;
} command_request_t;

/**
 * 
 */
typedef struct command {
  // Unique command id
  unsigned char id;
  // Function called when this command is requested
  void (* init)(command_request_t * req);
  // Function called when the master wants to read from this device.
  // The function should return the number of bytes left to be read by
  // the master.
  char (* read)(command_request_t * req, unsigned char * buffer);
  // Function called then the master wants to write to this device
  char (* write)(command_request_t * req, unsigned char * buffer);
} command_t;

/**
 * Global variables
 */

volatile uint8_t count = 0;
volatile uint8_t pulse1 = 0;
volatile uint8_t pulse2 = 0;
volatile uint8_t last_pulse_l = 0;
volatile uint8_t last_pulse_r = 0;
volatile uint8_t ref_l = 0;
volatile uint8_t ref_r = 0;
volatile uint8_t dir_l = 2, dir_r = 2;
volatile uint8_t k_p_left = 2;
volatile uint8_t k_p_right = 2;

/**
 * Function prototypes
 */

uint8_t get_speed(uint8_t encoder, uint8_t ref, uint8_t kp);

static char cmd_get_speed_read(command_request_t * req, 
  unsigned char * buffer);

static char cmd_set_speed_write(command_request_t * req, 
	unsigned char * buffer);

static char cmd_set_dir(command_request_t * req, 
	unsigned char * buffer);

static char cmd_set_ctrl_consts_write(command_request_t * req,
	unsigned char * buffer);


/**
 * Array of available commands and their definitions
 */
command_t commands[] = {
	{
		.id = 0x10,
		.init = NULL,
		.read = cmd_get_speed_read,
		.write = NULL
	},
	{
		.id = 0x20,
		.init = NULL,
		.read = NULL,
		.write = cmd_set_speed_write
	},
	{
		.id = 0x30,
		.init = NULL,
		.read = NULL,
		.write = cmd_set_dir
  	},
	{
		.id = 0x40,
		.init = NULL,
		.read = NULL,
		.write = cmd_set_ctrl_consts_write
  	}
};

/**
 * Number of commands
 */
unsigned char n_commands = sizeof(commands) / sizeof(command_t);


/**
 *
 * Interrupt Service Routines
 *
 */

ISR(INT0_vect)
{  // interrupt 0 from motor encoder A
	pulse1++;    
}

ISR(INT1_vect)
{  	// interrupt 1 from motor encoder B
	pulse2++;
}

ISR(TIMER1_OVF_vect)
{
   //This is the interrupt service routine for TIMER0 OVERFLOW Interrupt.
   //CPU automatically call this when TIMER0 overflows.
	count++;
}


ISR(TWI_vect)
{
  	static unsigned char i2c_state;
	static command_t * cmd;
  	static command_request_t req;
    
    unsigned char twi_status;
    unsigned char data;

    // Disable Global Interrupt
    cli();
    // Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
    twi_status = TWSR & 0xF8;     

    switch(twi_status) 
    {
      	case TW_SR_SLA_ACK:      // 0x60: SLA+W received, ACK returned
          i2c_state=0;           // Start I2C State for Register Address required  

          TWCR |= (1<<TWINT);    // Clear TWINT Flag
          break;

      	case TW_SR_DATA_ACK:     // 0x80: data received, ACK returned
          	if (i2c_state == 0) 
          	{
	            data = TWDR;      // Save data to the register address
	            
	            switch(data)
	            {
	           		case 0x10:
	           		{
	           			cmd = &commands[0];
	           			break;
	           		}
					case 0x20:
		            {
		            	cmd = &commands[1];
						break;
		            }
		            case 0x30:
		            {
		            	cmd = &commands[2];
		            	break;
		            }
		            case 0x40:
		            {
		            	cmd = &commands[3];
		            	break;
		            }
		            default:
		            {
		           		cmd = NULL;
		            }
            	}
	            //cmd = cmd_find(data);
	            if (cmd != NULL)
	            {
		            // Command found.
		            // Reset round counter and possibly call the commands
		            // initialization function
		            req.current_round = 0;
		            if (cmd->init != NULL)
		            {
		            cmd->init(&req);
		            }

		            i2c_state = 1;  
	            }
	        } 
	        else 
	        {
	            data = TWDR;
	            if (cmd->write != NULL)
	            {
	            	if (cmd->write(&req, &data) == 0)
	            	{
	            		i2c_state = 0;
	            	}
	            }
	            req.current_round++;

	        	//regdata = TWDR;      // Save to the register data
	        	//i2c_state = 2;
	        }

          	TWCR |= (1<<TWINT);    // Clear TWINT Flag
          	break;

      	case TW_SR_STOP:         // 0xA0: stop or repeated start condition received while selected
        	TWCR |= (1<<TWINT);    // Clear TWINT Flag
        	break;

      case TW_ST_SLA_ACK:      // 0xA8: SLA+R received, ACK returned
      case TW_ST_DATA_ACK:     // 0xB8: data transmitted, ACK received
          if (i2c_state == 1) 
          {
            if (cmd->read != NULL)
            { 
              // If the read function returns zero (which means no more bytes are left to be read),
              // then reset i2c state to 0
              if (cmd->read(&req, &data) == 0)
              { 
                i2c_state = 0;
              }
              TWDR = data;
            }

            req.current_round++;

        //i2c_slave_action(0); // Call Read I2C Action (rw_status = 0)
        //TWDR = regdata;      // Store data in TWDR register
        //i2c_state = 0;        // Reset I2C State
          }       

          TWCR |= (1<<TWINT);    // Clear TWINT Flag
          break;

      case TW_ST_DATA_NACK:    // 0xC0: data transmitted, NACK received
      case TW_ST_LAST_DATA:    // 0xC8: last data byte transmitted, ACK received
      case TW_BUS_ERROR:       // 0x00: illegal start or stop condition
      default:
          TWCR |= (1<<TWINT);    // Clear TWINT Flag
          i2c_state = 0;         // Back to the Begining State
    }
    // Enable Global Interrupt
    sei();
}


/**
 * I2C command for updating the constants (parameters) for the
 * P-controller.
 *
 * \param req
 * \param buffer
 */
static char cmd_set_ctrl_consts_write(command_request_t * req,
	unsigned char * buffer)
{
	if (req->current_round == 0)
	{
		k_p_left = buffer[0];
	}
	else
	{
		k_p_right = buffer[0];
	}
	return req->current_round == 1 ? 0 : 1;
}

/**
 * I2C command to read the latest tachometer measurement.
 *
 * \param req
 * \param buffer
 */
static char cmd_get_speed_read(command_request_t * req, 
	unsigned char * buffer)
{
	if (req->current_round == 0)
	{
		buffer[0] = last_pulse_l;
	}
	else
	{
		buffer[0] = last_pulse_r;
	}
	return req->current_round == 1 ? 0 : 1;
}

/**
 * I2C command to update the speed of the motors.
 *
 * \param req
 * \param buffer
 */
static char cmd_set_speed_write(command_request_t * req, 
	unsigned char * buffer)
{
	if(req->current_round == 0)
	{
		ref_l = buffer[0];
	} 
	else
	{
		ref_r = buffer[0];
	} 
	return req->current_round == 1 ? 0 : 1;
}

/**
 * I2C command for setting the direction of the motors.
 *
 * \param req
 * \param buffer
 */
static char cmd_set_dir(command_request_t * req, 
	unsigned char * buffer)
{
	if(buffer[0] == 0x10)
	{
		dir_l = 0;  
		dir_r = 0;
	}
	else if(buffer[0] == 0x20)
	{
		dir_l = 1;
		dir_r = 1;
	}
	else if(buffer[0] == 0x30)
	{
		dir_l = 2;
		dir_r = 2;
	}
	return 0;
}


/**
 * Find a command by id
 *
 * \param id Command identifier
 * \return Pointer to command, or null if no such command exists
 */
/*
static command_t * cmd_find(unsigned char id)
{
  unsigned char i;
  for (i = 0; i < n_commands; i++)
  {
    if (commands[i].id == id)
    {
      return &commands[i];
    }
  }
  return NULL;
}
*/

/**
 * Calculates the speed for a motor, given the current tachometer
 * measurement, a reference and parameters for the P-controller.
 *
 * \param encoder
 * \param ref
 * \param kp
 */
uint8_t calculate_speed(uint8_t encoder, uint8_t ref, uint8_t kp)
{
	int16_t ke = 2; 
	int16_t pwm_tmp = 0; 
	int16_t error = 0; 
	int16_t encoder_sig = 0; 
	int16_t p_error = 0; 
	uint8_t pwm = 0; 

	encoder_sig = ke * encoder;
	error = ref - encoder_sig;
	p_error = error * kp;
	pwm_tmp = ref + p_error;

	if (pwm_tmp < 0)
	{
		pwm = 0;
	}
	else if (pwm_tmp > 255)
	{
		pwm = 255;
	}
	else 
	{
		pwm = (uint8_t) pwm_tmp;
	}

	return pwm; 
}



int main(void)
{
	uint8_t speed1 = ref_l, speed2 = ref_r;

   	motor_init();
   	int_init();
    	
    // TODO Make i2c_init() ...

    // Initial I2C Slave
  	TWAR = (ADDR<<1);// & 0xFE; // Set I2C Address, Ignore I2C General Address 0x00
	// Default Initial Value
  	// Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable
  	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
  	// Enable Global Interrupt
  	sei();

  	// Enable watch dog to trigger every 30 ms.
  	// Useful in case something goes terribly wrong.
  	wdt_enable(WDTO_30MS);

	while(1)
	{
		if(count == 5) 
		{
			wdt_reset();

			speed1 = calculate_speed(pulse1, ref_l, k_p_left);
			speed2 = calculate_speed(pulse2, ref_r, k_p_right);

			last_pulse_l = pulse1;
			last_pulse_r = pulse2;

			pulse1 = pulse2 = count = 0;
	
			motor_set(speed1, speed2, dir_l, dir_r);
		}
	}    
}
