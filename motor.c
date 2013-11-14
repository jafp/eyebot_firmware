

#include <stdint.h>
#include <avr/io.h>

#include "pwm.h"
#include "motor.h"

static void set_speed(uint8_t s_l, uint8_t s_r);
static void set_dir_left(int dir_l);
static void set_dir_right(int dir_r);

/**
 * Initialize the motor module.
 */
void motor_init()
{
	pwm_init();
}

/**
 * Set speed and direction for both motors. Speed is a number in the 
 * interval 0-255. Direction is a number, where 1 is forward, ...

 * \param speed_left Speed of left motor
 * \param speed_right
 * \param dir_left
 * \param dir_right 
 */
void motor_set(uint8_t speed_left, uint8_t speed_right, uint8_t dir_left, 
	uint8_t dir_right)
{
	set_speed(speed_left, speed_right);
    set_dir_left(dir_left);
    set_dir_right(dir_right);
}


static void set_speed(uint8_t speed_l, uint8_t speed_r)
{  
	OCR0 = speed_l;
 	OCR2 = speed_r;
}
 
static void set_dir_left(int direct_l){
  if (direct_l == 1){  // moving forward    
    PORTB |= (1<<0);
    PORTB &= ~(1<<1);
  
   
  }

  else if (direct_l == 2){
    PORTB |= (1<<1);
    PORTB &= ~(1<<0);
  }

  else if (direct_l == 0){
    PORTB |= (1<<1) | (1<<0);
  }
}

static void set_dir_right(int direct_r)
{
  if (direct_r == 1){  // moving back
    PORTB |= (1<<4);
    PORTB &= ~(1<<2);
  }
  else if (direct_r == 2){
    PORTB |= (1<<2);
    PORTB &= ~(1<<4);
  }
  else if (direct_r == 0){
    PORTB |= (1<<2) | (1<<4);
 }
}



