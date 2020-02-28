/**
 * @file MiP_final
 *
 * This program calculates the angle that the MIP is currently at
 *  with a starting point at the vertical axis.
 * 
 * Once the angle is found, then it implements the controller connecting
 * the PWM duty cycle to the MiP orientation.
 * This calculation is inside the fast loop as corrections for the MiP body
 * angle is volatile. Frequency for the loop is set at 100Hz.
 * 
 * Once the body is stabilized, the next controller needed is to stabilize 
 * the position of the angle. This means that the wheels need to be stabilized.
 * For this controller, since it's not as important as the body angle volatility,
 * is a slower controller at 20Hz. 
 * 
 * 
 * 
 */
#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems


// The time constant is set to slightly favor the accelerometer 
// as 1/1.7>0.5 and favoring the low pass filter
#define timeconstant 1.7  //time constant set as 1.7;
#define dt 0.01 //dt is 0.01s since it's 100Hz


// function declarations
void on_pause_press();
void on_pause_release();

//interrupt declaration
void theta_calculations(void);
//pthread declaration
void* slow_loop(void* reference);

// function declarations for sensor readings
// gyroscope filter
float highpass(float gain,float input);
//accelerometer filter
float lowpass(float gain,float input);


/**
 * This template contains these critical components
 * - start the signal handler
 * - initialize subsystems
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 * - This module incorporates pthreads & interrupts
 * - Interrupt to do the calculations
 * - pthread with lower priority for slower loop
 * @return    0 during normal operation, -1 on error
 */


//initialie the variables

// for the low-pass filter difference equation
float low_oldoutput=0;
// for the high pass filter difference equation
float high_oldoutput = 0;
float high_oldinput = 0;


// Variables for the calculation of theta
float theta_a_raw; 
float theta_a; //theta_a after filter

float theta_g_raw;
float theta_g_raw_old = 0; //needed because it's an integration formula
float theta_g; //theta_g after filter

//theta_f = theta_a + theta_g 
float theta_f; 
float theta_f_old;


//output for D1
float duty_old = 0;
float duty;

// old variables for D2 & D1 controller
float theta_diff_old;
float inputd2_old2;
float outputd2_old2;
float inputd2_old1;
float outputd2_old1;

//left motor in radians
float L_rad=0;
//right motor in radians
float R_rad =0;
// average left and right motor
float phi = 0;
//output of the outer loop controller
float theta_ref = 0;

// initialize the conversion factor c
// ppr = 35.577 * 15 * 4 = 2134.62
// c = 2*pi/ppr = 0.0029434678
const float c = (2*3.141592)/(35.577 * 15 * 4); 
static float k1 = 1;
static float k2 = 0.6;

//function declarations for slow & fast controller
float d1(float theta_diff);
float d2(float phi_diff);

float gain = dt/(timeconstant+dt); //define the gain
float wc = 1/timeconstant;


// Preload a file for csv
char filename[] = "Justin";
FILE *filepointer; 

rc_mpu_data_t data; //struct to hold new data

int main()
{   
	//open the file needed to be exported
	filepointer=fopen(strcat(filename,".csv"),"a"); //opens a file for reading and appending

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
			fprintf(stderr,"ERROR: failed to start signal handler\n");
			return -1;
	}

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
											RC_BTN_DEBOUNCE_DEFAULT_US)){
			fprintf(stderr,"ERROR: failed to initialize pause button\n");
			return -1;
	}
	
	// initialize hardware 
	rc_mpu_config_t conf = rc_mpu_default_config();

	//initialize mpu with dmp
	if(rc_mpu_initialize_dmp(&data, conf)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	

    // Assign functions to be called when button events occur
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

    //Initialize motors
    rc_motor_init();
    //initialize encoders
    rc_encoder_eqep_init();

	//interrupt functions
	rc_mpu_set_dmp_callback (&theta_calculations);

	//create thread
	pthread_t slow_thread = 0;
	
	//initialize the pthreads
	if(rc_pthread_create(&slow_thread, slow_loop, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start thread\n");
		return -1;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	printf("\nDone with initializing\n");

	printf("Hold the pause button for 2 seconds to exit\n");
    //set initial state to running
	rc_set_state(RUNNING);

	// run the !EXITING state loop inside main 
	while(rc_get_state()!=EXITING){
                //run green LED if RUNNING
                if(rc_get_state()==RUNNING){ 
                        rc_led_set(RC_LED_GREEN, 1);
                        rc_led_set(RC_LED_RED, 0);
                }
                else{
                // run red LED if PAUSED
                        rc_led_set(RC_LED_GREEN, 0);
                        rc_led_set(RC_LED_RED, 1);
                }
                rc_usleep(100000);
                // always sleep at some point
	rc_usleep(10000); //100 Hz
	}
	
	//join threads
	rc_pthread_timed_join(slow_loop,NULL,1.5);
        //turn off LEDs
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
	//close file 
	fclose(filepointer);
	// close file descriptors
        //clean up LED
        rc_led_cleanup();
	rc_cleanup();
	rc_mpu_power_off();
	rc_button_cleanup();  // stop button handlers
	rc_remove_pid_file(); //remove pid file LAST
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{		//toggle from running to pause
        if(rc_get_state()==RUNNING){
			rc_set_state(PAUSED);
		}
		//toggle from pause to running
        else if(rc_get_state()==PAUSED){
			rc_set_state(RUNNING);
		}
        return;
}
/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
        int i;
        const int samples = 100; // check for release 100 times in this period
        const int us_wait = 2000000; // 2 seconds
        // now keep checking to see if the button is still held down
        for(i=0;i<samples;i++){
                rc_usleep(us_wait/samples);
                if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
        }
        printf("long press detected, shutting down\n");
        rc_set_state(EXITING);
        return;
}


//Low pass filter difference equation
float lowpass(float gain,float input){
		//difference equation for a low pass filter 
        float output = gain*input + (1-gain)*low_oldoutput;
        low_oldoutput = output;
        return output;
}

//High pass filter difference equation
float highpass(float gain,float input){
		//difference equation for a high pass filter 
        float output = gain*(high_oldoutput + input - high_oldinput); 
        high_oldoutput = output;
        high_oldinput = input;
        return output;
}



/**
 * This function calculations inside the fast loop (interruption)
 * averages out the gyroscope and accelerameter reading
 * After it finds a reading, then apply the controller
 * to find what the duty cycle is.
 * This function also applies the duty cycle to the motors 
 * to balance the MiP body angle
 * 
 * If the angle is over 1 radian, then give up the program
 * 
 */
void theta_calculations(void)
{
	// read sensor data with accelerometer
	if(rc_mpu_read_accel(&data)<0)
	{
		printf("read accel data failed\n");
	}
	// read sensor data with gyroscope
	if(rc_mpu_read_gyro(&data)<0)
	{
		printf("read gyro data failed\n");
	}
	
	//collect data for theta_a_raw
	theta_a_raw= atan2(-data.accel[2],data.accel[1]);
	//collect data for theta_g_raw from euler approximation
	//0.01 because that's the delta T for 100Hz
	theta_g_raw = theta_g_raw + (data.gyro[0]*DEG_TO_RAD*0.01); 
	//update the old as the new
	theta_g_raw_old = theta_g_raw;
	//low pass filter for theta_a
	theta_a = lowpass(gain,theta_a_raw);
	//high pass filter for theta_g
	theta_g = highpass(1-gain,theta_g_raw);
	//theta_f is the sum plus the offset for the way MiP naturally balances
        // At this offset angle is the MiP should naturally stand still
	theta_f = theta_a + theta_g+0.25;
        //Apply the fast controller with loop prefactor 1/1.48
        duty = d1(theta_ref/1.48 - theta_f);

        //set the position of the motors
        // one is CW the other is CCW
        rc_motor_set(2,-duty);
        rc_motor_set(3,duty);

        // if the MiP body angle is over +/- (1 radian = 57 degrees)
        // then just give up 
        if(theta_f>1)
        {
            rc_set_state(EXITING);
            return;
        }
        if(theta_f<-1)
        {
            rc_set_state(EXITING);
            return;
        }


}

/**
 * This function calculations inside the slow loop 
 * It gets the encoder position (feed back values of x(t))
 * Then it applies the controller to find the theta reference for
 * the fast controller and to stabilize G1(s)
 * 
 */
void* slow_loop(__attribute__ ((unused)) void* reference)
{
 while(rc_get_state()!=EXITING)
	{
                //set the encoder positions to zero initially
                rc_set_encoder_pos(2,0);
                rc_set_encoder_pos(3,0);
                //convert the encoder to radians
                L_rad = rc_get_encoder_pos(2)*c;
                R_rad = rc_get_encoder_pos(3)*c;
                //averges the left and right encoder readings
                //and make it absolute as the readings are 
                //dependent on the MiP body angle
                phi = (L_rad+R_rad)/2 - theta_f;
                theta_ref = d2(phi);
		usleep(200000); //set frequency to 20Hz
	}
}


/**
 * This function is the fast loop controller
 * input: difference in theta_reading & theta_ref
 * output: duty cycle for the motors
 * D1(s) = -(s+8)/s
 * Using Tustin's Approximation
 * D1(z) = (-1.04z + 0.96)/(z-1)
 */
float d1(float theta_diff)
{
    float output = duty_old +k1*(-1.04*theta_diff + 0.96*theta_diff_old);
    duty_old = output;
    theta_diff_old = theta_diff;
    return output;
}

/**
 * This function is the slow loop controller
 * input: difference in phi and phi ref (which is zero)
 * output: theta reference for the fast loop controller
 * 
 *  D2(s) = s/(s+13)(s+7)
 * Using Tustin's Approximation
 * D2(z) = (0.01606z^2-0.01606)/(z^2-1.212z+0.3577)
 */


float d2(float phi_diff)
{
    float output = 1.212*outputd2_old1 -0.3577*outputd2_old2 + k2*(0.01606*phi_diff - 0.01606*inputd2_old2);
    outputd2_old2 = outputd2_old1;
	outputd2_old1 = output;
	inputd2_old2 = inputd2_old1;
	inputd2_old1 = phi;
    return output;
}