
#include <utility/utility.h>

#include "CtrlFlame.h"

#include "CtrlStanding.h"
#include "CtrlExercise.h"
#include "CtrlWalking.h"
#include "hardware_drivers/FlameIO.h"




// Global object implementations (the actual state objects)
CFlameController			gFlameController;

CFlame_StStartup			gFlame_StStartup;
CFlame_StIdle				gFlame_StIdle;
CFlame_StBeginShutdown		gFlame_StBeginShutdown;
CFlame_StPowerUpDrivers		gFlame_StPowerUpDrivers;
CFlame_StPowerDownDrivers	gFlame_StPowerDownDrivers;
CFlame_StStanding			gFlame_StStanding;
CFlame_StExercise			gFlame_StExercise;
CFlame_StWalking			gFlame_StWalking;



//****** CFlame Controller *******//

CFlameController::CFlameController()
{
	mShouldShutdown = false;
}


void CFlameController::Init()
{
  Transition(&gFlame_StStartup);
}

void CFlameController::Deinit()
{
}

void CFlameController::CallForShutdown()
{
	mShouldShutdown = true;
}

bool CFlameController::ShouldShutdown()
{
	return mShouldShutdown;
}




//***** CFlame Controller STATES ****//

void CFlame_StStartup::Init()
{
	logprintf("entered STARTUP mode.\n");
	if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE0 ) )
		logprintf("warning: RUN/STOP initially set to RUN, waiting for STOP.\n"); 

}

void CFlame_StStartup::Update()
{
	if ( !FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE0 ))
		Controller()->Transition(&gFlame_StIdle);

	// by default, make sure the motor power relay is always forced off
	FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_OFF );
	s.powered = 0;
}

void CFlame_StIdle::Init()
{
	logprintf("entered IDLE mode.\n toggle 1 up to power up drivers\n");
}

void CFlame_StIdle::Update()
{
	s.powered = 0; // turn off all motor driver enable flags
	s.LEDS &= ~LED0;

	// zero out all the torque values
	s.hipx.tau     = 0.0; 
	s.l().hipy.tau   = 0.0; 
	s.l().knee.tau   = 0.0; 
	s.l().ankley.tau = 0.0; 
	s.r().hipy.tau   = 0.0; 
	s.r().knee.tau   = 0.0; 
	s.r().ankley.tau = 0.0; 

	if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE0 ) )
		Controller()->Transition(&gFlame_StPowerUpDrivers);

	// by default, make sure the motor power relay is always forced off
	FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_OFF );

	// Shortcut when not powering the motor drivers (for testing)
	// Do not forget to comment the power down part in the cases themself!
	//Controller()->Transition(&gFlame_StExercise);
}

void CFlame_StBeginShutdown::Init()
{
}

void CFlame_StBeginShutdown::Update()
{
  //    // tell the monitor we are quitting
  //    send_signal ( rt_out_port, MSG_SHUTDOWN ); // moved to after state machine in Flame_core

    // zero out all the torque values
    s.hipx.tau     = 0.0; 
    s.l().hipy.tau   = 0.0; 
    s.l().knee.tau   = 0.0; 
    s.l().ankley.tau = 0.0; 
    s.r().hipy.tau   = 0.0; 
    s.r().knee.tau   = 0.0; 
    s.r().ankley.tau = 0.0; 

    // make sure motors are off
    s.powered = 0;
    
    // make sure the LEDS are off
    s.LEDS = 0;

    // make sure the motor power relay is always forced off, and the motor power on is not enabled
    FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_OFF );

    // this could do anything else to safely turn off the machine, or take some cycles to do so.

    // finish the shutdown, the loop will exit this cycle
    Controller()->CallForShutdown();
}

/*
void CFlame_StShutdown::Init()
{
}

void CFlame_StShutdown::Update()
{
}
*/

void CFlame_StPowerUpDrivers::Init()
{
      logprintf( "powering up drivers.\n\n Press second button to start EXERCISE mode...\nPress third button to go to standing\n" );
      s.powered = 0;  // no drivers enabled
      
      FlameIO_enable_motor_drivers( &io, s.powered );
      // moved to Update() due to potential occasional failing of dig output writing
      //FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_ON );
            
}

void CFlame_StPowerUpDrivers::Update()
{
	FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_ON );
	
	// zero out all the torque values
	s.hipx.tau     = 0.0; 
	s.l().hipy.tau   = 0.0; 
	s.l().knee.tau   = 0.0; 
	s.l().ankley.tau = 0.0; 
	s.r().hipy.tau   = 0.0; 
	s.r().knee.tau   = 0.0; 
	s.r().ankley.tau = 0.0; 

	// Wait a short time for the voltage to stabilize before continuing.  
	if ( Controller()->TimeElapsed() > 0.2 )
	{
		static int debounce = 0;
		//FlameIO_write_power_control_outputs( &io, 0 );

		if ( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON1 ))
		{
			debounce++;	
			if ( debounce = 5 )
				Controller()->Transition(&gFlame_StExercise);

		}

		if ( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON2 ))
		{
			debounce++;
			if ( debounce == 5 )
			{
				logprintf("recalibrating the motor encoder offsets.\n");
				// Now it is assumed that the robot did an exercise to find the indexpulses before with the computer still on. 
				// It knows its indexpositions and only needs to reset the motor positions equal to the joint positions. 

				// the motors can rotate more than once per rotation, so assume that the springs are relaxed and just null
				// the difference between the joint and motor encoders; this assumes that the joints have already been moved
				// to find the index pulses
				
				params.l().hipymot.q.offset    +=  s.l().hipy.q   - s.l().hipymot.q;
				params.l().kneemot.q.offset    +=  s.l().knee.q   - s.l().kneemot.q;
				params.l().ankleymot.q.offset  +=  s.l().ankley.q - s.l().ankleymot.q;
				params.r().hipymot.q.offset    +=  s.r().hipy.q   - s.r().hipymot.q;
				params.r().kneemot.q.offset    +=  s.r().knee.q   - s.r().kneemot.q;
				params.r().ankleymot.q.offset  +=  s.r().ankley.q - s.r().ankleymot.q;

				Controller()->Transition(&gFlame_StStanding);
			}
		}
	}

/*
#if 0
    // Wait for the voltage to stabilize before continuing.
    if ( c.mot_sw_filt > FLAME_STABLE_MOTOR_VOLTAGE )
	{
      logprintf( "stable motor voltage detected.\n");
      FlameIO_write_power_control_outputs( &io, 0 );
      if (FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON1 ))
	  { 
		mode_transition( MODE_DEMO );
      }
    }

    // A timeout on the power supply power up.
    if ( mode_time_elapsed() > 1.0 )
	{
      logprintf( "motor voltage failed to stabilize.\n");
      FlameIO_write_power_control_outputs( &io, 0 );
      mode_transition( MODE_POWER_DN_DRIVERS );
    }
#endif  
*/
}

void CFlame_StPowerDownDrivers::Init()
{
      logprintf( "powering down drivers.\n" );
      s.powered = 0;  // no drivers enabled
      FlameIO_enable_motor_drivers( &io, s.powered );
      // moved to Update() due to potential occasional failing of dig output writing
      //FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_OFF );
}

void CFlame_StPowerDownDrivers::Update()
{
	FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_OFF );
	
	// zero out all the torque values
	s.hipx.tau     = 0.0; 
	s.l().hipy.tau   = 0.0; 
	s.l().knee.tau   = 0.0; 
	s.l().ankley.tau = 0.0; 
	s.r().hipy.tau   = 0.0; 
	s.r().knee.tau   = 0.0; 
	s.r().ankley.tau = 0.0; 

	// This could wait for the voltages to stabilize near zero before continuing, but 
	// instead just enforces a short delay.
	if ( Controller()->TimeElapsed() > 0.2 )
	{
		// Leave the MOTOR_POWER_OFF output active to force the relay voltage to zero; this
		// compensates for the residual current through the power on drive transistor Q3.
		Controller()->Transition(&gFlame_StIdle);
	}
}

void CFlame_StStanding::Init()
{
      logprintf("entered STANDING mode.\n");
      // reset the controller state machine
      gStandingController.Transition(&gStanding_StSleep);
      s.LEDS |= LED0;
}

void CFlame_StStanding::Update()
{

	if ( gStandingController.IsInState(&gStanding_StStandControl) && ( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON1 )))
	{
		if ((s.l().foot.back.count > 100 || s.l().foot.front.count > 100) && (s.r().foot.front.count > 100 || s.r().foot.back.count > 100)
			)
		{
			Controller()->Transition(&gFlame_StWalking);
		}
	}

	//power down if the RUN/STOP switch goes to STOP
	if (!FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE0 ))
	{
		s.LEDS &= ~LED0;
		Controller()->Transition(&gFlame_StPowerDownDrivers);
	}
	else
		gStandingController.Update();
		//update_standing_controller(); // standing.c
}

void CFlame_StExercise::Init()
{
	logprintf("entered EXERCISE mode.\n");

	// reset the controller state machine
	gExerciseController.Init();//Transition(&gExercise_StBegin);
	s.LEDS |= LED0;
}

void CFlame_StExercise::Update()
{
	if (gExerciseController.IsInState(&gExercise_StEnd))
	{
		if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE2 ) )
			Controller()->Transition(&gFlame_StWalking);
		else
			Controller()->Transition(&gFlame_StStanding);
	}

	// power down if the RUN/STOP switch goes to STOP
	if ( !FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE0 ) )
	{
		s.LEDS &= ~LED0;
		Controller()->Transition(&gFlame_StPowerDownDrivers);
	}
	else
		gExerciseController.Update();
}

void CFlame_StWalking::Init()
{
	logprintf("entered walking mode.\n");
	s.LEDS |= LED0;         // turn on the "controller active" LED
	gWalkingController.Transition(&gWalking_StGetReady);
}

void CFlame_StWalking::Update()
{
	// power down if the RUN/STOP switch goes to STOP
	if ( !FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE0 ) )
	{
		s.LEDS &= ~LED0;
		Controller()->Transition(&gFlame_StPowerDownDrivers);
	}
	else
	{
		gWalkingController.Update();	// CtrlWalking.cpp

/*		// If a cable breaks, then stop the motors from turning or both sides of the cable will break
		if ((s.r().kneemot.q > 1.8) || (s.r().kneemot.q < -0.2))
			s.r().knee.tau = 0.0;

		if ((s.l().kneemot.q > 1.8) || (s.l().kneemot.q < -0.2))
			s.l().knee.tau = 0.0;

		if ((s.r().hipymot.q > 1.6) || (s.r().hipymot.q < -1.6))
			s.r().hipy.tau = 0.0;

		if ((s.l().hipymot.q > 1.6) || (s.l().hipymot.q < -1.6))
			s.l().hipy.tau = 0.0;
*/
	}
}

