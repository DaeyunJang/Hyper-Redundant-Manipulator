/**
*	@brief		This is a testprogram which demonstrates the use of the EPOS4 SDK
*	@detail		The drive will run back and forth when the program is started. It will also
*				record some test data. This data can be displayed by pressing the "Oscilloscope(Record)"
*				button after the program has run.
*
*	$Revision: 30 $
*
*	@example 	ECAT_3Ax_EPOS4-Test_csp.mc
*
*/
//#include "ApossC_SDK_V01.05\SDK\SDK_ApossC.mc"
#include "Impec_210409.mh" // Header file to include
dim long MOTOR_Write_Array[100]; // Array at index 0x2100
dim long MOTOR_Read_Array[100];  // Array at index 0x2101


#define C_AXIS1	0						// Axis module number: axis 1 is module 0, used by setup and motion functions.
#define C_AXIS2 1                       // Axis module number: axis 2 is module 1, used by setup and motion functions.

#define C_DRIVE_BUSID1 1000001			// EtherCAT network ID: axis 1 starts at 1000001.
#define C_DRIVE_BUSID2 1000002          // EtherCAT network ID: axis 2 uses 1000002.

#define C_ENC_QC_1 512*4               // Axis 1 encoder resolution: the 512-count encoder is quadrature-decoded to 512 * 4 = 2048 counts per motor revolution.
#define C_ENC_QC_2 512*4               // Axis 2 uses the same encoder specifications as axis 1.

#define C_EC_CYCLE_TIME	1				// EtherCAT cycle time: 1 ms. This setting does not need to be changed.
#define C_EC_OFFSET		1				// EtherCAT shift offset. This setting does not need to be changed.

#define C_PDO_NUMBER	1				// PDO number used. This setting does not need to be changed.
#define C_OP_MODE_CSP   1               // Argument selecting EPOS4 CSP operation mode in the setup functions below

#define C_MOTOR_MAX_RPM		6600		// Maximum velocity in RPM

#define C_AXISPOLARITY		1			// Definition of the polarity 0: Normal, 1: Inverse //

#define VEL_ACC 30 // Acceleration used by motion functions, in percent
#define VEL_DEC 30 // Deceleration used by motion functions, in percent

#define PAUSETIME 3000 // Test pause time; the application should obtain this value from the array below.

#define SMDATA_SAVEARRAY      0
#define POSREACHED 3
//////////////////////////////////////////////////
/////////////////Writing Array////////////////////
//////////////////////////////////////////////////
#define MOTOR1_MODE Sysvar[0x01210005]           // Motor 1 mode: 0 = continuous, 1 = quantity
#define MOTOR1_SPEED Sysvar[0x01210006]          // Motor 1 speed input
#define MOTOR1_SUCKBACK_DEGREE Sysvar[0x01210007]// Motor 1 suck-back angle input
#define MOTOR1_PAUSETIME Sysvar[0x01210008]      // Motor 1 pause time before suck-back
#define MOTOR1_QUANTITY_DEGREE Sysvar[0x01210009]// Motor 1 rotation-angle input for quantity mode

#define MOTOR2_MODE Sysvar[0x0121000A]           // Motor 2 mode: 0 = continuous, 1 = quantity
#define MOTOR2_SPEED Sysvar[0x0121000B]          // Motor 2 speed input
#define MOTOR2_SUCKBACK_DEGREE Sysvar[0x0121000C]// Motor 2 suck-back angle input
#define MOTOR2_PAUSETIME Sysvar[0x0121000D]      // Motor 2 pause time before suck-back
#define MOTOR2_QUANTITY_DEGREE Sysvar[0x0121000E]// Motor 2 rotation-angle input for quantity mode

#define Error Sysvar[0x0121000F]                 // Enter 1 to reset errors.

#define MOTOR1_SUCKBACK_LIMIT Sysvar[0x01210010] // Motor 1 suck-back angle limit: with a limit of 360, inputs above 360 are clamped to the limit.
#define MOTOR2_SUCKBACK_LIMIT Sysvar[0x01210011] // Motor 2 suck-back angle limit input

#define SAVEDATAROM Sysvar[0x01210012]           // Enter 1 to save data.

///////////////////////////////////////////////////
////////////////Reading Array//////////////////////
///////////////////////////////////////////////////
#define MOTOR1_DONE Sysvar[0x01210105].i1        // Motor 1 completion acknowledgement; bit indices start at 0.
#define MOTOR2_DONE Sysvar[0x01210105].i2        // Motor 2 completion acknowledgement
#define MOTOR1_Actual_SPEED Sysvar[0x01210106]   // Monitor the actual motor 1 velocity.
#define MOTOR2_Actual_SPEED Sysvar[0x01210107]   // Monitor the actual motor 2 velocity.
#define MOTOR1_ZUB_ERROR Sysvar[0x01210108]      // Zub controller error code for the motor 1 state machine
#define MOTOR1_EPOS4_ERROR Sysvar[0x01210109]    // EPOS4 error code for the motor 1 state machine
#define MOTOR2_ZUB_ERROR Sysvar[0x0121010A]      // Zub controller error code for motor 2, as above
#define MOTOR2_EPOS4_ERROR Sysvar[0x0121010B]    // EPOS4 error code for the motor 2 state machine
#define EtherCAT_Slave_Count Sysvar[0x01210105].ub2 // Number of EtherCAT slaves (EPOS4 drives) connected to the Zub controller

#define ErrorReset_Mask 1                           // Mask for the ErrorReset event
#define Save_Mask 1                                 // Mask for the save event

double NumberOfTurn,NumberOfTurn_QUANTITY,MOTOR1_RPM,NumberOfTurn_MOTOR2,NumberOfTurn_QUANTITY_MOTOR2,MOTOR2_RPM ; // Declare a floating-point variable.

long GearRatio,i,res,slaveCount,DI1,DI2; // Declare a 32-bit integer variable.

#pragma SmConfig {1,       // runflag: choose 1 or 0 according to whether Delay is used.
                  25,      // Number of events
                  6,       // Number of timers
                  1,
                  12,
                  0,
                  2}



SmEvent SIG_TARGET_REACHED {}                       // Motor 1 position-reached event
SmEvent SIG_TARGET_REACHED_2 {}                     // Motor 2 position-reached event
SmEvent TickPeriod {PRM_CLOCK}                      // Timer named TickPeriod in the motor 1 state machine
SmEvent TickPeriod_1 {PRM_CLOCK}                    // Timer named TickPeriod_1 in the motor 2 state machine
SmEvent TickSingle_DisCharge_Delay { PRM_CLOCK }    // Timer named TickSingle_DisCharge_Delay in the motor 1 state machine
SmEvent TickSingle_DisCharge_Delay_1 { PRM_CLOCK }  // Timer named TickSingle_DisCharge_Delay_1 in the motor 2 state machine
SmEvent Input1Rising  { PRM_NUMBER, PRM_VALUE }     // Digital input 1 rising-edge event to start motor 1; uses PRM_NUMBER and PRM_VALUE.
SmEvent Input1Falling  { PRM_NUMBER, PRM_VALUE }    // Digital input 1 falling-edge event to stop motor 1
SmEvent Input2Rising { PRM_NUMBER, PRM_VALUE }      // Digital input 2 rising-edge event to start motor 2
SmEvent Input2Falling { PRM_NUMBER, PRM_VALUE }     // Digital input 2 falling-edge event to stop motor 2
SmEvent Error_Reset {}                              // Error-reset event
SmEvent SAVEARRAY {}                                // Save-array event

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Motor1 State Machine//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SmState Motor1 {
               SIG_INIT = {
                            SmSubscribe(id, SIG_ERROR); // Error-handling state machine

                            SmParam (0x0125007B, SM_STAT_POSREACHED, SM_PARAM_RISING, id, SIG_TARGET_REACHED); // Motor 1 position-reached event: SIG_TARGET_REACHED; uses SM_STAT.
                            SmInput(1, SM_INPUT_RISING,  id, Input1Rising);                                    // Motor 1 digital input 1 rising-edge event: Input1Rising
                            SmInput(1, SM_INPUT_FALLING,  id, Input1Falling);                                  // Motor 1 digital input 1 falling-edge event: Input1Falling
                            SmParam (0x01210012, Save_Mask, SM_PARAM_EQUAL, id, SAVEARRAY);                    // Trigger SAVEARRAY when index 0x2100, subindex 0x12 matches the mask declared above.
                            SmParam (0x0121000F, ErrorReset_Mask, SM_PARAM_EQUAL, id,Error_Reset);             // Trigger Error_Reset when index 0x2100, subindex 0x0F matches the mask declared above.
                            SmPeriod (100, id, TickPeriod);                                                    // Timer event named TickPeriod


                            return(SmTrans(->Standing)); // After registering the events, transition to Standing.
                            }
                            SIG_ERROR = {
                                          res = ErrorAxis();

                                          if (res == 0)
                                          {
                                             MOTOR1_ZUB_ERROR = ErrorNo();               // Read the Zub controller error with ErrorNo() and store it in MOTOR1_ZUB_ERROR.
                                             MOTOR1_EPOS4_ERROR = Sysvar[0x014C0005];    // Copy the motor 1 EPOS4 error from index 0x4C00, subindex 0x05 to MOTOR1_EPOS4_ERROR.
                                                 if(ErrorNo() == F_POSERR)
                                                 {
                                                    print ("track error limit reached");
                                                    return(SmTrans(->Standing));
                                                 }
                                                 if(ErrorNo() == F_AMP)
                                                 {
                                                    print ("Axis Error");
                                                    return(SmTrans(->Standing));
                                                 }
                                                 if(ErrorNo() == F_ECAT_MASTER )
                                                 {
											        if(data[SMDATA_SAVEARRAY]==1)
												    {
												        print ("EtherCAT Error Save Array");	//$B
												        ErrorClear();
												        data[SMDATA_SAVEARRAY]  = 0;
                                          		    }
                                         		     else
												    {
													print ("EtherCAT Error");
													ErrorClear();
													return(SmTrans(->Standing));
												    }
                                                 }
                                          }
                                         }
                   SmState Standing

                             {
                             SIG_ENTRY = { // State-machine entry handler
                                            print ("Go Ready ", id);
                                            Sysvar[0x01210012] = 0;

                                         }
                                         SAVEARRAY = Save_Data; // Call Save_Data when SAVEARRAY occurs.
                             Error_Reset = Error__RESET;        // Call Error__RESET when Error_Reset occurs.
                             Input1Rising = DISCHARGE_START;    // Call DISCHARGE_START when Input1Rising occurs.

                             TickPeriod = {   // 100 ms timer loop, equivalent to a periodic thread, running until one of the events above occurs.
                                              GearRatio = 44.0;
                                              MOTOR1_RPM = GearRatio * (MOTOR1_SPEED / (C_MOTOR_MAX_RPM / 100.0)); // Use MOTOR1_SPEED instead of 1.
                                              NumberOfTurn = MOTOR1_SUCKBACK_DEGREE * (C_ENC_QC_1 / 360.0); // Use MOTOR1_SUCKBACK_DEGREE instead of 100.
                                              NumberOfTurn_QUANTITY = MOTOR1_QUANTITY_DEGREE * (C_ENC_QC_1 / 360.0);// Use MOTOR1_QUANTITY_DEGREE instead of 300.
                                              MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                              if ( MOTOR1_SUCKBACK_DEGREE > MOTOR1_SUCKBACK_LIMIT)// Check the suck-back angle against its configured limit.
                                              {
                                                 MOTOR1_SUCKBACK_DEGREE = MOTOR1_SUCKBACK_LIMIT;
                                              }
                                              DI1 = DigInput(1); // Read the DI1 state using DigInput(1).
                                          }
                              }

                   SmState DISCHARGE_PROGRESS_CONTINUOUS { //State machine for the initial continuous-mode sequence

                              SIG_ENTRY = {  print ("DISCHARGE Continuous PROGRESS");}

                              Input1Falling = DISCHARGE_DONE; // Transition to DISCHARGE_DONE when digital input 1 switches off.
                                TickPeriod = { // Continuously monitor DI1 and actual motor velocity in the TickPeriod loop.
                                              DI1 = DigInput(1);
                                              MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                              }
                                              }


                   SmState DISCHARGE_PROGRESS_QUANTITY {   //State machine for the initial quantity-mode sequence


                              SIG_ENTRY = {  print ("DISCHARGE Quantity PROGRESS");}

                              SIG_TARGET_REACHED = DISCHARGE_DONE; // Transition to DISCHARGE_DONE when the position-reached event occurs.

                               TickPeriod = {DI1 = DigInput(1);
                                             MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                             }

                                                       }

                   SmState PAUSE_TIME {

                               SIG_ENTRY = {  print ("PAUSE TIME");

                                            }

                               TickPeriod = {DI1 = DigInput(1);
                                            MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                            }
                               TickSingle_DisCharge_Delay = START_SUCKBACK; //Call START_SUCKBACK when TickSingle_DisCharge_Delay occurs.

                                       }

                   SmState SUCKBACK_PROGRESS {
                                SIG_ENTRY = {  print ("SUCKBACK Done"); }

                                TickPeriod = {DI1 = DigInput(1);
                                             MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                             }
                                SIG_TARGET_REACHED = SUCKBACK_DONE; //Transition to SUCKBACK_DONE when suck-back positioning is complete.
                                             }



                                    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Motor2 State Machine//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SmState Motor2 {
               SIG_INIT = {
                            SmSubscribe(id, SIG_ERROR);


                            SmParam (0x0125017B, SM_STAT_POSREACHED, SM_PARAM_RISING, id, SIG_TARGET_REACHED_2);
                            SmInput(2, SM_INPUT_RISING,  id, Input2Rising);
                            SmInput(2, SM_INPUT_FALLING,  id, Input2Falling);
                            SmPeriod (100, id, TickPeriod_1);


                            return(SmTrans(->Standing_MOTOR2));
                            }


                             SIG_ERROR = {
                                          res = ErrorAxis();

                                          if (res == 1)
                                          {
                                             MOTOR2_ZUB_ERROR = ErrorNo();
                                             MOTOR2_EPOS4_ERROR = Sysvar[0x014C0105];
                                                 if(ErrorNo() == F_POSERR)
                                                 {
                                                    print ("track error limit reached");
                                                    return(SmTrans(->Standing_MOTOR2));
                                                 }
                                                 if(ErrorNo() == F_AMP)
                                                 {
                                                    print ("Axis Error");
                                                    return(SmTrans(->Standing_MOTOR2));
                                                 }

                                          }
                                         }




                   SmState Standing_MOTOR2

                             {
                             SIG_ENTRY = {
                                            print ("Go Ready ", id);
                                            Delay(500);
                                         }

                             Input2Rising = DISCHARGE_START_MOTOR2;
                             TickPeriod_1 = {
                                              GearRatio = 44.0;
                                              MOTOR2_RPM = GearRatio * (MOTOR2_SPEED / (C_MOTOR_MAX_RPM / 100.0)); // Use MOTOR1_SPEED instead of 1.
                                              NumberOfTurn_MOTOR2 = MOTOR2_SUCKBACK_DEGREE * (C_ENC_QC_2 / 360.0); // Use MOTOR1_SUCKBACK_DEGREE instead of 100.
                                              NumberOfTurn_QUANTITY_MOTOR2 = MOTOR2_QUANTITY_DEGREE * (C_ENC_QC_2 / 360.0);// Use MOTOR1_QUANTITY_DEGREE instead of 300.
                                              MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                              if ( MOTOR2_SUCKBACK_DEGREE > MOTOR2_SUCKBACK_LIMIT)
                                              {
                                                 MOTOR2_SUCKBACK_DEGREE = MOTOR2_SUCKBACK_LIMIT;
                                              }
                                              DI2 = DigInput(2);
                                          }
                              }

                   SmState DISCHARGE_PROGRESS_CONTINUOUS_MOTOR2 {

                              SIG_ENTRY = {  print ("DISCHARGE PROGRESS");}

                              Input2Falling = DISCHARGE_DONE_MOTOR2;
                                TickPeriod_1 = {DI2 = DigInput(2);
                                MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                }
                                              }


                   SmState DISCHARGE_PROGRESS_QUANTITY_MOTOR2 {

                              SIG_ENTRY = {  print ("DISCHARGE PROGRESS");}

                              SIG_TARGET_REACHED_2 = DISCHARGE_DONE_MOTOR2;

                               TickPeriod_1 = {DI2 = DigInput(2);
                               MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                }

                                                       }

                   SmState PAUSE_TIME_MOTOR2 {

                               SIG_ENTRY = {  print ("PAUSE TIME");

                                            }

                               TickPeriod_1 = {DI2 = DigInput(2);
                                               MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                                }
                               TickSingle_DisCharge_Delay_1 = START_SUCKBACK_MOTOR2;

                                       }

                   SmState SUCKBACK_PROGRESS_MOTOR2 {
                                SIG_ENTRY = {  print ("SUCKBACK Done"); }

                                TickPeriod_1 = {DI1 = DigInput(1);
                                               MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                                }
                                SIG_TARGET_REACHED_2 = SUCKBACK_DONE_MOTOR2;
                                             }



                                    }




SmMachine Operation{1, *, Motor1, 20, 2}        // Operation has Motor1 as its top-level state machine.
SmMachine Operation_MOTOR2{2, *, Motor2, 20, 2} // Operation_MOTOR2 has Motor2 as its top-level state machine.




long DISCHARGE_START (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x01210005] == 0)) // When continuous mode is selected
  {
     if(MOTOR1_RPM == 0)
     {
        print ("Motor1 RPM 0");
        return(SmTrans(Standing));
     }
     if(MOTOR1_RPM != 0)
     {
        print (" START DISCHARGE CONTINUOUS MODE ");
        Cvel (C_AXIS1, MOTOR1_RPM); // Motor 1 velocity control
        Acc(C_AXIS1, VEL_ACC);      // Acceleration for motor 1 velocity control
        Dec(C_AXIS1, VEL_DEC);      // Deceleration for motor 1 velocity control

        AxisControl(C_AXIS1,ON);    // Enable motor 1.
        AxisCvelStart(C_AXIS1);     // Start motor 1 velocity control.
        MOTOR1_DONE = 0;            // Clear the motor 1 acknowledgement signal.
        return(SmTrans(DISCHARGE_PROGRESS_CONTINUOUS)); // After starting, transition to DISCHARGE_PROGRESS_CONTINUOUS.
     }
  }

  if ((Sysvar[0x01210005] == 1)) // When quantity mode is selected
  {
      if(MOTOR1_RPM == 0)
      {
         print ("Motor1 RPM 0");
         return(SmTrans(Standing));
      }
      if(MOTOR1_RPM != 0)
      {
        print (" START DISCHARGE QUANTITY MODE ");

        Vel(C_AXIS1, MOTOR1_RPM); // Velocity for motor 1 position control
        Acc(C_AXIS1, VEL_ACC);    // Acceleration for motor 1 position control
        Dec(C_AXIS1, VEL_DEC);    // Deceleration for motor 1 position control

        AxisControl(C_AXIS1,ON); // Enable motor 1.
        AxisPosAbsStart(C_AXIS1, Apos(C_AXIS1) + NumberOfTurn_QUANTITY); // Absolute move to the current position plus the configured quantity angle.
        MOTOR1_DONE = 0;        // Clear the motor 1 acknowledgement signal.
        return(SmTrans(DISCHARGE_PROGRESS_QUANTITY)); // After starting, transition to DISCHARGE_PROGRESS_QUANTITY.
      }

  }


}

long DISCHARGE_DONE(long id, long signal, long event[], long data[])
{


  if (( Sysvar[0x01210005] == 0))
  {
  print (" DISCHARGE CONTINUOUS MODE DONE");

  AxisCvelStop(C_AXIS1); // Stop velocity control.
  SmTimer (MOTOR1_PAUSETIME,id, TickSingle_DisCharge_Delay); // MOTOR1_PAUSETIME
  return(SmTrans(PAUSE_TIME)); //Transition to the PAUSE_TIME state.

  }

  if (( Sysvar[0x01210005] == 1))
  {
  print (" DISCHARGE QUANTITY MODE DONE");
  SmTimer (MOTOR1_PAUSETIME,id, TickSingle_DisCharge_Delay); // MOTOR1_PAUSETIME
  return(SmTrans(PAUSE_TIME));//Transition to the PAUSE_TIME state.

  }


}


long START_SUCKBACK (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x01210005] == 0))
  {
  print (" START SUCKBACK Continuous Mode");
  Vel (C_AXIS1,MOTOR1_RPM);
  Acc(C_AXIS1, VEL_ACC);
  Dec(C_AXIS1, VEL_DEC);
  AxisPosAbsStart(C_AXIS1, Apos(C_AXIS1) - NumberOfTurn);
  return(SmTrans(SUCKBACK_PROGRESS)); // Transition to the SUCKBACK_PROGRESS state.
  }

  if (( Sysvar[0x01210005] == 1))
  {
  print (" START SUCKBACK QUANTITY Mode");
  Vel (C_AXIS1,MOTOR1_RPM);
  Acc(C_AXIS1, VEL_ACC);
  Dec(C_AXIS1, VEL_DEC);
  AxisPosAbsStart(C_AXIS1, Apos(C_AXIS1) - NumberOfTurn);
  return(SmTrans(SUCKBACK_PROGRESS));// Transition to the SUCKBACK_PROGRESS state.
  }



}


long SUCKBACK_DONE (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x01210005] == 0))
  {
  print (" SUCKBACK Continuous Mode DONE ");
  MOTOR1_DONE = 1; // Set the motor 1 acknowledgement signal to 1.
  return(SmTrans(Standing)); // Transition to the Standing state.
  }
  if (( Sysvar[0x01210005] == 1))
  {
  print (" SUCKBACK Quantity Mode DONE ");
  MOTOR1_DONE = 1; // Set the motor 1 acknowledgement signal to 1.
  return(SmTrans(Standing));// Transition to the Standing state.
  }

}

////////////////////////////////////////////////////////////////////////
////////////////////////////MOTOR 2/////////////////////////////////////
////////////////////////////////////////////////////////////////////////

long DISCHARGE_START_MOTOR2 (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x0121000A] == 0))
  {
       if(MOTOR2_RPM == 0)
       {
       print ("Motor2 RPM 0");
       return(SmTrans(Standing_MOTOR2));
       }
       if(MOTOR2_RPM != 0)
       {
       print (" START DISCHARGE CONTINUOUS MODE ");
       Cvel (C_AXIS2, MOTOR2_RPM);
       Acc(C_AXIS2, VEL_ACC);
       Dec(C_AXIS2, VEL_DEC);

       AxisControl(C_AXIS2,ON);
       AxisCvelStart(C_AXIS2);
       MOTOR2_DONE = 0;
       return(SmTrans(DISCHARGE_PROGRESS_CONTINUOUS_MOTOR2));
       }
  }

  if ((Sysvar[0x0121000A] == 1))
  {
       if(MOTOR2_RPM == 0)
       {
       print ("Motor2 RPM 0");
       return(SmTrans(Standing_MOTOR2));
       }
       if(MOTOR2_RPM != 0)
       {
       print (" START DISCHARGE QUANTITY MODE ");

       Vel(C_AXIS2, MOTOR2_RPM);
       Acc(C_AXIS2, VEL_ACC);
       Dec(C_AXIS2, VEL_DEC);

       AxisControl(C_AXIS2,ON);
       AxisPosAbsStart(C_AXIS2, Apos(C_AXIS2) + NumberOfTurn_QUANTITY_MOTOR2);
       MOTOR2_DONE = 0;
       return(SmTrans(DISCHARGE_PROGRESS_QUANTITY_MOTOR2));
       }

  }


}

long DISCHARGE_DONE_MOTOR2(long id, long signal, long event[], long data[])
{


  if (( Sysvar[0x0121000A] == 0))
  {
  print (" DISCHARGE CONTINUOUS MODE DONE");

  AxisCvelStop(C_AXIS2);
  SmTimer (MOTOR2_PAUSETIME,id, TickSingle_DisCharge_Delay_1); // Use MOTOR1_PAUSETIME instead of 1000.
  return(SmTrans(PAUSE_TIME_MOTOR2));

  }

  if (( Sysvar[0x0121000A] == 1))
  {
  print (" DISCHARGE QUANTITY MODE DONE");
  SmTimer (MOTOR2_PAUSETIME,id, TickSingle_DisCharge_Delay_1); // Use MOTOR1_PAUSETIME instead of 1000.
  return(SmTrans(PAUSE_TIME_MOTOR2));

  }


}


long START_SUCKBACK_MOTOR2 (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x0121000A] == 0))
  {
  print (" START SUCKBACK Continuous Mode");
  Vel (C_AXIS2,MOTOR2_RPM);
  Acc(C_AXIS2, VEL_ACC);
  Dec(C_AXIS2, VEL_DEC);
  AxisPosAbsStart(C_AXIS2, Apos(C_AXIS2) - NumberOfTurn_MOTOR2);
  return(SmTrans(SUCKBACK_PROGRESS_MOTOR2));
  }

  if (( Sysvar[0x0121000A] == 1))
  {
  print (" START SUCKBACK QUANTITY Mode");
  Vel (C_AXIS2,MOTOR2_RPM);
  Acc(C_AXIS2, VEL_ACC);
  Dec(C_AXIS2, VEL_DEC);
  AxisPosAbsStart(C_AXIS2, Apos(C_AXIS2) - NumberOfTurn_MOTOR2);
  return(SmTrans(SUCKBACK_PROGRESS_MOTOR2));
  }



}


long SUCKBACK_DONE_MOTOR2 (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x0121000A] == 0))
  {
  print (" SUCKBACK Continuous Mode DONE ");
  MOTOR2_DONE = 1;
  return(SmTrans(Standing_MOTOR2));
  }
  if (( Sysvar[0x0121000A] == 1))
  {
  print (" SUCKBACK Quantity Mode DONE ");
  MOTOR2_DONE = 1;
  return(SmTrans(Standing_MOTOR2));
  }

}

long Error__RESET (long id, long signal, long event[], long data[])
{

    ErrorClear();
    AmpErrorClear(C_AXIS1);
    AmpErrorClear(C_AXIS2);
    Sysvar[0x01210012] = 0;
    return(SmTrans(Standing));

}

long Save_Data (long id, long signal, long event[], long data[])
{
print (" SAVE DATA TO ROM");
data[SMDATA_SAVEARRAY]  = 1;
Delay(1);
Save(ARRAYS);
return(SmTrans(Standing));
}







long main(void)
{




	ErrorClear();

	ECatMasterCommand(0x1000, 0);

	EtherCAT_Slave_Count = sdkEtherCATMasterInitialize();

    for ( i = 1000001; i <= 1000001+EtherCAT_Slave_Count-1; i++) {
    sdkEpos4_SetupECatSdoParam(i, C_PDO_NUMBER, C_AXISPOLARITY, C_OP_MODE_CSP );

    }

	for (i = 1; i <= EtherCAT_Slave_Count; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
        }

	sdkEtherCATMasterStart();

    for (i = 0; i <= EtherCAT_Slave_Count-1 ; i++)
    {
    sdkEpos4_SetupECatBusModule(i, i+1000001, C_PDO_NUMBER, C_OP_MODE_CSP);
    sdkEpos4_SetupECatVirtAmp(i, C_MOTOR_MAX_RPM, C_OP_MODE_CSP);
    sdkEpos4_SetupECatVirtCntin(i, C_OP_MODE_CSP);

    AXE_PARAM(i, POSENCREV) = 1;		// Number of revolutions of the motor
    AXE_PARAM(i, POSENCQC)  = 512*4;     // Number of quadcounts in POSENCREV revolutions
    AXE_PARAM(i, POSFACT_Z) = 4356;		// Number of revolutions of the input shaft
    AXE_PARAM(i, POSFACT_N) = 100;		// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
    AXE_PARAM(i, FEEDREV)   = 1;		// Number of revolutions of the gear box output shaft
    AXE_PARAM(i, FEEDDIST)  = 512*4;	// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft


    AXE_PARAM(i, VELMAX) = C_MOTOR_MAX_RPM;
    AXE_PARAM(i, RAMPMIN) = 100;	// Maximum acceleration

    AXE_PARAM(i, VELMAX) = 4000;

    AXE_PARAM(i, KPROP) = 0;
    AXE_PARAM(i, KDER) = 0;
    AXE_PARAM(i, POSERR) = 200000;
    AXE_PARAM(i, RAMPTYPE) = 0;
    AXE_PARAM(i, JERKMIN) = 100;
    AXE_PARAM(i, VELRES) = 100;
    }





	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	ErrorClear();
    AmpErrorClear(C_AXIS1);
    AmpErrorClear(C_AXIS2);


    res = SmRun(Operation,Operation_MOTOR2);

}
//$X {Element (1),1,1,0,-1,0,-1,0,(-1),-1},0x2100,5,0
//$X {Element (2),1,1,0,-1,0,-1,0,(-1),-1},0x2100,6,0
//$X {Element (3),1,1,0,-1,0,-1,0,(-1),-1},0x2100,7,0
//$X {Element (4),1,1,0,-1,0,-1,0,(-1),-1},0x2100,8,0
//$X {Element (5),1,1,0,-1,0,-1,0,(-1),-1},0x2100,9,0
//$X {Element (6),1,1,0,-1,0,-1,0,(-1),-1},0x2100,10,0
//$X {Element (7),1,1,0,-1,0,-1,0,(-1),-1},0x2100,11,0
//$X {Element (8),1,1,0,-1,0,-1,0,(-1),-1},0x2100,12,0
//$X {Element (9),1,1,0,-1,0,-1,0,(-1),-1},0x2100,13,0
//$X {Element (10),1,1,0,-1,0,-1,0,(-1),-1},0x2100,14,0
//$X {Element (12),1,1,0,-1,0,-1,0,(-1),-1},0x2100,16,0
//$X {Element (13),1,1,0,-1,0,-1,0,(-1),-1},0x2100,17,0
//$X {Element (14),1,1,0,-1,0,-1,0,(-1),-1},0x2100,18,0
//$X {Element (11),1,1,0,-1,0,-1,0,(-1),-1},0x2100,15,0
//$X {MOTOR_Write_Array,1,1,0,14,1,0,1,(18,100),1},0x2100,23,0
