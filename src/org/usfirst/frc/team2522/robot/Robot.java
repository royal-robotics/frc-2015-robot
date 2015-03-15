package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class Robot extends IterativeRobot {

	RobotDrive robotDrive;

	public static final double WHEEL_PULSES_PER_INCH = 171.0/12.0;
	public static final double BUTTON_DELAY = 0.5;
	public static final double LIFT_PULSES_PER_INCH = 28;
	public String CurrentAuto = " ";

	// Joysticks
	// -------------------------------------------------------------------
	Joystick leftstick;
	Joystick rightstick;
	Joystick operatorstick;

	// Motors
	// -------------------------------------------------------------------
	VictorSP frontRightChannel;
	VictorSP rearRightChannel;
	VictorSP frontLeftChannel;
	VictorSP rearLeftChannel;
	VictorSP liftLeft;
	VictorSP liftRight;
	Talon canRight;
	Talon canLeft;

	// Limit Switches
	// -------------------------------------------------------------------
	DigitalInput liftLimitUp;
	DigitalInput liftLimitDown;
	DigitalInput canLimitUp;

	// Encoders
	// ---------------------------------------------------------------
	Encoder rightDrive;
	// Encoder leftDrive;
	Encoder lift;
	Encoder canArm;

	Gyro gyro;
	// LEDs
	// ---------------------------------------------------------------
	LEDController leds;

	// Camera
	// ---------------------------------------------------------------
	// CameraServer server;

	// Misc.
	// ---------------------------------------------------------------
	DriverStation ds;
	Relay canHolder;
	Timer holderTimer;
	Boolean isIn = true;
	Boolean resetButton = true;
	Boolean seenBottom = false;
	double topEncoder = 100;
	
	public void robotInit() {
		// Init Joysticks
		// ---------------------------------------------------------------
		leftstick = new Joystick(0);
		rightstick = new Joystick(1);
		operatorstick = new Joystick(2);

		// Init Motors
		// ---------------------------------------------------------------
		frontRightChannel = new VictorSP(0);
		frontLeftChannel = new VictorSP(1);
		rearRightChannel = new VictorSP(2);
		rearLeftChannel = new VictorSP(3);
		liftRight = new VictorSP(4);
		liftLeft = new VictorSP(5);
		canRight = new Talon(6);
		canLeft = new Talon(7);
		canHolder = new Relay(0);
		holderTimer = new Timer();

		// Init Drive
		// ---------------------------------------------------------------
		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);

		// Init Limit Switches
		// ---------------------------------------------------------------
		liftLimitUp = new DigitalInput(0);
		liftLimitDown = new DigitalInput(1);
		canLimitUp = new DigitalInput(2);

		gyro = new Gyro(0);
		gyro.initGyro();
		
		// Init Encoders
		// ---------------------------------------------------------------
		lift = new Encoder(new DigitalInput(4), new DigitalInput(5));
		rightDrive = new Encoder(new DigitalInput(6), new DigitalInput(7));
		canArm = new Encoder (new DigitalInput(8), new DigitalInput(9));
		

		lift.setDistancePerPulse(1 / LIFT_PULSES_PER_INCH);
		rightDrive.setDistancePerPulse(1 / WHEEL_PULSES_PER_INCH);

		lift.reset();
		rightDrive.reset();
		
		canArm.reset();

		// Init LEDs
		// ---------------------------------------------------------------
		leds = new LEDController(new DigitalOutput(16), new DigitalOutput(18), new DigitalOutput(19));
		
		// Init Camera
		// ---------------------------------------------------------------
		//server = CameraServer.getInstance();
		//server.startAutomaticCapture("cam0");
		
		ds = DriverStation.getInstance();

		// Init SmartDashboard
		// ---------------------------------------------------------------
		dashboardOutput(operatorstick.getY(), 0);
	}

	public void disabledPeriodic() {
		//leds.setColor(LEDUtil.Color.WHITE);
		dashboardOutput(operatorstick.getY(), 0);
	}
	
	public void disabledInit() {
		leds.setColor(LEDUtil.Color.WHITE);
	}
	
	public void testPerodic()
	{
		
	}
	
	int autoState = 0;

	public void autonomousInit()
	{
		autoState = 0;
		
		rightDrive.reset();
		
		lift.reset();
		gyro.reset();
	}

	//rotation and gyro positive = right
	//rotation and gyro negative = left
	public void autonomousPeriodic() {

		int autoMode = getAutoMode();
		
		if (autoMode == 1)                                      // Do nothing
		{
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		}
		if (autoMode == 2)                                      // Two totes and turn
		{
			if (autoState == 0)                                 //Drive Forward
			{
				if (rightDrive.getDistance() < 10) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				} else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 1)                            // Pick Up totes
			{
				moveLift(.75, 8);
				if (lift.getDistance() >= 8)
					
				{
					moveLift(0.0);
					autoState++;
				}
			} 
			else if (autoState == 2)                            // Drive Back
			{
				if (rightDrive.getDistance() > -40) 
				{
					robotDrive.mecanumDrive_Cartesian(0, .50, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
					
				}
			}
			else if (autoState == 3)
			{
				if (gyro.getAngle() > -160)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, -.4, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
				}
			}
			else if (autoState == 4)
			{
				if (gyro.getAngle() > -180)
				{
					robotDrive.mecanumDrive_Cartesian(0,0,-.25,0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 5)
			{
				if (rightDrive.getDistance() < 58)
				{
					driveStraight(-.30);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
			else if (autoState == 6)
			{
				moveLift(.5, 0);
				if (lift.getDistance() <= 0)
					
				{
					moveLift(0.0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
				}
			}
			else if (autoState == 7)
			{
				if (rightDrive.getDistance() > -50)
				{
					robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
				} 
			}
		
		}
		else if (autoMode == 4)			// Pick up yellow tote and drive backwards
		{
			if (autoState == 0)
			{
				moveLift(.25, 4.0);
				if (lift.getDistance() >= 4.0)
				{
					moveLift(0.0);
					autoState++;
				}
			}
			else if (autoState == 1)
			{
				if (rightDrive.getDistance() < 6) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				} else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
				}
			} 
			else if (autoState == 2)
			{
				moveLift(.25, 12.0);
				if (lift.getDistance() >= 12.0)
				{
					moveLift(0.0);
					autoState++;
				}
			}
			else if (autoState == 3)
			{
				if (rightDrive.getDistance() > -80)
				{
					robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
		}
		else if (autoMode == 5)                                 //Two Totes from Landfill and Drive Back
		{
			if (autoState == 0)
			{
				if (rightDrive.getDistance() < 9.5) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				} else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 1)
			{
				moveLift(.5, 6);
				if (lift.getDistance() >= 6)
					
				{
					moveLift(0.0);
					autoState++;
				}
			} 
			else if (autoState == 2)
			{
				// -75
				if (rightDrive.getDistance() > -75) 
				{
					robotDrive.mecanumDrive_Cartesian(0, .50, 0, gyro.getAngle());
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
					
				}
			}
		}
		else if (autoMode == 3)									//Drive Backwards from LandFill
		{
			if (autoState == 0 && rightDrive.getDistance() > -84) 
			{
				robotDrive.mecanumDrive_Cartesian(0, .75, 0, 0);
			}
			else
			{
				robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoState++;
			}
		}
		else if (autoMode == 6) 								// Drive Backwards from Yellow Crates
		{
			if (autoState == 0 && rightDrive.getDistance() > -80)
			{
				robotDrive.mecanumDrive_Cartesian(0, .75, 0, 0);
			}
			else
			{
				robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoState++;
			}
		}
		else if (autoMode == 7)									//Pick up and stack 3 yellow totes
		{
			if (autoState == 0)
			{
				moveLift(.25, 4.0);
				if (lift.getDistance() >= 4.0)
				{
					moveLift(0.0);
					autoState++;
				}
			} 
			else if (autoState == 1)
			{
				if (rightDrive.getDistance() < 6) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				} else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
				}
			} 
			else if (autoState == 2)
			{
				moveLift(.5, 12);
				if (lift.getDistance() >= 12)
					
				{
					moveLift(0.0);
					autoState++;
				}
			}
			else if (autoState == 3)
			{
				if (rightDrive.getDistance() > -13)
				{
					robotDrive.mecanumDrive_Cartesian(0, .60, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
				}
			}
			else if (autoState == 4)
			{
				if (rightDrive.getDistance() < 84)
				{
					strafe(0.5);
					//robotDrive.mecanumDrive_Cartesian(0.5, 0.0, 0.0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
			else if (autoState == 5) 
			{
				moveLift(.25, 23);
				if (lift.getDistance() >= 23)
				{
					moveLift(0.0);
					autoState++;
					
					rightDrive.reset();
				}
			}
			else if (autoState == 6) 
			{
				
				if (rightDrive.getDistance() < 29) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				} else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					
					rightDrive.reset();
				}
			}
			else if (autoState == 7)
			{
				moveLift(.30, 12);
				if (lift.getDistance() < 18)
				{
					robotDrive.mecanumDrive_Cartesian(0, .3, 0, gyro.getAngle());
				}
				if (rightDrive.getDistance() < -8)
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
				}
				if (lift.getDistance() <= 12)
				{
					moveLift(0.0);
					autoState++;
					
					rightDrive.reset();
				}
			}
			else if (autoState == 8)
			{
				moveLift(.25, 3.0);
				if (lift.getDistance() <= 4)
				{
					moveLift(0);
					autoState++;
				}
			}
			else if (autoState == 9)
			{
				if (rightDrive.getDistance() < 12) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.25, 0.0, 0.0);
				} else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					
					rightDrive.reset();
				}
			}
			else if (autoState == 10)
			{
				moveLift(.25, 12);
				if (lift.getDistance() >= 12)
				{
					moveLift(0.0);
					autoState++;
				}
			}
			else if (autoState == 11)
			{
				if (rightDrive.getDistance() > -15)
				{
					robotDrive.mecanumDrive_Cartesian(0, .50, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					
					rightDrive.reset();
				}
			}
			else if (autoState == 12)
			{
				if (rightDrive.getDistance() < 112)
				{
					strafe(0.5);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
			else if (autoState == 13)
			{
				moveLift(.25, 23);
				if (lift.getDistance() >= 23)
				{
					moveLift(0.0);
					autoState++;
					rightDrive.reset();
					
				}
			}
			else if (autoState == 14)
			{
				if (rightDrive.getDistance() < 45)
				{
					robotDrive.mecanumDrive_Cartesian(0, -0.3, 0.0, 0.0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
					
				}
			}
			else if (autoState == 15)
			{
				moveLift(0.25, 3.0);
				if (lift.getDistance() <= 18)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0.3, 0, 0);
				}
				if (rightDrive.getDistance() < -8)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				}
				if (lift.getDistance() <= 3.0)
				{
					moveLift(0);
					autoState++;
					rightDrive.reset();
					
				}
			}
			else if (autoState == 16)
			{
				if (rightDrive.getDistance() < 12)
				{
					robotDrive.mecanumDrive_Cartesian(0, -0.3, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
			else if (autoState == 17)
			{
				moveLift(0.25, 8.0);
				if (lift.getDistance() >= 8.0)
				{
					moveLift(0);
					autoState++;
					rightDrive.reset();
					
				}
			}
			else if (autoState == 18)
			{
				if (rightDrive.getDistance() > -60.0)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0.3, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
					
				}
			}
			else if (autoState == 19)
			{
				if (rightDrive.getDistance() < 20.0)
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, -0.5, 0.0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
				}
			}
			else if (autoMode == 8)                                 //Stack 4 totes in Landfill
			{
				if (autoState == 0)                                 //Drive Forward
				{
					if (rightDrive.getDistance() < 10) 
					{
						robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
					} else 
					{
						robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
						autoState++;
						rightDrive.reset();
					}
				}
				else if (autoState == 1)                            // Pick Up totes
				{
					moveLift(.75, 20);
					if (lift.getDistance() >= 20)
						
					{
						moveLift(0.0);
						autoState++;
					}
				} 
				else if (autoState == 2)                            // Drive Back
				{
					if (rightDrive.getDistance() > -16) 
					{
						robotDrive.mecanumDrive_Cartesian(0, .50, 0, 0);
					}
					else
					{
						robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
						autoState++;
						rightDrive.reset();
						gyro.reset();
						
					}
				}
				else if (autoState == 3)                            //Strafe Left
				{
					if (rightDrive.getDistance() > -86)
					{
						strafe(-.50);
					}
					else 
					{
						robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
						autoState++;
						rightDrive.reset();
					}
				}
				else if (autoState == 4)                          //Drive Forward with totes
				{
					if (rightDrive.getDistance() < 9)
					{
						robotDrive.mecanumDrive_Cartesian(0, -.5, 0, 0);
					}
					else 
					{
						robotDrive.mecanumDrive_Cartesian(0,0,0,0);
						autoState++;
						rightDrive.reset();
					}
				}
				else if (autoState == 5)                        //Release Totes
				{
					moveLift(.75, 11);
					if (lift.getDistance() <= 11)
						
					{
						moveLift(0.0);
						autoState++;
					}
				}
				else if (autoState == 6)                      //Drive Back without totes
				{
					if (rightDrive.getDistance() > -10)
					{
						robotDrive.mecanumDrive_Cartesian(0, .25, 0, 0);
					}
					else 
					{
						robotDrive.mecanumDrive_Cartesian(0,0,0,0);
						autoState++;
						rightDrive.reset();
					}
				}
				else if (autoState == 7)                      //Lower lift down
				{
					moveLift(.75, 0);
					if (lift.getDistance() <= 0)
						
					{
						moveLift(0.0);
						autoState++;
					}
				}
				else if (autoState == 8)                     //Drive forward  
				{
					if (rightDrive.getDistance() < 18.5)
					{
						robotDrive.mecanumDrive_Cartesian(0,-.25,0,0);
					}
					else 
					{
						robotDrive.mecanumDrive_Cartesian(0,0,0,0);
						autoState++;
						rightDrive.reset();
					}
				}
				else if (autoState == 9)                     // Pick up totes
				{
					moveLift(.75, 6);
					if (lift.getDistance() >= 6)
						
					{
						moveLift(0.0);
						autoState++;
					}
				}
				else if (autoState == 10)                  //Drive back
				{
					if (rightDrive.getDistance() > -74)
					{
						robotDrive.mecanumDrive_Cartesian(0,.5,0,0);
					}
					else 
					{
						robotDrive.mecanumDrive_Cartesian(0,0,0,0);
						autoState++;
						rightDrive.reset();
					}
				}
					
					
			}
		}
		
		dashboardOutput(0, autoState);
	}
	
	public void teleopInit() 
	{
		seenBottom = false;
	}
	
	public void teleopPeriodic() {
		/*
		if (ds.getMatchTime() < 120.0) {
			if (ds.getAlliance() == Alliance.Blue) {
				leds.setColor(LEDUtil.Color.BLUE);
			} else if (ds.getAlliance() == Alliance.Red) {
				leds.setColor(LEDUtil.Color.RED);
			} else {
				leds.setColor(LEDUtil.Color.WHITE);
			}
		} else {
			if ((int)(ds.getMatchTime() * 10) % 5 == 0) {
				leds.setColor(LEDUtil.Color.GREEN);
			} else {
				leds.setColor(LEDUtil.Color.OFF);
			}
		}
		*/
		//Can arm Placement
		

		
		
		
		//Drive power adjustment
		
		double drivePower = .35;
		if (leftstick.getRawButton(1) || rightstick.getRawButton(1)) 
		{
			if (leftstick.getRawButton(1) && rightstick.getRawButton(1)) 
			{
				drivePower = 1;
			} else {
				drivePower = .75;
			}
		} 
		
		// DeadZone Value
		// ---------------------------------------------------------------
		double deadzone = 0.15;
		
		if (operatorstick.getRawButton(1))
		{
			moveLift(.75, 0);
			moveCan (1, 90);
		}
		else if (operatorstick.getRawButton(2))
		{
			moveLift(.75, 0);
			moveCan (1, 69);
		}
		else if (operatorstick.getRawButton(3))
		{
			moveLift(.75, 2.7);
			moveCan (1, 60);
		}
		else if (operatorstick.getRawButton(4))
		{
			moveLift(.75, 0);
			moveCan(1, 110);
		}
		else 
		{
			// Drive Container Lift
			// ---------------------------------------------------------------
			
			double containerLift = -operatorstick.getRawAxis(3) * 0.70;
		      
			if (operatorstick.getRawButton(7))
			{
				moveLift(0.50, 11);
			} 
			else if (operatorstick.getRawButton(5))
			{
				moveLift(.50, 24.5);
			}
			else
	       	{
	   			moveLift(containerLift);
	       	}
			
			// Drive Can Lift
			// ---------------------------------------------------------------
			
			double canLift = operatorstick.getY();
			if ((canLift > 0) && (canLift > deadzone))
			{
				moveCan(canLift);
			} 
			else if ((canLift < 0) && (canLift < -deadzone) && canLimitUp.get()) 
			{
				if (operatorstick.getRawButton(6)) 
				{
					moveCan(canLift/2);
				} else 
				{
					moveCan(canLift);
				}

			} else 
			{
				moveCan(0);
			}
		}


		// Magnitude for Mecanum
		// ---------------------------------------------------------------

		// X-Direction for Mecanum
		// ---------------------------------------------------------------
		double leftX = leftstick.getX();
		if (leftX > -deadzone && leftX < deadzone) {
			leftX = 0;
		}

		// Y-Direction for Mecanum
		// ---------------------------------------------------------------
		double leftY = leftstick.getY();
		if (leftY > -deadzone && leftY < deadzone) {
			leftY = 0;
		} else if((leftX > .9 || leftX < -.9) && (leftY > -.50 && leftY < .50))
		{
			leftY = 0;
		}

		// Rotation for Mecanum
		// ---------------------------------------------------------------
		double rotation = rightstick.getX();
		if (rotation > -deadzone && rotation < deadzone) {
			rotation = 0;
		}

		// Mecanum Drive
		// ---------------------------------------------------------------
		robotDrive.mecanumDrive_Cartesian(leftX * drivePower, leftY * drivePower, rotation * drivePower, 0.0);
	
		dashboardOutput(0, 0);
	}
	
	public int getAutoMode()
	{
		if (leftstick.getRawAxis(2) >= .5)
		{
			CurrentAuto = "Do Nothing";
			return 1;
		}
		else if (leftstick.getRawAxis(2) < .5 && leftstick.getRawAxis(2) > 0) 
		{
			CurrentAuto = "2 Totes & Drive Back";
			return 2;
		}
		else if (leftstick.getRawAxis(2) < 0 && leftstick.getRawAxis(2) > -.5)
		{
			CurrentAuto = "Drive Back From Landfill";
			return 3;
		}
		else
		{
			CurrentAuto = "Yellow Tote & Drive Back";
			return 4;
		}
	}
	
	public void dashboardOutput(double canLift, int autoState)
	{	
		
		getAutoMode();
		
		SmartDashboard.putNumber("Lift Encoder Value: ", lift.getDistance());
		SmartDashboard.putNumber("Right Drive Encoder Value: ", rightDrive.getDistance());
		SmartDashboard.putNumber("Can Lift Speed: ", canLift);
		SmartDashboard.putBoolean("LiftUp", liftLimitUp.get());
		SmartDashboard.putBoolean("LiftDown", liftLimitDown.get());
		SmartDashboard.putBoolean("CanUp", canLimitUp.get());
		SmartDashboard.putNumber("Autonomous Mode", leftstick.getRawAxis(2));
		SmartDashboard.putString("Current Autonomus", (String) CurrentAuto);
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		SmartDashboard.putNumber("AutoState", autoState );
		SmartDashboard.putNumber("CanArm Encoder Value", canArm.get());
		
	}

	public void strafe(double speed)
	{
		if (gyro.getAngle() > 0 )
		{
			robotDrive.mecanumDrive_Cartesian(speed, 0, -.05, 0.0);
		}
		else if (gyro.getAngle() < 0)
		{
			robotDrive.mecanumDrive_Cartesian(speed, 0, .05, 0.0);
		}
	}
	
	public void driveStraight(double speed)
	{
		if (gyro.getAngle() > -180 )
		{
			robotDrive.mecanumDrive_Cartesian(0, speed, -.05, 0.0);
		}
		else if (gyro.getAngle() < -180)
		{
			robotDrive.mecanumDrive_Cartesian(0, speed, .05, 0.0);
		}
	}
	public void moveCan (double speed)
	{
		if (speed > 0) {
			moveCan(speed, 110);
			
		} else {
			moveCan(-speed, -360);		
		}
	}
	public void moveCan(double speed, double target)
	{
		if (canArm.get() < target - 2)
		{
			//canLeft.set(speed);
			canRight.set(-speed);
		}
		else if (canArm.get() > target + 2)
		{
			//canLeft.set(-speed);
			canRight.set(speed);
		}
		else
		{
			//canLeft.set(0);
			canRight.set(0);
		}
		
		if (!canLimitUp.get())
		{
			canArm.reset();
		}
	}
	
	public void moveLift(double speed) {
		if (speed > 0) {
			moveLift(speed, 100);
			
		} else {
			moveLift(-speed, -100);		
		}
	}
    
	public void moveLift(double speed, double target) {
		
		SmartDashboard.putNumber("LiftSpeed", speed);
		SmartDashboard.putNumber("LiftTarget", target);
		
		if (liftLimitUp.get() && lift.getDistance() < target - .15) 
		{
			if (seenBottom && lift.getDistance() > topEncoder)
			{
				liftLeft.set(0);
				liftRight.set(0);	
			}
			else
			{
				liftLeft.set(-speed);
				liftRight.set(speed);
			}

		}
		else if (liftLimitDown.get() && lift.getDistance() > target + .15) 
		{
			if (seenBottom && lift.getDistance() < 0) 
			{
				liftLeft.set(0);
				liftRight.set(0);
			}
			else 
			{
				liftLeft.set(speed);
				liftRight.set(-speed);
			}
		}
		else
		{
			liftLeft.set(0);
			liftRight.set(0);
		}

		if (!liftLimitDown.get()) 
		{
			lift.reset();
			seenBottom = true;
		} 
		if (!liftLimitUp.get())
		{
			topEncoder = lift.getDistance();
			
					
		}
	}
}
