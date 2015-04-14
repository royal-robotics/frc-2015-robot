package org.usfirst.frc.team2522.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

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

import java.util.Comparator;
import java.util.Vector;

public class Robot extends IterativeRobot {

	RobotDrive robotDrive;

	public static final double WHEEL_PULSES_PER_INCH = 171.0/12.0;
	public static final double BUTTON_DELAY = 0.5;
	public static final double LIFT_PULSES_PER_INCH = 28;
	public String CurrentAuto = " ";
	
	int session;
	
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
		double PercentAreaToImageArea;
		double Area;
		double ConvexHullArea;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		
		public int compareTo(ParticleReport r)
		{
			return (int)(r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int)(r1.Area - r2.Area);
		}
	};

	//Structure to represent the scores for the various tests used for target identification
	public class Scores {
		double Trapezoid;
		double LongAspect;
		double ShortAspect;
		double AreaToConvexHullArea;
	};

	//Images
	Image frame;
	Image binaryFrame;
	int imaqError;

	//Constants
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(24, 49);	//Default hue range for yellow tote
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(67, 255);	//Default saturation range for yellow tote
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(49, 255);	//Default value range for yellow tote
	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
	double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
	double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
	double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
	double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	Scores scores = new Scores();


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
	Talon canBack;

	// Limit Switches
	// -------------------------------------------------------------------
	DigitalInput liftLimitUp;
	DigitalInput liftLimitDown;
	DigitalInput canLimitUp;
	DigitalInput canThiefLimit;

	// Encoders
	// ---------------------------------------------------------------
	Encoder rightDrive;
	// Encoder leftDrive;
	Encoder lift;
	Encoder canArm;
	Encoder canThief;

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
	Timer canThiefTimer = new Timer();
	
	public void robotInit() {
		// Init Vision Tracking
		// ---------------------------------------------------------------
		// create images
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);

		
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);

		//Put default values to SmartDashboard so fields will appear
		SmartDashboard.putNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Tote val min", TOTE_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Tote val max", TOTE_VAL_RANGE.maxValue);
		SmartDashboard.putNumber("Area min %", AREA_MINIMUM);
		
		NIVision.IMAQdxStartAcquisition(session);
		
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
		canBack = new Talon(7);
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
		canThiefLimit = new DigitalInput(14);

		gyro = new Gyro(0);
		gyro.initGyro();
		
		// Init Encoders
		// ---------------------------------------------------------------
		lift = new Encoder(new DigitalInput(4), new DigitalInput(5));
		rightDrive = new Encoder(new DigitalInput(6), new DigitalInput(7));
		canArm = new Encoder (new DigitalInput(8), new DigitalInput(9));
		canThief = new Encoder(new DigitalInput (10), new DigitalInput(12));
		

		lift.setDistancePerPulse(1 / LIFT_PULSES_PER_INCH);
		rightDrive.setDistancePerPulse(1 / WHEEL_PULSES_PER_INCH);

		lift.reset();
		rightDrive.reset();
		
		canArm.reset();
		canThief.reset();

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
		
		canThiefTimer.start();
		canThiefTimer.reset();
	}

	//rotation and gyro positive = right
	//rotation and gyro negative = left
	double adjDistance = 0.0;
	public void autonomousPeriodic() {

		int autoMode = getAutoMode();
		
		if (autoMode == 1)                                      // Do nothing
		{
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		}
		if (autoMode == 2)                                      // 2 totes and drive back
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
				moveLift(.75, 8, false);
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
				if (gyro.getAngle() < 165)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, .4, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
				}
			}
			else if (autoState == 4)
			{
				if (gyro.getAngle() < 185)
				{
					robotDrive.mecanumDrive_Cartesian(0,0,.25,0);
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
				if (rightDrive.getDistance() < 64) 
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
				moveLift(.5, 0, false);
				if (lift.getDistance() <= 0)
					
				{
					moveLift(0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 7)
			{
				if (rightDrive.getDistance() > -10)
				{
					robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
				} 
			} 
			else if (autoState == 8)
			{
				if (gyro.getAngle() > 0)
				{
					robotDrive.mecanumDrive_Cartesian(0,0,-.4,0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
					rightDrive.reset();
				}	
			}
			/*else if (autoState == 9)
			{
				if (rightDrive.getDistance() < 70)
				{
					strafe(0.5);
					//robotDrive.mecanumDrive_Cartesian(0.5, 0.0, 0.0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				} 	
			} */
			
		
		}
		else if (autoMode == 9)			// Pick up yellow tote and drive backwards
		{
			if (autoState == 0)
			{
				moveLift(.25, 4.0, false);
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
				moveLift(.25, 12.0, false);
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
					robotDrive.mecanumDrive_Cartesian(0, .3, 0, 0);
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
				moveLift(.5, 6, false);
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
		else if (autoMode == 3)									//Two Totes and Drive Backwards from LandFill
		{
			if (autoState == 0)                                 //Drive Forward
			{
				if (rightDrive.getDistance() < 10) 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 1)                            // Pick Up totes
			{
				moveLift(.75, 8, false);
				if (lift.getDistance() >= 8)
					
				{
					moveLift(0.0);
					autoState++;
				}
			}
			if (autoState == 2) 
			{
				if (rightDrive.getDistance() > -72)
				{
					robotDrive.mecanumDrive_Cartesian(0, .50, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
		}
		else if (autoMode == 4)                                //2 totes from ramp
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
				moveLift(.75, 8, false);
				if (lift.getDistance() >= 8)
					
				{
					moveLift(0.0);
					autoState++;
				}
			} 
			else if (autoState == 2)                            // Drive Back
			{
				if (rightDrive.getDistance() > -60) 
				{
					robotDrive.mecanumDrive_Cartesian(0, .35, 0, 0);
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
				if (rightDrive.getDistance() < 40)
				{
					strafe(0.45);
					//robotDrive.mecanumDrive_Cartesian(0.5, 0.0, 0.0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			} 
			else if (autoState == 4)
			{
				moveLift(.5, 0, false);
				if (lift.getDistance() <= 0)
					
				{
					moveLift(0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 4)
			{
				if (rightDrive.getDistance() > -10)
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
		else if (autoMode == 10)                                            // 2 bins from step
		{
		
			if (autoState == 0)                                        //Drive backwards towards bins
			{
				if (rightDrive.getDistance() > -8) 
				{
					// driveStraight(.30);
					robotDrive.mecanumDrive_Cartesian(0, .3, 0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 1)                                             //Lower down canThief
			{
				if (canThief.getDistance() >= 72)
				{
					moveThief(0.0);
					if (canThiefTimer.get() > 0.5)
					{
						autoState++;
					}
				}
				else 
				{
					moveThief(.60, 73);
					canThiefTimer.reset();
				}
			
			} 			
			else if (autoState == 2)                                     // Drive forward with bins
			{
				if (rightDrive.getDistance() < 44)
				{
					// driveStraight(-.30);
					robotDrive.mecanumDrive_Cartesian(0, -.3, 0, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
				}
			}	
			else if (autoState == 3)                                  // Drive backwards
			{
				if (rightDrive.getDistance() > -34)
				{
					// driveStraight(.30);
					robotDrive.mecanumDrive_Cartesian(0, .3, 0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
				}
				
				if (rightDrive.getDistance() > -26)
				{
					moveThief(0.0);
				}
				else
				{
					moveThief(.30, 0);
				}
			}
			else if (autoState == 4)                                  //Reset canThief
			{
				moveThief (.30, 0);
				if (canThief.getDistance() <= 0)
				{
					moveThief(0.0);
					autoState++;
				}
			} 
			else if (autoState == 5)
			{
				if (rightDrive.getDistance() < 70)
				{
					// driveStraight(-.30);
					robotDrive.mecanumDrive_Cartesian(0, -.3, 0, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
				}
			}
			else if (autoState == 6)
			{
				if (gyro.getAngle() < 180)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, .4, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
				}
			} 
		}
		else if (autoMode == 11)
		{
			if (autoState == 0)                                        //Drive backwards towards bins
			{
				if (rightDrive.getDistance() > -8) 
				{
					// driveStraight(.30);
					robotDrive.mecanumDrive_Cartesian(0, .3, 0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					autoState++;
					rightDrive.reset();
				}
			}
			else if (autoState == 1)                                             //Lower down canThief
			{
				if (canThief.getDistance() >= 79)
				{
					moveThief(0.0);
					if (canThiefTimer.get() > 0.5)
					{
						autoState++;
					}
				}
				else 
				{
					moveThief(.60, 80);
					canThiefTimer.reset();
				}
			
			} 			
			else if (autoState == 2)                                     // Drive forward with bins
			{
				if (rightDrive.getDistance() < 40)
				{
					// driveStraight(-.30);
					robotDrive.mecanumDrive_Cartesian(0, -.3, 0, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
					rightDrive.reset();
				}
			}	
			else if (autoState == 3)                                  // Drive backwards
			{
				if (rightDrive.getDistance() > -36)
				{
					// driveStraight(.30);
					robotDrive.mecanumDrive_Cartesian(0, .3, 0, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
					rightDrive.reset();
				}
				
				if (rightDrive.getDistance() > -28)
				{
					moveThief(0.0);
				}
				else
				{
					moveThief(.30, 0);
				}
			}
			else if (autoState == 4)                                  //Reset canThief
			{
				moveThief (.30, 0);
				if (canThief.getDistance() <= 0)
				{
					moveThief(0.0);
					autoState++;
				}
			} 
			else if (autoState == 5)
			{
				if (rightDrive.getDistance() < 30)
				{
					// driveStraight(-.30);
					robotDrive.mecanumDrive_Cartesian(0, -.3, 0, 0);
				}
				else 
				{							
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
					rightDrive.reset();
					gyro.reset();
				}
			}
			else if (autoState == 6)
			{
				if (gyro.getAngle() < 180)
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, .4, 0);
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0,0,0,0);
					autoState++;
				}
			}
			else if (autoState == 7)
			{
				
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
				moveLift(.25, 4.0, false);
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
				moveLift(.5, 12, false);
				if (lift.getDistance() >= 12)
					
				{
					moveLift(0.0);
					autoState++;
				}
			}
			else if (autoState == 3)
			{
				if (rightDrive.getDistance() > -22.5) // -13 on practice robot
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
				if (rightDrive.getDistance() < 94)
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
				moveLift(.25, 23, false);
				if (lift.getDistance() >= 23)
				{
					moveLift(0.0);
					autoState++;
					
					rightDrive.reset();
					adjDistance = -updateCamera() / 18.0;
					SmartDashboard.putNumber("AdjDistance: ", adjDistance);
				}
			}
			else if(autoState == 6)
			{
				if ((adjDistance > 0.0) && (rightDrive.getDistance() < adjDistance))
				{
					strafe(0.5);
					//robotDrive.mecanumDrive_Cartesian(0.5, 0.0, 0.0, gyro.getAngle());
				}
				else if ((adjDistance < 0.0) && (rightDrive.getDistance() > adjDistance))
				{
					strafe(-0.5);
					//robotDrive.mecanumDrive_Cartesian(-0.5, 0.0, 0.0, gyro.getAngle());
				}
				else 
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				rightDrive.reset();
					autoState++;
				}
			}
			else if (autoState == 7) 
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
			
		/*	else if (autoState == 7)
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
			} */
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
					moveLift(.75, 20, false);
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
					moveLift(.75, 11, false);
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
					moveLift(.75, 0, false);
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
					moveLift(.75, 6, false);
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
		if (operatorstick.getRawButton(8))
		{
			updateCamera();
		}
		
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
			moveLift(.75, 0, true);
			moveCan (1, 92);
			//moveCan (1, 90);      Practice Robot
		}
		else if (operatorstick.getRawButton(2))
		{
			moveLift(.75, 0, true);
			moveCan(1, 70);
			//moveCan (1, 69);     Practice Robot
		}
		else if (operatorstick.getRawButton(3))
		{
			moveLift(.75, 2.4, true);
			//moveLift(.75, 2.7, true);     Practice Robot
			moveCan (1, 60);
		}
		else if (operatorstick.getRawButton(4))
		{
			moveLift(.75, 0, true);
			moveCan(1, 110);
		}
		else 
		{
			// Drive Container Lift
			// ---------------------------------------------------------------
			
			double containerLift = -operatorstick.getRawAxis(3) * 0.70;
		      
			if (operatorstick.getRawButton(7))
			{
				moveLift(0.50, 11, false);
			} 
			else if (operatorstick.getRawButton(5))
			{
				moveLift(.50, 24.5, false);
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
				if (operatorstick.getRawButton(6))
				{
					moveThief(canLift);
				}
				else
				{
					moveCan(canLift);
				}
			} 
			else if ((canLift < 0) && (canLift < -deadzone)) 
			{
				if (operatorstick.getRawButton(6)) 
				{
					moveThief(canLift);  
				} else 
				{
					moveCan(canLift); 
				}

			} 
			else 
			{
				moveCan(0);
				moveThief(0);
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
		if (leftstick.getRawAxis(2) >= .5 && rightstick.getRawAxis(2) > .5)     // remove rightstick.getRawAxis(2) >.5
		{
			CurrentAuto = "Do Nothing";
			return 1;
		}
		else if (leftstick.getRawAxis(2) >= .5 && rightstick.getRawAxis(2) < .5) // remove all of this
		{
			CurrentAuto = "Two Can from Step auto";
			return 10;
		}
		else if (leftstick.getRawAxis(2) < .5 && leftstick.getRawAxis(2) > 0) 
		{
			CurrentAuto = "2 Totes & Drive Back ";
			return 2;
		} 
		else if (leftstick.getRawAxis(2) < 0 && leftstick.getRawAxis(2) > -.5)
		{
			CurrentAuto = "Drive Back From Landfill";
			return 3;
		}
		else
		{
			CurrentAuto = "2 Totes and Drive Back From Ramp";
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
		SmartDashboard.putNumber("canThief Encoder Value", canThief.get());
		SmartDashboard.putBoolean("canThief Limit", canThiefLimit.get());
	}
	
	public double updateCamera()
		{
			NIVision.Rect rect = new NIVision.Rect(10,10,100,100);
			//read file in from disk. For this example to run you need to copy image20.jpg from the SampleImages folder to the
			//directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
			NIVision.IMAQdxGrab(session, frame, 1);


			//Update threshold values from SmartDashboard. For performance reasons it is recommended to remove this after calibration is finished.
			TOTE_HUE_RANGE.minValue = (int)SmartDashboard.getNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
			TOTE_HUE_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
			TOTE_SAT_RANGE.minValue = (int)SmartDashboard.getNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
			TOTE_SAT_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
			TOTE_VAL_RANGE.minValue = (int)SmartDashboard.getNumber("Tote val min", TOTE_VAL_RANGE.minValue);
			TOTE_VAL_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote val max", TOTE_VAL_RANGE.maxValue);

			//Threshold the image looking for yellow (tote color)
			 NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

			//Send particle count to dashboard
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 8);
			SmartDashboard.putNumber("Masked particles", numParticles);

			//Send masked image to dashboard to assist in tweaking mask.
			CameraServer.getInstance().setImage(binaryFrame);
			//CameraServer.getInstance().setImage(frame);

			//filter out small particles
			float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
			criteria[0].lower = areaMin;
			imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

			//Send particle count after filtering to dashboard
			numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Filtered particles", numParticles);

			if(numParticles > 0)
			{
				//Measure particles and sort by particle size
				Vector<ParticleReport> particles = new Vector<ParticleReport>();
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par = new ParticleReport();
					par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					par.ConvexHullArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
					par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
					particles.add(par);
				}
				particles.sort(null);

				//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
				//for the reader. Note that the long and short side scores expect a single tote and will not work for a stack of 2 or more totes.
				//Modification of the code to accommodate 2 or more stacked totes is left as an exercise for the reader.
				/*scores.Trapezoid = TrapezoidScore(particles.elementAt(0));
				SmartDashboard.putNumber("Trapezoid", scores.Trapezoid);
				scores.LongAspect = LongSideScore(particles.elementAt(0));
				SmartDashboard.putNumber("Long Aspect", scores.LongAspect);
				scores.ShortAspect = ShortSideScore(particles.elementAt(0));
				SmartDashboard.putNumber("Short Aspect", scores.ShortAspect);
				scores.AreaToConvexHullArea = ConvexHullAreaScore(particles.elementAt(0));
				SmartDashboard.putNumber("Convex Hull Area", scores.AreaToConvexHullArea);
				boolean isTote = scores.Trapezoid > SCORE_MIN && (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN) && scores.AreaToConvexHullArea > SCORE_MIN;
				boolean isLong = scores.LongAspect > scores.ShortAspect; 

				//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
				SmartDashboard.putBoolean("IsTote", isTote);
				SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.elementAt(0), isLong)); */
				String rectv2 = particles.elementAt(0).BoundingRectTop + ", " + particles.elementAt(0).BoundingRectLeft + ", " 
						+ particles.elementAt(0).BoundingRectBottom + ", " + particles.elementAt(0).BoundingRectRight;
				SmartDashboard.putString("Rect", rectv2);
				
				return 590.0 - particles.elementAt(0).BoundingRectRight;
			} 
			else 
			{
				SmartDashboard.putBoolean("IsTote", false);
			}
			
			return 0.0;
	}

	//Comparator function for sorting particles. Returns true if particle 1 is larger
	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}

	/**
	 * Method to score convex hull area. This scores how "complete" the particle is. Particles with large holes will score worse than a filled in shape
	 */
	double ConvexHullAreaScore(ParticleReport report)
	{
		return ratioToScore((report.Area/report.ConvexHullArea)*1.18);
	}

	/**
	 * Method to score if the particle appears to be a trapezoid. Compares the convex hull (filled in) area to the area of the bounding box.
	 * The expectation is that the convex hull area is about 95.4% of the bounding box area for an ideal tote.
	 */
	double TrapezoidScore(ParticleReport report)
	{
		return ratioToScore(report.ConvexHullArea/((report.BoundingRectRight-report.BoundingRectLeft)*(report.BoundingRectBottom-report.BoundingRectTop)*.954));
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the long side of a tote.
	 */
	double LongSideScore(ParticleReport report)
	{
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/LONG_RATIO);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the short side of a tote.
	 */
	double ShortSideScore(ParticleReport report){
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/SHORT_RATIO);
	}

	/**
	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 *
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param isLong Boolean indicating if the target is believed to be the long side of a tote
	 * @return The estimated distance to the target in feet.
	 */
	double computeDistance (Image image, ParticleReport report, boolean isLong) {
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/size.width;
		targetWidth = isLong ? 26.0 : 16.9;

		return  targetWidth/(normalizedWidth*12*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
	} // xxyyzz
	
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

	public void moveThief(double speed)
	{
		if (speed > 0) 
		{
			moveThief(speed, 110);
			
		} else 
		{

			if (canThiefLimit.get())
			{
				moveThief(-speed, -360);
			}
			else 
			{
				moveThief(0, 0);
			}		
		}
	}
	
	public void moveThief(double speed, double target)
	{
		if (canThief.get() < target -1)
		{
			canBack.set(speed);
		}
		else if (canThief.get() > target + 1)
		{
			canBack.set(-speed);
		}
		else
		{
			canBack.set(0);
		}
		
		if (!canThiefLimit.get())
		{
			canThief.reset();
		}
	}
	
	public void moveCan (double speed)
	{
		if (speed > 0) 
		{
			moveCan(speed, 110);
			
		} 
		else 
		{ 
			if (canLimitUp.get())
			{
				moveCan(-speed, -360);
			}
			else 
			{
				moveCan(0, 0);
			}
		}
	}
	
	public void moveCan(double speed, double target)
	{
		if (canArm.get() < target - 1) // -2 on practice robot
		{
			//canBack.set(speed);
			canRight.set(-speed);
		}
		else if (canArm.get() > target + 1) // +2 on practice robot
		{ 
			//canBack.set(-speed);
			canRight.set(speed);
		}
		else
		{
			//canBack.set(0);
			canRight.set(0);
		}
		
		if (!canLimitUp.get())
		{
			canArm.reset();
		}
	}
	
	public void moveLift(double speed) {
		if (speed > 0) {
			moveLift(speed, 100, false);
			
		} else {
			moveLift(-speed, -100, false);		
		}
	}
    
	public void moveLift(double speed, double target, boolean deadzone) {
		
		
		SmartDashboard.putNumber("LiftSpeed", speed);
		SmartDashboard.putNumber("LiftTarget", target);
		
		if ((deadzone && liftLimitUp.get() && lift.getDistance() < target - .15) || (!deadzone && liftLimitUp.get() && lift.getDistance() < target)) 
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
		else if ((deadzone && liftLimitDown.get() && lift.getDistance() > target + .15) || (!deadzone && liftLimitDown.get() && lift.getDistance() > target)) 
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
