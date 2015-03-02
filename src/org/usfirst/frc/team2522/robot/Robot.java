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
	DigitalInput canLimitDown;

	// Encoders
	// ---------------------------------------------------------------
	Encoder rightDrive;
	Encoder leftDrive;
	Encoder lift;

	Gyro gyro;
	// LEDs
	// ---------------------------------------------------------------
	LEDController leds;

	// Camera
	// ---------------------------------------------------------------
	CameraServer server;

	// Misc.
	// ---------------------------------------------------------------
	DriverStation ds;
	Relay canHolder;
	Timer holderTimer;
	Boolean isIn = true;
	Boolean resetButton = true;
	Boolean liftUp = true;
	Boolean seenBottom = false;
	
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
		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel,
				frontRightChannel, rearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);

		// Init Limit Switches
		// ---------------------------------------------------------------
		liftLimitUp = new DigitalInput(0);
		liftLimitDown = new DigitalInput(1);
		canLimitUp = new DigitalInput(2);
		canLimitDown = new DigitalInput(3);

		gyro = new Gyro(0);
		// Init Encoders
		// ---------------------------------------------------------------
		lift = new Encoder(new DigitalInput(4), new DigitalInput(5));
		rightDrive = new Encoder(new DigitalInput(6), new DigitalInput(7));
		leftDrive = new Encoder(new DigitalInput(9), new DigitalInput(8));

		lift.setDistancePerPulse(1 / LIFT_PULSES_PER_INCH);
		rightDrive.setDistancePerPulse(1 / WHEEL_PULSES_PER_INCH);
		leftDrive.setDistancePerPulse(1 / WHEEL_PULSES_PER_INCH);

		lift.reset();
		rightDrive.reset();
		leftDrive.reset();

		// Init LEDs
		// ---------------------------------------------------------------
		leds = new LEDController(new DigitalOutput(16), new DigitalOutput(18),
				new DigitalOutput(19));
		// Init Camera
		// ---------------------------------------------------------------
		server = CameraServer.getInstance();
		server.startAutomaticCapture("cam0");
		
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

	public void autonomousInit() {
		autoState = 0;
		
		gyro.initGyro();
		rightDrive.reset();
		lift.reset();
		
		liftUp = false;
	}
	//rotation and gyro positive = right
	//rotation and gyro negative = left
	public void autonomousPeriodic() {

		
		if (leftstick.getRawAxis(2) >= .25)	// Autonomus1
		{
			if (autoState == 0 && rightDrive.getDistance() > -110)
			{
				robotDrive.mecanumDrive_Cartesian(0, .75, 0, 0);
			}
			else
			{
				robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoState++;
			}
		}
		else if (leftstick.getRawAxis(2) <= -.25)	// Autonomus 2
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
				if (rightDrive.getDistance() > -115)
				{
					robotDrive.mecanumDrive_Cartesian(0, .75, 0, 0);
				}
				else
				{
					robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoState++;
				}
			}
		}
		else											//Autonomous 3
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
					leftDrive.reset();
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
					leftDrive.reset();
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
					leftDrive.reset();
					rightDrive.reset();
				}
			}
			else if (autoState == 7)
			{
				moveLift(-.30, 12);
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
					leftDrive.reset();
					rightDrive.reset();
				}
			}
			else if (autoState == 8)
			{
				moveLift(-.25, 3.0);
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
					leftDrive.reset();
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
					leftDrive.reset();
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
					leftDrive.reset();
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
					leftDrive.reset();
				}
			}
			else if (autoState == 15)
			{
				moveLift(-0.25, 3.0);
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
					leftDrive.reset();
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
					leftDrive.reset();
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
					leftDrive.reset();
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
		}
		
		dashboardOutput(0, autoState);
	}
	
	public void teleopInit() 
	{
		lift.reset();
		seenBottom = false;
	}
	
	public void teleopPeriodic() {
		
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
		
		


		//Drive power adjustment
		
		double drivePower = .5;
		
		if (leftstick.getRawButton(1) || rightstick.getRawButton(1)) {
			if (leftstick.getRawButton(1) && rightstick.getRawButton(1)) {
				drivePower = 1;
			} else {
				drivePower = .75;
			}
		} 
		
		// DeadZone Value
		// ---------------------------------------------------------------
		double deadzone = 0.15;

		// Drive Container Lift
		// ---------------------------------------------------------------
		double containerLift = -operatorstick.getRawAxis(3) * 0.75;
       
		moveLift(containerLift);


		// Drive Can Lift
		// ---------------------------------------------------------------
		double canLift = operatorstick.getY();
		if ((canLift > 0) && (canLift > deadzone)) {
			canLeft.set(-canLift * 0.25);
			canRight.set(canLift * 0.25);
		} else if ((canLift < 0) && (canLift < -deadzone) && canLimitUp.get()) {
			if (operatorstick.getRawButton(6)) {
				canLeft.set(-canLift / 2);
				canRight.set(canLift / 2);
			} else {
				canLeft.set(-canLift);
				canRight.set(canLift);
			}

		} else {
			canLeft.set(0.0);
			canRight.set(0.0);
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
	
		dashboardOutput(canLift, 0);
	}

	public void dashboardOutput(double canLift, int autoState)
	{
		if (leftstick.getRawAxis(2) >= .25)
		{
			CurrentAuto = "Autonomus 1";
		}
		else if (leftstick.getRawAxis(2) <= -.25) 
		{
			CurrentAuto = "Autonomus 2";
		}
		else
		{
			CurrentAuto = "Autonomus 3";
		}
	
		SmartDashboard.putNumber("Lift Encoder Value: ", lift.getDistance());
		SmartDashboard.putNumber("Right Drive Encoder Value: ", rightDrive.getDistance());
		SmartDashboard.putNumber("Left Drive Encoder Value: ", leftDrive.getDistance());
		SmartDashboard.putNumber("Can Lift Speed: ", canLift);
		SmartDashboard.putBoolean("LiftUp", liftLimitUp.get());
		SmartDashboard.putBoolean("LiftDown", liftLimitDown.get());
		SmartDashboard.putBoolean("CanUp", canLimitUp.get());
		SmartDashboard.putBoolean("CanDown", canLimitDown.get());
		SmartDashboard.putNumber("Autonomous Mode", leftstick.getRawAxis(2));
		SmartDashboard.putString("Current Autonomus", (String) CurrentAuto);
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		SmartDashboard.putNumber("AutoState", autoState );
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
	public void moveCan(double speed, int encoder, boolean ignoreLimit) {

	}
	
	public void moveLift(double speed) {
		if (speed > 0) {
			moveLift(speed, 100);
			
		} else {
			moveLift(speed, -100);		
		}
	}
    
	public void moveLift(double speed, double target) {
		
		SmartDashboard.putNumber("LiftSpeed", speed);
		SmartDashboard.putNumber("LiftTarget", target);
		
		if ((speed > 0) && liftLimitUp.get() && lift.getDistance() < target) 
		{
			liftLeft.set(-speed);
			liftRight.set(speed);
		}
		else if ((speed < 0) && liftLimitDown.get() && lift.getDistance() > target) 
		{
			if (seenBottom && lift.getDistance() < 0) 
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
	}
}
