package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
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

public class Robot extends IterativeRobot {

	RobotDrive robotDrive;

	public static final double PULSES_PER_FOOT = 171.0;
	public static final double ButtonDelay = 0.5;
	// public static final int Swag = 1337;
	public String CurrentAuto = " ";
	// Joy sticks
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
	Relay canHolder;
	Timer holderTimer;
	Boolean isIn = true;
	Boolean resetButton = true;
	Boolean liftUp = true;
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

	// LEDs
	// ---------------------------------------------------------------
	LEDController leds;
	// Camera
	// ---------------------------------------------------------------
	CameraServer server;

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

		// Init Encoders
		// ---------------------------------------------------------------
		lift = new Encoder(new DigitalInput(5), new DigitalInput(4));
		rightDrive = new Encoder(new DigitalInput(6), new DigitalInput(7));
		leftDrive = new Encoder(new DigitalInput(9), new DigitalInput(8));

		lift.setDistancePerPulse(1 / 7009);
		rightDrive.setDistancePerPulse(1 / PULSES_PER_FOOT);
		leftDrive.setDistancePerPulse(1 / PULSES_PER_FOOT);

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

		// Init SmartDashboard
		// ---------------------------------------------------------------
		SmartDashboard.putNumber("Encoder Value: ", lift.get());
		SmartDashboard.putNumber("Right Drive Encoder Value: ",
				rightDrive.get());
		SmartDashboard.putNumber("Left Drive Encoder Value: ", leftDrive.get());
		SmartDashboard.putNumber("Can Lift Speed: ", 0.0);
	}

	public void disabledPeriodic() {
		leds.setColor(LEDUtil.Color.WHITE);
		dashboardOutput(operatorstick.getY());
	}

	int autoState = 0;

	public void autonomousInit() {
		autoState = 0;

		rightDrive.reset();
		/*
		 * while(rightDrive.getDistance() < 9.0) {
		 * robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
		 * SmartDashboard.putNumber("autnomousLeft",leftDrive.getDistance());
		 * SmartDashboard.putNumber("autonomousRight",rightDrive.getDistance());
		 * 
		 * } robotDrive.mecanumDrive_Cartesian(0,0,0,0);
		 */
		/*
		 * while(liftLimitUp.get()) { liftLeft.set(-0.75); liftRight.set(0.75);
		 * } liftLeft.set(0); liftRight.set(0); while(rightDrive.getDistance() <
		 * -4.0) { robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
		 * SmartDashboard.putNumber("autnomousLeft",leftDrive.getDistance());
		 * SmartDashboard.putNumber("autonomousRight",rightDrive.getDistance());
		 * 
		 * } robotDrive.mecanumDrive_Cartesian(0,0,0,0);
		 */
		liftUp = false;
		SmartDashboard.putBoolean("LiftLimitHit", false);
	}

	public void autonomousPeriodic() {

		SmartDashboard.putNumber("autnomousLeft", leftDrive.getDistance());
		SmartDashboard.putNumber("autonomousRight", rightDrive.getDistance());
		// Autonomus1
		if (autoState == 0 && leftstick.getRawAxis(2) > 0) {
			if (rightDrive.getDistance() < 0.5) {
				robotDrive.mecanumDrive_Cartesian(0.0, -.3, 0.0, 0.0);
				// SmartDashboard.putNumber("autnomousLeft",leftDrive.getDistance());
				// SmartDashboard.putNumber("autnomousRight",rightDrive.getDistance());
			} else {
				robotDrive.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
				autoState++;
			}
		} else if (autoState == 1 && leftstick.getRawAxis(2) > 0) {
			if (liftLimitUp.get()) {
				if (!liftUp) {
					liftLeft.set(-0.25);
					liftRight.set(0.25);
				}
			} else {
				SmartDashboard.putBoolean("LiftLimitHit", true);
				liftUp = true;
				liftLeft.set(0.0);
				liftRight.set(0.0);
			}

			if (rightDrive.getDistance() > -8.0 && liftUp) {
				robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
				// SmartDashboard.putNumber("autnomousLeft",leftDrive.getDistance());
				// SmartDashboard.putNumber("autonomousRight",rightDrive.getDistance());

			} else {
				robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				if (liftUp) {
					autoState++;
				}
			}
		}
		// Autonomus1 end

		// Autonomus2
		if (autoState == 0 && leftstick.getRawAxis(2) < 0) {
			if (rightDrive.getDistance() < 20) {
				robotDrive.mecanumDrive_Cartesian(.75, 0, 0, 0);
			} else {
				robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoState++;

			}
		}
	}

	public void teleopPeriodic() {
		// Set LED's
		// ---------------------------------------------------------------
		double drivePower = .5;

		if (leftstick.getRawButton(4)) {
			leds.setColor(LEDUtil.Color.RED);
		}

		if (leftstick.getRawButton(3)) {
			leds.setColor(LEDUtil.Color.GREEN);
		}

		if (leftstick.getRawButton(5)) {
			leds.setColor(LEDUtil.Color.BLUE);
		}

		if (leftstick.getRawButton(2)) {
			leds.setColor(LEDUtil.Color.OFF);
		}

		if (leftstick.getRawButton(1)) {

		}

		if (leftstick.getRawButton(1) || rightstick.getRawButton(1)) {
			if (leftstick.getRawButton(1) && rightstick.getRawButton(1)) {
				drivePower = 1;
			} else {
				drivePower = .75;
			}
		} else {
			drivePower = .5;
		}
		// leds.setColor(LEDUtil.Color.GREEN);
		// DeadZone Value
		// ---------------------------------------------------------------
		double deadzone = 0.15;

		// Drive Container Lift
		// ---------------------------------------------------------------
		if (operatorstick.getRawButton(4) && liftLimitUp.get()) {
			liftLeft.set(-0.75);
			liftRight.set(0.75);
		} else if (operatorstick.getRawButton(2) && liftLimitDown.get()) {
			liftLeft.set(0.65);
			liftRight.set(-0.65);
		} else {
			liftLeft.set(0);
			liftRight.set(0);
		}

		if (!liftLimitDown.get()) {
			lift.reset();
		}

		/*
		 * double liftSpeed = operatorstick.getY(); if((canLift > 0) && (canLift
		 * > deadzone)) { canLeft.set(-canLift*0.25);
		 * canRight.set(canLift*0.25); }
		 */
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

		if (operatorstick.getRawButton(5) && resetButton) {
			if (isIn) {
				canHolder.set(Value.kForward);
				isIn = false;
				Timer.delay(ButtonDelay);
				canHolder.set(Value.kReverse);
				isIn = true;
				Timer.delay(ButtonDelay);
			}
			resetButton = false;
		} else {
			canHolder.set(Value.kOff);
			resetButton = true;
		}
		/*
		 * if (operatorstick.getRawButton(6)) { canHolder.set(Value.kForward); }
		 * else if(operatorstick.getRawButton(8)) {
		 * canHolder.set(Value.kReverse); } else { canHolder.set(Value.kOff); }
		 */

		// Magnitude for Mecanum
		// ---------------------------------------------------------------
		/*
		 * double magnitude = rightstick.getMagnitude(); // Magnitude of Drive
		 * if (magnitude < deadzone) // Dead Zone { magnitude = 0; }
		 * 
		 * if (leftstick.getTrigger() && rightstick.getTrigger()) {
		 * SmartDashboard.putNumber("Drive Multiplier: ", 1); } else if
		 * (leftstick.getTrigger() || rightstick.getTrigger()) { magnitude =
		 * magnitude * 0.75; SmartDashboard.putNumber("Drive Multiplier: ",
		 * .75); } else { magnitude = magnitude * 0.50;
		 * SmartDashboard.putNumber("Drive Multiplier: ", .5); }
		 * 
		 * SmartDashboard.putNumber("Drive Magnitude: ", magnitude);
		 */

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
		robotDrive.mecanumDrive_Cartesian(leftX * drivePower, leftY
				* drivePower, rotation * drivePower, 0.0);

		// Encoder Test
		dashboardOutput(canLift);
	}

	public void dashboardOutput(double canLift) {
		if (leftstick.getRawAxis(2) > 0)
			;
		{
			CurrentAuto = "Autonomus 1";
		}
		if (leftstick.getRawAxis(2) < 0) {
			CurrentAuto = "Autonomus 2";
		}

		SmartDashboard.putNumber("Lift Encoder Value: ", lift.get());
		SmartDashboard.putNumber("Right Drive Encoder Value: ",
				rightDrive.getDistance());
		SmartDashboard.putNumber("Left Drive Encoder Value: ",
				leftDrive.getDistance());
		SmartDashboard.putNumber("Can Lift Speed: ", canLift);
		SmartDashboard.putBoolean("LiftUp", liftLimitUp.get());
		SmartDashboard.putBoolean("LiftDown", liftLimitDown.get());
		SmartDashboard.putBoolean("CanUp", canLimitUp.get());
		SmartDashboard.putBoolean("CanDown", canLimitDown.get());
		SmartDashboard.putNumber("Autonomous Mode", leftstick.getRawAxis(2));
		SmartDashboard.putString("Current Autonomus", (String) CurrentAuto);

		// SmartDashboard.putNumber("ROYAL ROBOTICS SWAG AMOUNT", Swag);
	}
}
