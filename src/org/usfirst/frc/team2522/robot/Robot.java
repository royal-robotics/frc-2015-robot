package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	RobotDrive robotDrive;

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
		lift = new Encoder(new DigitalInput(4), new DigitalInput(5));
		rightDrive = new Encoder(new DigitalInput(6), new DigitalInput(7));
		leftDrive = new Encoder(new DigitalInput(8), new DigitalInput(9));
		
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
	}

	public void teleopPeriodic() {
		// Set LED's
		// ---------------------------------------------------------------
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
		
		// DeadZone Value
		// ---------------------------------------------------------------
		double deadzone = 0.1;
		
		// Drive Container Lift
		// ---------------------------------------------------------------
		if (operatorstick.getRawButton(4) && !liftLimitUp.get()) {
			liftLeft.set(-1);
			liftRight.set(1);
		} else if (operatorstick.getRawButton(2) && !liftLimitDown.get()) {
			liftLeft.set(1);
			liftRight.set(-1);
		} else {
			liftLeft.set(0);
			liftRight.set(0);
		}
		
		// Drive Can Lift
		// ---------------------------------------------------------------
		if (operatorstick.getY() < deadzone && operatorstick.getY() > -deadzone) {
			canLeft.set(0);
			canRight.set(0);
		} else {
			canLeft.set(-operatorstick.getY());
			canRight.set(operatorstick.getY());
		}

		// Magnitude for Mecanum
		// ---------------------------------------------------------------
		double magnitude = rightstick.getMagnitude(); // Magnitude of Drive
		if (magnitude < deadzone) // Dead Zone
		{
			magnitude = 0;
		}
		
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
		robotDrive.mecanumDrive_Cartesian(leftX, leftY, rotation, 0.0);
		
		// Encoder Test
		SmartDashboard.putNumber("Encoder Value: ", lift.get());
	}
}
