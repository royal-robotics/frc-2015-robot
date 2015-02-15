package org.usfirst.frc.team2522.robot;

import org.usfirst.frc.team2522.robot.LEDUtil.Color;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends IterativeRobot {

	RobotDrive robotDrive;

	// Joystick
	// -------------------------------------------------------------------
	Joystick leftstick;
	Joystick rightstick;
	Joystick operatorstick;

	// VICTORSP MOTOR CONTORL
	// -------------------------------------------------------------------
	VictorSP frontRightChannel;
	VictorSP rearRightChannel;
	VictorSP frontLeftChannel;
	VictorSP rearLeftChannel;
	VictorSP liftLeft;
	VictorSP liftRight;
	Talon canRight;
	Talon canLeft;

	// DIO
	// -------------------------------------------------------------------

	DigitalInput limitUp;
	DigitalInput limitDown;
	DigitalOutput tests;
	/*
	 * DigitalOutput ledBox1; DigitalOutput ledBox2; DigitalOutput ledBox3;
	 */
	LEDController leds;
	DigitalOutput redLED;
	DigitalOutput greenLED;
	DigitalOutput blueLED;
	CameraServer server;

	public void robotInit() {

		server = CameraServer.getInstance();
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");
		// Encoder
		// ---------------------------------------------------------------
		// LED
		// ---------------------------------------------------------------
		// redLED = new DigitalOutput(0);
		// greenLED = new DigitalOutput(19);
		// blueLED = new DigitalOutput(2);

		frontRightChannel = new VictorSP(0);
		frontLeftChannel = new VictorSP(1);
		rearRightChannel = new VictorSP(2);
		rearLeftChannel = new VictorSP(3);
		liftRight = new VictorSP(4);
		liftLeft = new VictorSP(5);
		canRight = new Talon(6);
		canLeft = new Talon(7);

		// Init Joysticks
		// ---------------------------------------------------------------
		leftstick = new Joystick(0);
		rightstick = new Joystick(1);
		operatorstick = new Joystick(2);

		limitUp = new DigitalInput(3);
		limitDown = new DigitalInput(4);

		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel,
				frontRightChannel, rearRightChannel);
		// robotDrive = new RobotDrive(rearLeftChannel, frontLeftChannel,
		// frontRightChannel, rearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);

		// leds = new LEDController(new Relay(0), new Relay(1), new Relay(2));
		leds = new LEDController(new DigitalOutput(16), new DigitalOutput(18),
				new DigitalOutput(19));
	}

	public void teleopPeriodic() {

		if (leftstick.getRawButton(4)) {
			leds.setColor(LEDUtil.Color.RED);
		}
		if (leftstick.getRawButton(3)) {
			leds.setColor(LEDUtil.Color.GREEN);
		}
		if (leftstick.getRawButton(5)) {
			leds.setColor(LEDUtil.Color.BLUE);
		}
		if (leftstick.getRawButton(2))
		{
			leds.setColor(LEDUtil.Color.OFF);
		}

		// redLED.set(leftstick.getRawButton(4));
		// greenLED.set(leftstick.getRawButton(3));
		// blueLED.set(leftstick.getRawButton(5));
		// Lift
		// ---------------------------------------------------------------
		if (operatorstick.getRawButton(4) && limitUp.get()) {
			liftLeft.set(-1);
			liftRight.set(1);
		} else if (operatorstick.getRawButton(2) && limitDown.get()) {
			liftLeft.set(1);
			liftRight.set(-1);
		} else {
			liftLeft.set(0);
			liftRight.set(0);
		}

		if (operatorstick.getRawButton(3)) {
			canLeft.set(-.25);
			canRight.set(.25);
		} else if (operatorstick.getRawButton(1)) {
			canLeft.set(.50);
			canRight.set(-.50);
		} else {
			canLeft.set(0);
			canRight.set(0);
		}
		/*
		 * if(limitUp.get()) { liftLeft.set(0); liftRight.set(0); }
		 */
		// DeadZone Value
		// ---------------------------------------------------------------
		double deadzone = 0.1;

		// Magnitude for Mecanum
		// ---------------------------------------------------------------
		double magnitude = rightstick.getMagnitude(); // Magnitude of Drive
		if (magnitude < deadzone) // Dead Zone
		{
			magnitude = 0;
		}

		// Direction for Mecanum
		// ---------------------------------------------------------------
		// double direction = rightstick.getDirectionDegrees(); //Direction of
		// Drive
		// if(direction > -deadzone && direction < deadzone)
		// {
		// direction = 0;
		// }

		// Rotation for Mecanum
		// ---------------------------------------------------------------
		double rotation = rightstick.getX();
		double leftX = leftstick.getX();
		double leftY = leftstick.getY();
		if (rotation > -deadzone && rotation < deadzone) {
			rotation = 0;
		}
		if (leftX > -deadzone && leftX < deadzone) {
			leftX = 0;
		}
		if (leftY > -deadzone && leftY < deadzone) {
			leftY = 0;
		}
		// Mecanum Polar PARAMETERS(magnitude, direction, rotation, gyro);
		// ---------------- -----------------------------------------------

		robotDrive.mecanumDrive_Cartesian(leftX, leftY, rotation /*
																 * leftstick.getX
																 * ()
																 */, 0.0);

	}
}
