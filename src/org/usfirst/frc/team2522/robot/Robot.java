package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
public class Robot extends SampleRobot {
	
    RobotDrive robotDrive;
    Joystick stick;
    DigitalOutput ledBox1;
    DigitalOutput ledBox2; 
    DigitalOutput ledBox3;
    
    // Channels for the wheels
    final int frontLeftChannel	= 2;
    final int rearLeftChannel	= 3;
    final int frontRightChannel	= 1;
    final int rearRightChannel	= 0;
    
    // The channel on the driver station that the joystick is connected to
    final int joystickChannel	= 0;

    public Robot() {
        robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
        robotDrive.setExpiration(0.1);
    	robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
    	
        stick = new Joystick(joystickChannel);
        
        ledBox1 = new DigitalOutput(0);
        ledBox2 = new DigitalOutput(1);
        ledBox3 = new DigitalOutput(2);
  
    }
    /**
     * Runs the motors with Mecanum drive.
     */
    public void operatorControl() {  
        robotDrive.setSafetyEnabled(true);
        
        //LED TEST
        //-------------------
        ledBox1.set(true);
        Timer.delay(5.0);
        ledBox1.set(false);
        Timer.delay(1.0);
        ledBox2.set(true);
        Timer.delay(5.0);
        ledBox2.set(false);
        Timer.delay(1.0);
        ledBox3.set(true);
        Timer.delay(5.0);
        ledBox3.set(false);
        Timer.delay(1.0);
        ledBox1.set(true);
        ledBox2.set(true);
        ledBox3.set(true); 
        //-------------------
        
        while (isOperatorControl() && isEnabled()) {
        	
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), 0);
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
            
            
        }
    }
    
}
