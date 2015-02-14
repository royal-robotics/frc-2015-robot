package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.*;

public class LEDOutput {
	private Object output;
	private boolean ledOn;
	
	public LEDOutput(DigitalOutput out) {
		out.set(false);
		output = out;
	}
	
	public LEDOutput(PWM out) {
		out.setRaw(0);
		output = out;
	}
	
	public LEDOutput(Solenoid out) {
		out.set(true);
		output = out;
	}
	
	public LEDOutput(Relay out) {
		out.set(Relay.Value.kOn);
		output = out;
	}
	
	public void set(boolean on) {
		ledOn = on;
		if(output instanceof DigitalOutput) {
			DigitalOutput led = (DigitalOutput) output;
			led.set(ledOn);
		} else if(output instanceof PWM) {
			PWM led = (PWM) output;
			if(ledOn) {
				led.setRaw(255);
			} else {
				led.setRaw(0);
			}
		} else if(output instanceof Solenoid) {
			Solenoid led = (Solenoid) output;
			led.set(ledOn);
		} else if(output instanceof Relay) {
			if(on) {
				((Relay) output).set(Relay.Value.kOn);
			} else {
				((Relay) output).set(Relay.Value.kOff);
			}
		}
	}
	
	public boolean get() {
		return ledOn;
	}
}
