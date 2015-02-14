package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team2522.robot.*;

public class LEDController {
	private LEDOutput rLED;
	private LEDOutput gLED;
	private LEDOutput bLED;
	private LEDUtil.Color color;
	
	public LEDController(LEDOutput r, LEDOutput g, LEDOutput b) {
		this.rLED = r;
		this.gLED = g;
		this.bLED = b;
	}
	
	public LEDController(DigitalOutput r, DigitalOutput g, DigitalOutput b) {
		this(new LEDOutput(r), new LEDOutput(g), new LEDOutput(b));
	}
	
	public LEDController(PWM r, PWM g, PWM b) {
		this(new LEDOutput(r), new LEDOutput(g), new LEDOutput(b));
	}
	
	public LEDController(Solenoid r, Solenoid g, Solenoid b) {
		this(new LEDOutput(r), new LEDOutput(g), new LEDOutput(b));
	}
	
	public LEDController(Relay r, Relay g, Relay b) {
		this(new LEDOutput(r), new LEDOutput(g), new LEDOutput(b));
	}
	
	public void setColor(LEDUtil.Color color) {
		this.color = color;
		rLED.set(color.hasRed());
		gLED.set(color.hasGreen());
		bLED.set(color.hasBlue());
	}
	
	public LEDUtil.Color getColor() {
		return color;
	}
}
