package org.usfirst.frc.team2522.robot;

public class LEDUtil {
	public enum Color {
		OFF(false, false, false),
		RED(true, false, false),
		GREEN(false, true, false),
		BLUE(false, false, true),
		YELLOW(true, true, false),
		MAGENTA(true, false, true),
		CYAN(false, true, true),
		WHITE(true, true, true);
		
		private boolean red;
		private boolean green;
		private boolean blue;
		
		private Color (boolean r, boolean g, boolean b) {
			this.red = r;
			this.green = g;
			this.blue = b;
		}
		
		public boolean hasRed() {
			return red;
		}
		
		public boolean hasGreen() {
			return green;
		}
		
		public boolean hasBlue() {
			return blue;
		}
	}
}
