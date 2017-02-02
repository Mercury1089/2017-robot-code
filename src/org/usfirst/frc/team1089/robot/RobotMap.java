package org.usfirst.frc.team1089.robot;

/**
 * The {@code RobotMap} class contains constants for all the components' connected ports.
 * It is is a mapping from the ports sensors and actuators are wired into. There are subclasses
 * separating the types of ports for each subset of the robot.
 * This provides flexibility changing wiring, makes checking the wiring easier, and significantly
 * reduces use of magic numbers.
 */
public class RobotMap {
	/**
	 * The {@code Analog} subclass contains all the ports for anything giving an analog output.
	 * The only thing in this class is the gyro's port.
	 */
	public static class Analog {
		public static final int GYRO = 0;
	}

	/**
	 * The {@code Digital} subclass contains all the ports for anything giving an digital output.
	 */
	public static class Digital {
	}
	/**
	 * The {@code Relay} subclass contains all the ports for anything in the relay section.
	 */
	public static class Relay {
	}
	/**
	 * The {@code CAN} subclass contains all the ports for anything using the CAN interface.
	 */
	public static class CAN {
		public static final int LEFT_FRONT_TALON_ID = 4;
		public static final int RIGHT_FRONT_TALON_ID = 2;
		public static final int LEFT_BACK_TALON_ID = 3;
		public static final int RIGHT_BACK_TALON_ID = 1;
		public static final int PDP_ID = 5;
		public static final int PCM_ID = 6;
	}

	/**
	 * The {@code USB} subclass contains all the ports for anything using the USB interface.
	 * This only contains the joysticks.
	 */
	public static class DS_USB {
		public static final int LEFT_STICK = 1;
		public static final int RIGHT_STICK = 0;
		public static final int GAMEPAD = 2;
	}

	/**
	 * The {@code PCM} subclass contains all the ports for anything connected to the pneumatics control module (PCM).
	 */
	public static class PCM {
		// Add Pneumatics ports as we decide them
	}

	public static class GamepadButtons {
		public static final int
			A = 1,
			B = 2,
			X = 3,
			Y = 4,
			LB = 5,
			RB = 6,
			BACK = 7,
			START = 8,
			L3 = 9,
			R3 = 10;
	}
	
	/**
	 * The {@code JoystickButtons} class contains all the button bindings for the
	 * Joysticks.
	 */
	public static class JoystickButtons {
		// Well this defeats the purpose of constants, doesn't it?
		public static final int
			BTN1 = 1,
			BTN2 = 2,
			BTN3 = 3,
			BTN4 = 4,
			BTN5 = 5,
			BTN6 = 6,
			BTN7 = 7,
			BTN8 = 8,
			BTN9 = 9,
			BTN10 = 10,
			BTN11 = 11;
	}
}
