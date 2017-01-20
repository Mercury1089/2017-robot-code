package org.usfirst.frc.team1089.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * This {@link Subsystem} handles targeting using the combo of a Camera
 * and an OpenCV pipeline.
 */
// TODO: Sort out this class; not everything belongs here.
public class Vision extends Subsystem {
	private double targetWidth, targetHeight;
	private final NetworkTable GRIP_TABLE;
	private final int NUM_TARGETS = 2;
	private final double[] DEFAULT_VALUE = {-1};
	private final double 
		IMG_WIDTH, 
		IMG_HEIGHT,
		HFOV = 58.0, // The horizontal FOV of the camera. TODO: Calibrate values
		TARGET_WIDTH_INCHES = 10.75,
		TARGET_HEIGHT_INCHES = 5,
		IN_TO_FT = 12.0, // Study your freedom units guys
		TARGET_ELEVATION_FEET = 10.75;

	
	private final int MAX_RETRIES = 3;
	
	// Initializes the GRIP NetworkTable
	// and initializes the image res.
	public Vision() {
		GRIP_TABLE = NetworkTable.getTable("GRIP");
		
		// NOTES:
		// 	> These values should never be null. If it is, the camera is broke.
		//	> The values are off by exactly 2 pixels.
		IMG_WIDTH = GRIP_TABLE.getSubTable("screenSize").getNumberArray("width", DEFAULT_VALUE)[0] + 2;
		IMG_HEIGHT = GRIP_TABLE.getSubTable("screenSize").getNumberArray("height", DEFAULT_VALUE)[0] + 2;
	}
	
	/**
	 * <pre>
	 * private boolean updateTarget()
	 * </pre>
	 * Checks if the data coming from the {@link NetworkTable}s are valid,
	 * and if they are, updates the data needed to find the target.
	 * 
	 * @return whether or not the data coming in is valid.
	 */
	
	private boolean updateTarget() {
		double[] ntWidth, ntHeight;
		
		for (int i = 0; i < MAX_RETRIES; i++) {
			ntWidth = NetworkTable.getTable("GRIP/target").getNumberArray("width", DEFAULT_VALUE);
			ntHeight = NetworkTable.getTable("GRIP/target").getNumberArray("height", DEFAULT_VALUE);
			
			// Since we are looking for only two targets, and we defined a default value above,
			// we don't need to check if either number arrays are null, just if their lengths
			// equal NUM_TARGETS.
			if (ntWidth.length == NUM_TARGETS && ntHeight.length == NUM_TARGETS) { // Always a need for second chances
				targetWidth = ntWidth[0] + ntWidth[1];
				targetHeight = ntHeight[0] + ntHeight[1];
				System.out.println("it worked you want a cookie?");
				return true;
			} else {
				System.out.println("ntWidth[0] = " + ntWidth[0] + ", ntHeight[0] = " + ntHeight[0]);
			}
		}
		
		System.out.println("did not work");
		return false;
	}
	
	/**
	 * <pre>
	 * public double getDist()
	 * </pre>
	 * Gets the distance between the camera and the target.
	 * 
	 */
	// TODO: Figure out what units are returned (ft, pixels, etc.)
	public double getDist() {
		double dist = 0;
		
		// First we need to validate the incoming data
		// before we do any maths.
		if (updateTarget()) {
			
			// Using proportions, we can convert the target width in pixels 
			// to the target width in feet.
			// TODO: Fix this; it does not work.
			dist = (TARGET_WIDTH_INCHES / IN_TO_FT) * (IMG_WIDTH / targetWidth);
			dist /= 2.0;
			dist /= Math.tan(Math.toRadians(HFOV / 2));
		}
		
		return dist;
	}
	
	public void initDefaultCommand() {
		
	}
}
