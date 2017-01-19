package com.mercury1089.vision;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * 
 * @author Jared Tulayan
 */
public class VisionProc {
	private final NetworkTable NT; 
	private double targetWidth, targetHeight;
	private final int NUM_TARGETS = 2;
	
	private final double[] DEFAULT_VALUE = {-1};
	private final double 
		IMG_WIDTH, 
		IMG_HEIGHT,
		HFOV = 67.0, // The horizontal FOV of the camera. TODO: Calibrate values
		TARGET_WIDTH_INCHES = 6.75,
		TARGET_HEIGHT_INCHES = 5,
		IN_TO_FT = 12.0, // Study your freedom units guys
		TARGET_ELEVATION_FEET = 6.5;

	
	private final int MAX_RETRIES = 3;
	
	public VisionProc(String key) {
		NT = NetworkTable.getTable(key);
		
		// These values should never be null. If it is, the camera is broke.
		IMG_WIDTH = NT.getNumberArray("width", DEFAULT_VALUE)[0];
		IMG_HEIGHT = NT.getNumberArray("height", DEFAULT_VALUE)[0];
	}
	
	/**
	 * 
	 * @return
	 */
	// Updates values of target (if network table values are valid)
	// and updates the values.
	private boolean validateData() {
		double[] ntWidth, ntHeight;
		
		for (int i = 0; i < MAX_RETRIES; i++) {
			ntWidth = NT.getNumberArray("width", DEFAULT_VALUE);
			ntHeight = NT.getNumberArray("height", DEFAULT_VALUE);
			
			// Since we are looking for only two targets, and we defined a default value above,
			// we don't need to check if either number arrays are null, just if their lengths
			// equal NUM_TARGETS.
			if (ntWidth.length == NUM_TARGETS && ntHeight.length == NUM_TARGETS) {
				targetWidth = ntWidth[0] + ntWidth[1];
				targetHeight = ntHeight[0] + ntHeight[1];
				return true;
			}
		}
		
		return false;
	}
	
	public double getDist() {
		double dist = 0;
		
		// First we need to validate the incoming data
		// before we do any maths.
		if (validateData()) {
			
			// Using proportions, we can convert the target width in pixels 
			// to the target width in feet.
			dist = (TARGET_WIDTH_INCHES / IN_TO_FT) * (IMG_WIDTH / targetWidth);
			dist /= 2.0;
			dist /= Math.tan(Math.toRadians(HFOV / 2));
			
		}
		
		return dist;
	}
}
