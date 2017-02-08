package org.usfirst.frc.team1089.robot.util;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class VisionProcessor {
	// Types of targets
	public static enum TargetType {
		GEAR_VISION, 
		HIGH_GOAL
	};

	// Network Table stuff
	private final String VISION_ROOT = "Vision/";
	
	// Vision constants
	public final double 
		HFOV_PI = 53.50,   // The horizontal FOV of the pi camera.
		HFOV_LIFECAM = 60; // The horizontal FOV of the pi camera.
	
	public final int 
		IMG_WIDTH = 320,   // Resolution-x of the camera feed (from both cameras)
		IMG_HEIGHT = 240;  // Resolution-y of the camera feed (from both cameras) 
	
	// Targeting constants
	private final double 
		TARGET_WIDTH_INCHES_GEAR = 10.75,
		TARGET_HEIGHT_INCHES_GEAR = 5,
		TARGET_ELEVATION_FEET_GEAR = 10.75,
		TARGET_WIDTH_INCHES_HIGH = 15,
		TARGET_HEIGHT_INCHES_HIGH = 10,
		TARGET_ELEVATION_FEET_HIGH = 6.5,
		IN_TO_FT = 12.0; // Study your freedom units guys
	
	// Default value for number arrays 
	// so we can avoid recreating this space-consuming line
	private double[] DEF_VALUE = {-1, -1};
	
	public VisionProcessor() { }
	
	/**
	 * <pre>
	 * public double getDistance(TargetType type)
	 * </pre>
	 * Gets the distance between the center of the camera and the center of the target.
	 * Calculates and returns different distances for the different vision targets
	 * 
	 * @param type the type of target that is being targeted
	 * @return the distance between the center of the camera and the center of the target, in feet. 
	 *         Accurate to 0.15 ft.
	 */
	public double getDistance(TargetType type) {
		double targetWidth, hfov;
		
		switch (type) {
			case GEAR_VISION:
				targetWidth = NetworkTable.getTable(VISION_ROOT + "gearVision").getNumber("targetWidth", 0);
				break;
			case HIGH_GOAL:
				targetWidth = NetworkTable.getTable(VISION_ROOT + "highGoal").getNumber("targetWidth", 0);
				break;
			default: 
				return Double.NEGATIVE_INFINITY;
		}
		
		return (TARGET_WIDTH_INCHES_GEAR / IN_TO_FT) * IMG_WIDTH / ( 2 * targetWidth * Math.tan( Math.toRadians( HFOV / 2 ) ) );
	}
	
	/**
	 * <pre>
	 * public double getIsCentered(TargetType type)
	 * </pre>
	 * Gets the distance that the center of the robot is from the center of the target.
	 * 
	 * @param type the type of target that is being targeted
	 * @return the distance in pixels that the center of the camera is from the center of the visible target,
	 *         or 0 if the distance from the center is at most 5 pixels
	 */
	public double getDistanceFromCenter(TargetType type) {
		double centerX;
		
		switch (type) {
			case GEAR_VISION:
				centerX = NetworkTable.getTable(VISION_ROOT + "gearVision").getNumberArray("center", DEF_VALUE)[0];
				break;
			case HIGH_GOAL:
				centerX = NetworkTable.getTable(VISION_ROOT + "highGoal").getNumberArray("center", DEF_VALUE)[0];
				break;
			default: 
				return Double.NEGATIVE_INFINITY;
		}
		
		return Math.abs(centerX - IMG_WIDTH / 2) <= 5 ? centerX - IMG_WIDTH / 2 : 0;
	}

	public void initDefaultCommand() {
		//setDefaultCommand(new GetDistanceFromTarget());
	}
}
