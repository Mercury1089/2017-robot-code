package org.usfirst.frc.team1089.robot.util;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

/**
 * This class encapsulates some methods and fields to properly
 * access data from the Pi vision processing.
 */
public class VisionProcessor {
	// Types of targets
	public static enum TargetType {
		GEAR_VISION, 
		HIGH_GOAL
	};

	// Network Table stuff
	private final String VISION_ROOT = "Vision/";
	private final NetworkTable GEAR_VISION_TABLE, HIGH_GOAL_TABLE;
	
	// Vision constants
	public final double 
		HFOV_PI = 53.50,   // The horizontal FOV of the pi camera (for gear vision).
		HFOV_LIFECAM = 60; // The horizontal FOV of the LifeCam (for high goal).
	
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
	
	public VisionProcessor() {
		GEAR_VISION_TABLE = NetworkTable.getTable(VISION_ROOT + "gearVision");
		HIGH_GOAL_TABLE = NetworkTable.getTable(VISION_ROOT + "highGoal");
	}
	
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
		double percievedWidth, hfov, targetWidth;
		
		switch (type) {
			case GEAR_VISION:
				percievedWidth = GEAR_VISION_TABLE.getNumber("targetWidth", 0);
				hfov = HFOV_PI;
				targetWidth = TARGET_WIDTH_INCHES_GEAR;
				break;
			case HIGH_GOAL:
				percievedWidth = HIGH_GOAL_TABLE.getNumber("targetWidth", 0);
				hfov = HFOV_LIFECAM;
				targetWidth = TARGET_WIDTH_INCHES_HIGH;
				break;
			default: 
				return Double.NEGATIVE_INFINITY;
		}
		
		return (targetWidth / IN_TO_FT) * IMG_WIDTH / ( 2 * percievedWidth * Math.tan( Math.toRadians( hfov / 2 ) ) );
	}
	
	/**
	 * <pre>
	 * public double getAngleFromCenter(TargetType type)
	 * </pre>
	 * Gets the angle needed to center the robot to the target
	 * 
	 * @param type the type of target that is being targeted
	 * @return the angle needed to turn to rotate towards the target
	 */
	public double getAngleFromCenter(TargetType type) {
		double centerX, dist, ratio, hfov;
		
		// Define our variables
		switch (type) {
			case GEAR_VISION:
				centerX = GEAR_VISION_TABLE.getNumberArray("center", DEF_VALUE)[0];
				hfov = HFOV_PI;
				break;
			case HIGH_GOAL:
				centerX = HIGH_GOAL_TABLE.getNumberArray("center", DEF_VALUE)[0];
				hfov = HFOV_LIFECAM;
				break;
			default: 
				return Double.NEGATIVE_INFINITY;
		}
		
		// Get the ratio of the distance from the center to the entire image width
		dist = centerX - IMG_WIDTH / 2.0;
		ratio = dist / (double)IMG_WIDTH;
		
		// Multiply the ratio by the HFOV
		return ratio * hfov;
	}

	public void initDefaultCommand() {
		//setDefaultCommand(new GetDistanceFromTarget());
	}
}
