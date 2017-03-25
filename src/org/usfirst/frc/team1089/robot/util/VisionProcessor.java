package org.usfirst.frc.team1089.robot.util;

import org.opencv.core.Point;
import org.opencv.core.Size;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

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
	private final long LATENCY_MS = 100;
	
	// Timestamps for each of the vision threads
	private double
		timeGear = 0,
		timeHigh = 0;
	
	// Useful public fields for easy access from other objects
	public double
		angleGear = 0,
		angleHigh = 0,
		distGear = 0,
		distHigh = 0;
	
	// Classes to encapsulate the dimensions and coordinates of the centers of each target
	private final Size
		BOUNDS_TARGET1_GEAR,
		BOUNDS_TARGET2_GEAR,
		BOUNDS_TOTAL_GEAR,
		BOUNDS_TARGET1_HIGH,
		BOUNDS_TARGET2_HIGH,
		BOUNDS_TOTAL_HIGH;
	
	private final Point
		CENTER_TARGET1_GEAR,
		CENTER_TARGET2_GEAR,
		CENTER_TOTAL_GEAR,
		CENTER_TARGET1_HIGH,
		CENTER_TARGET2_HIGH,
		CENTER_TOTAL_HIGH;
	
	private final double OUT_OF_RANGE_CONST = -1;
	private final double ON_TARGET_THRESHOLD = 1.5; 
	
	// Vision constants
	public static class PICam {
		public static final double 
            HFOV_PI = 53.50,      //Horizontal field of view for the PI Cam
            //VFOV_PI = SmartDashboard.getNumber("PI_VFOV", 40);
            //VFOV_PI = 41.41;      //Vertical field of view for the PI Cam
            VFOV_PI = 37.5;
		public static final int
			HRES_PI = 320,        //Resolution-x of the PI feed
			VRES_PI = 240;		  //Resolution-y of the PI feed
	}
	
	public static class LifeCam {
		public static final double 
			HFOV_LIFECAM = 60.00, //Horizontal Field of View for the Life Cam
            VFOV_LIFECAM = 33.05; //Vertical field of view for the Life Cam
		public static final int
	        HRES_LIFECAM = 320,   //Resolution-x of the Life Cam feed
			VRES_LIFECAM = 240;   //Resolution-y of the Life Cam feed
	} 
	
	// Targeting constants
	private final double
		TARGET_WIDTH_INCHES_GEAR = 10.75, // the width of the virtual target which is the aggregate of both targets 1 and 2
		TARGET_WIDTH_INCHES_GEAR_SINGLE = 2.0, // the width of the virtual target which is the aggregate of both targets 1 and 2
		TARGET_HEIGHT_INCHES_GEAR = 5,
		TARGET_ELEVATION_FEET_GEAR = 10.75 / 12,
		TARGET_WIDTH_INCHES_HIGH = 15,
		TARGET_HEIGHT_INCHES_HIGH = 10, // the height of the virtual target which is the aggregate of both targets 1 and 2
		TARGET_ELEVATION_FEET_HIGH = 6.5,
		IN_TO_FT = 12.0; // Study your freedom units guys
	
	// Default value for number arrays 
	// so we can avoid recreating this space-consuming line
	private double[] DEF_VALUE = {-1, -1};
	
	private double offset;
	
	public VisionProcessor() {
		// Get the tables
		GEAR_VISION_TABLE = NetworkTable.getTable(VISION_ROOT + "gearVision");
		HIGH_GOAL_TABLE = NetworkTable.getTable(VISION_ROOT + "highGoal");
		
		// Initialize our target objects
		CENTER_TARGET1_GEAR = new Point(-1, -1);
		CENTER_TARGET2_GEAR = new Point(-1, -1);
		CENTER_TOTAL_GEAR = new Point(-1, -1);
		CENTER_TARGET1_HIGH = new Point(-1, -1);
		CENTER_TARGET2_HIGH = new Point(-1, -1);
		CENTER_TOTAL_HIGH = new Point(-1, -1);
		
		BOUNDS_TARGET1_GEAR = new Size(-1, -1);
		BOUNDS_TARGET2_GEAR = new Size(-1, -1);
		BOUNDS_TOTAL_GEAR = new Size(-1, -1);
		BOUNDS_TARGET1_HIGH = new Size(-1, -1);
		BOUNDS_TARGET2_HIGH = new Size(-1, -1);
		BOUNDS_TOTAL_HIGH = new Size(-1, -1);
		
		// Implement them table listeners
		GEAR_VISION_TABLE.addTableListener((ITable table, String key, Object value, boolean isNew) -> {
			synchronized(this) {
				switch (key) {
					case "centerTarget1":
						CENTER_TARGET1_GEAR.set((double[])value);
						break;
					case "centerTarget2":
						CENTER_TARGET2_GEAR.set((double[])value);
						break;
					case "centerTotal":
						CENTER_TOTAL_GEAR.set((double[])value);
						break;
					case "boundsTarget1":
						BOUNDS_TARGET1_GEAR.set((double[])value);
						break;
					case "boundsTarget2":
						BOUNDS_TARGET2_GEAR.set((double[])value);
						break;
					case "boundsTotal":
						BOUNDS_TOTAL_GEAR.set((double[])value);
						break;
					case "deltaTime":
						long systm = System.currentTimeMillis();
						timeGear = (double)systm - (double)value; 
				}
				
				distGear = getDistanceUsingVerticalInformation(TargetType.GEAR_VISION) <= 3.0 ? 
						getDistanceUsingHorizontalInformation(TargetType.GEAR_VISION) : getDistanceUsingVerticalInformation(TargetType.GEAR_VISION);
				angleGear = getAngleFromCenter(TargetType.GEAR_VISION);
			}
		});
		
		HIGH_GOAL_TABLE.addTableListener((ITable table, String key, Object value, boolean isNew) -> {
			synchronized(this) {
				switch (key) {
					case "centerTarget1":
						CENTER_TARGET1_HIGH.set((double[])value);
						break;
					case "centerTarget2":
						CENTER_TARGET2_HIGH.set((double[])value);
						break;
					case "centerTotal":
						CENTER_TOTAL_HIGH.set((double[])value);
						break;
					case "boundsTarget1":
						BOUNDS_TARGET1_HIGH.set((double[])value);
						break;
					case "boundsTarget2":
						BOUNDS_TARGET2_HIGH.set((double[])value);
						break;
					case "boundsTotal":
						BOUNDS_TOTAL_HIGH.set((double[])value);
						break;
					case "deltaTime":
						long systm = System.currentTimeMillis();
						timeHigh = (double)systm - (double)value; 
				}
				
				distHigh = getDistanceUsingVerticalInformation(TargetType.HIGH_GOAL);
				angleHigh = getAngleFromCenter(TargetType.HIGH_GOAL);
			}
		});
	}
	
	/**
	 * <pre>
	 * public double getDistance(TargetType type)
	 * </pre>
	 * Gets the distance between the center of the camera and the center of the target.
	 * Calculates and returns different distances for the different vision targetshighGoalTable
	 * 
	 * @param type the type of target that is being targeted
	 * @return the distance between the center of the camera and the center of the target, in feet, 
	 *         or negative infinity if the target cannot be seen
	 */
	public double getDistanceUsingHorizontalInformation(TargetType type) {
		double percievedWidth, hfov, targetWidth, hres;
		//System.out.println("Getting distance to " + type.toString() + " using horizontal information");
		
		switch (type) {
			case GEAR_VISION:
				percievedWidth = GEAR_VISION_TABLE.getNumber("targetWidth", 0);
				hfov = PICam.HFOV_PI;
				hres = PICam.HRES_PI;
				targetWidth = TARGET_WIDTH_INCHES_GEAR;
				break;
			case HIGH_GOAL:
				percievedWidth = HIGH_GOAL_TABLE.getNumber("targetWidth", 0);
				hfov = LifeCam.HFOV_LIFECAM;
				hres = LifeCam.HRES_LIFECAM;
				targetWidth = TARGET_WIDTH_INCHES_HIGH;
				break;
			default: 
				return /*Double.NEGATIVE_INFINITY*/OUT_OF_RANGE_CONST;
		}
		
		// Don't return anything if it can't be seen
		if (percievedWidth == -1)
			return /*Double.NEGATIVE_INFINITY*/OUT_OF_RANGE_CONST;
		
		// Magic equation derived from a few things we know about the target
		return (targetWidth / IN_TO_FT) * hres / ( 2 * percievedWidth * Math.tan( Math.toRadians( hfov / 2 ) ) );
	}
	
	public double getDistance(TargetType type) {
		return getDistanceUsingHorizontalInformation(type);
	}
	
	public double getDistanceUsingVerticalInformation(TargetType type) {
		double perceivedHeight, vfov, targetHeight, vres;
		//PICam.VFOV_PI = SmartDashboard.getNumber("PI_VFOV", 40);
		//System.out.println("Getting distance to " + type.toString() + " using vertical information");
		
		switch (type) {
			case GEAR_VISION:
				perceivedHeight = GEAR_VISION_TABLE.getNumber("targetHeight", 0);
				vfov = PICam.VFOV_PI;
				vres = PICam.VRES_PI;
				targetHeight = TARGET_HEIGHT_INCHES_GEAR;
				offset = 0;
				break;
			case HIGH_GOAL:
				perceivedHeight = HIGH_GOAL_TABLE.getNumber("targetHeight", 0);
				vfov = LifeCam.VFOV_LIFECAM;
				vres = LifeCam.VRES_LIFECAM;
				targetHeight = TARGET_HEIGHT_INCHES_HIGH;
				offset = 20.5/12;
				break;
			default: 
				return Double.NEGATIVE_INFINITY;
		}
		
		// Don't return anything if it can't be seen
		if (perceivedHeight == -1)
			return Double.NEGATIVE_INFINITY;
		
		// Magic equation derived from a few things we know about the target
		return (targetHeight / IN_TO_FT) * vres / ( 2 * perceivedHeight * Math.tan( Math.toRadians( vfov / 2 ) ) ) + offset;
	}
	
	public double[] getDistancesToGearTargets() {
		double target1Height, target2Height, vfov, targetHeight, vres;
		double[] output = new double[2];
		//System.out.println("Getting distance to each target using vertical information");
		
		target1Height = GEAR_VISION_TABLE.getNumberArray("boundsTarget1", new double[]{-1, -1})[1];
		target2Height = GEAR_VISION_TABLE.getNumberArray("boundsTarget2", new double[]{-1, -1})[1];
		vfov = PICam.VFOV_PI;
		vres = PICam.VRES_PI;
		targetHeight = TARGET_HEIGHT_INCHES_GEAR;		
		
		// Don't return anything if either can't be seen
		if (target1Height == -1 || target2Height == -1) {
			output[0] = Double.NEGATIVE_INFINITY;
			output[1] = Double.NEGATIVE_INFINITY;
			return output;
		}
			
		
		// Magic equation derived from a few things we know about the target
		output[0] = (targetHeight / IN_TO_FT) * vres / ( 2 * target1Height * Math.tan( Math.toRadians( vfov / 2 ) ) );
		output[1] = (targetHeight / IN_TO_FT) * vres / ( 2 * target2Height * Math.tan( Math.toRadians( vfov / 2 ) ) );
		return output;
	}
	
	public double[] getDistancesToGearTargetsHorizontal() {
		double target1Width, target2Width, hfov, targetWidth, hres;
		double[] output = new double[2];
		//System.out.println("Getting distance to each target using vertical information");
		
		target1Width = GEAR_VISION_TABLE.getNumberArray("boundsTarget1", new double[]{-1, -1})[0];
		target2Width = GEAR_VISION_TABLE.getNumberArray("boundsTarget2", new double[]{-1, -1})[0];
		hfov = PICam.HFOV_PI;
		hres = PICam.HRES_PI;
		targetWidth = TARGET_WIDTH_INCHES_GEAR_SINGLE;
		
		// Don't return anything if either can't be seen
		if (target1Width == -1 || target2Width == -1) {
			output[0] = Double.NEGATIVE_INFINITY;
			output[1] = Double.NEGATIVE_INFINITY;
			return output;
		}
			
		
		// Magic equation derived from a few things we know about the target
		output[0] = (targetWidth / IN_TO_FT) * hres / ( 2 * target1Width * Math.tan( Math.toRadians( hfov / 2 ) ) );
		output[1] = (targetWidth / IN_TO_FT) * hres / ( 2 * target2Width * Math.tan( Math.toRadians( hfov / 2 ) ) );
		return output;
	}
	
	/**
	 * <pre>
	 * public double getAngleFromCenter(TargetType type)
	 * </pre>
	 * Gets the angle needed to center the robot to the target
	 * 
	 * @param type the type of target that is being targeted
	 * @return the angle needed to turn to rotate towards the target,
	 *         or negative infinity if the target cannot be seen
	 */
	public double getAngleFromCenter(TargetType type) {
		double centerX, dist, ratio, hfov, hres, angleOffset;
		double angleFromCenter;
	
		// Define our variables
		switch (type) {
			case GEAR_VISION:
				centerX = GEAR_VISION_TABLE.getNumberArray("center", DEF_VALUE)[0];
				hfov = PICam.HFOV_PI;
				hres = PICam.HRES_PI;
				angleOffset = 1;
				break;
			case HIGH_GOAL:
				centerX = HIGH_GOAL_TABLE.getNumberArray("center", DEF_VALUE)[0];
				hfov = LifeCam.HFOV_LIFECAM;
				hres = LifeCam.HRES_LIFECAM;
				angleOffset = 0;
				break;
			default: 
				return 0;
		}
		
		// Don't return anything if it can't be seen
		if (centerX == -1)
			return 0;
		
		// Get the ratio of the distance from the center to the entire image width
		dist = centerX - hres / 2.0;
		ratio = dist / (double)hres;
		
		angleFromCenter = ratio * hfov + angleOffset;
		// Multiply the ratio by the HFOV
		return Math.abs(angleFromCenter) > 1.0 ? angleFromCenter : 0.0;
	}
	
	/**
	 * <pre>
	 * public double[] getAnglesFromGearTargets()
	 * </pre>
	 * 
	 * Gets the angles that the robot needs to turn 
	 * to center itself to the left and right target
	 * 
	 * @return array of angles that the robot can turn to to face the left or right target;
	 *         left target is element 1, right target is element 2
	 */
	public double[] getAnglesFromGearTargets() {
		double centerX1, centerX2, dist1, dist2,  ratio1, ratio2, hfov, hres;
		double[] output = new double[2];
	
		// Define our variables
		centerX1 = GEAR_VISION_TABLE.getNumberArray("centerTarget1", DEF_VALUE)[0];
		centerX2 = GEAR_VISION_TABLE.getNumberArray("centerTarget2", DEF_VALUE)[0];
		hfov = PICam.HFOV_PI;
		hres = PICam.HRES_PI;
		
		// Don't return anything if it can't be seen
		if (centerX1 == -1 || centerX2 == -1) {
			output[0] = 0;
			output[1] = 0;
			return output;
		}
		
		// Get the ratio of the distance from the center to the entire image width
		dist1 = centerX1 - hres / 2.0;
		dist2 = centerX2 - hres / 2.0;
		ratio1 = dist1 / (double)hres;
		ratio2 = dist2 / (double)hres;
		
		// Multiply the ratio by the HFOV
		output[0] = Math.abs(ratio1 * hfov) > 1.0 ? ratio1 * hfov : 0.0;
		output[1] = Math.abs(ratio2 * hfov) > 1.0 ? ratio2 * hfov : 0.0;
		return output;
	}
	
	/**
	 * <pre>
	 * public boolean isRecent(TargetType type)
	 * </pre>
	 * Tests whether or not the image from the specified camera
	 * is current
	 * 
	 * @param type the {@code TargetType} for which camera to take the timestamp from
	 * @return whether or not the image timestamp is at least 10 milliseconds earlier than the system time
	 */
	public boolean isRecent(TargetType type) {
		long curTime = System.currentTimeMillis();
		long imgTime = 0;
		
		switch (type) {
			case GEAR_VISION:
				imgTime = (long)timeGear;
				break;
			case HIGH_GOAL:
				imgTime = (long)timeHigh;
				break;
			default: 
				return false;
		}
		
		return curTime >= imgTime + LATENCY_MS;
	}
	
	public double getAverageDistanceUsingHorAndVerDistances(TargetType type) {
		return (getDistanceUsingHorizontalInformation(type) + getDistanceUsingVerticalInformation(type)) / 2; 
	}
	
	public double getAverageDistanceToGearTargetsVertical() {
		double[] input = getDistancesToGearTargets();
		return (input[0] + input[1]) / 2; 
	}
	
	public double getAverageDistanceToGearTargetsHorizontal() {
		double[] input = getDistancesToGearTargetsHorizontal();
		return (input[0] + input[1]) / 2; 
	}

	public double getAdjustedAverageDistanceToGearTargetsHorizontal() {
		return -(getAverageDistanceToGearTargetsHorizontal() - 0.5); 
	}
	
	public boolean isOnTarget(TargetType target) {
		return Math.abs(getAngleFromCenter(target)) < ON_TARGET_THRESHOLD;
		
	}

	public void initDefaultCommand() {
		//setDefaultCommand(new GetDistanceFromTarget());
	}
}
