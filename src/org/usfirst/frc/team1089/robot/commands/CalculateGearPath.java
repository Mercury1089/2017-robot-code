package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.Preferences;
import org.usfirst.frc.team1089.robot.util.Utilities;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link InstantCommand} calculates the path needed for the robot
 * to take in the {@link DeliveryGear} command group.
 */
public class CalculateGearPath extends InstantCommand {

	private double signedDirection;	// Used to multiply directions
	private double distToMove, angleToTurn, theta;
	private static double centerToCenterDistanceByTwo = 5.125 / 12 - 1;
	public static final double INCH_OFFSET_FROM_TARGET = 36; //TODO Maybe move this to a better place. Also, edit this value

	public enum Direction { FORWARD, REVERSE }
	
	/**
	 * <pre>
	 * public CalculateGearPath(Direction direction)
	 * </pre>
	 * Creates an {@code InstantCommand} to generate the gear path. This is an InstantCommand that will run once and complete.
	 * The command will calculate the distance and angles for optimal gear delivery. After this command runs, the values
	 * can be accessed via getDistance(), getAngle(), getTheta() 
	 *
	 * @param reverseMovement whether the approach is forward or reverse (e.g. front/back of robot)
	 */
	public CalculateGearPath(Direction direction) {
		signedDirection = direction == Direction.FORWARD ? -1.0 : 1.0;
		distToMove = angleToTurn = theta = 0.0;
		
		MercLogger.logMessage(Level.INFO, "CalculateGearPath: Constructed using CalculateGearPath(Direction direction)");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		getAlignMovements();
		//getAlignMovementsOnAPoint();
		MercLogger.logMessage(Level.INFO, "CalculateGearPath: Initialized");
    }
	
    public double getDistance() {
    	MercLogger.logMessage(Level.INFO, "Calculated Distance : " + distToMove * signedDirection + " feet.");
    	double fullDist = distToMove + Preferences.ROBOT_LENGTH_COMPETITION / 2.0;
    	MercLogger.logMessage(Level.INFO, "getDistance() : returning " + fullDist * signedDirection + " feet.");
    	return fullDist * signedDirection;
    }
    
    public double getAngle() {
    	return angleToTurn;
    }

    public double getTheta() {
    	MercLogger.logMessage(Level.INFO, "getTheta() : returning " + theta + " degrees.");
    	return theta;
    }
    
    private synchronized void getAlignMovements() {			//Where getAlignMovements()[0] is the Move distance 
    												//and getAlignMovements()[1] is the turn angle

    	MercLogger.logMessage(Level.INFO, "_______________________________________________________________________________");
    	MercLogger.logMessage(Level.INFO, "Entering getAlignMovements() calculations: ");
    	
    	//Getting the closer tape
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] 
    					 <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1; //Coming in from right and left, respectively
    	int reversalFactor = targetTape == 0 ? -1 : 1;
    	
    	
    	double liftDistance = Robot.visionProcessor.getAverageDistanceToGearTargets(),				//Gets distance to center of lift
    		   angleFromTargetTape = Robot.visionProcessor.getAnglesFromGearTargets()[targetTape],	//Angle from closer tape
    		   liftAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION),		//Angle from center of lift
    		   targetTapeDistance = Robot.visionProcessor.getDistancesToGearTargets()[targetTape];	//Distance from target tape
    	/*SmartDashboard.putNumber("liftDistance", liftDistance);
    	SmartDashboard.putNumber("angleFromTargetTape", angleFromTargetTape);
    	SmartDashboard.putNumber("liftAngle", liftAngle);
    	SmartDashboard.putNumber("targetTapeDistance", targetTapeDistance);*/
    	MercLogger.logMessage(Level.INFO, "liftDistance: " + liftDistance);
    	MercLogger.logMessage(Level.INFO, "angleFromTargetTape: " + angleFromTargetTape);
    	MercLogger.logMessage(Level.INFO, "targetTapeDistance: " + targetTapeDistance);
    	
    	
    	//
    	double distanceFromRetroHorizontal = 
    			(Math.pow(targetTapeDistance, 2) - Math.pow(liftDistance, 2)) / (centerToCenterDistanceByTwo * 2) - 
    			(centerToCenterDistanceByTwo * 2) / 4;
//    	SmartDashboard.putNumber("distanceFromRetroHorizontal", Utilities.round(distanceFromRetroHorizontal, 3));
    	MercLogger.logMessage(Level.INFO, "distanceFromRetroHorizontal is: " + distanceFromRetroHorizontal);
    	
    	//
    	double distanceFromLiftFace =
    			Math.sqrt(Math.pow(targetTapeDistance,  2) - Math.pow(distanceFromRetroHorizontal, 2));
//    	SmartDashboard.putNumber("distanceFromLiftFace", Utilities.round(distanceFromLiftFace, 3));
    	MercLogger.logMessage(Level.INFO, "distanceFromLiftFace is: " + distanceFromLiftFace);
    	
    	//
    	double phi = 
    			Math.toDegrees(Math.atan(distanceFromRetroHorizontal / distanceFromLiftFace));
//    	SmartDashboard.putNumber("phi", Utilities.round(phi, 3));
    	MercLogger.logMessage(Level.INFO, "phi is: " + phi);
    	
    	theta = 
    			(Math.abs(phi) + Math.abs(angleFromTargetTape));
//    	SmartDashboard.putNumber("theta", theta);
    	
    	//Getting the distance to move
    	distToMove = 
    			(distanceFromRetroHorizontal + centerToCenterDistanceByTwo) / Math.sin(Math.toRadians(theta));	// * 12 for feet
//    	SmartDashboard.putNumber("distToMove", Utilities.round(distToMove, 3));
    	MercLogger.logMessage(Level.INFO, "distToMove is: " + distToMove);
    	
    	theta *= reversalFactor;
    	if(theta > 0) 
    		theta = 90 - theta;
    	MercLogger.logMessage(Level.INFO, "theta is: " + theta);
    	
    	//Return. Congratulations! You have made it.
    	MercLogger.logMessage(Level.INFO, "Calculating that we should move " + distToMove + " feet and rotate " + theta + "degrees.");
    	MercLogger.logMessage(Level.INFO, "Exiting getAlignMovements().");
    	MercLogger.logMessage(Level.INFO, "_______________________________________________________________________________");

    }
    
    private synchronized void getAlignMovementsOnAPoint() {
    	
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1;//Coming in from right
    	int reversalFactor = targetTape == 0 ? -1 : 1;
    	double liftDistance = Robot.visionProcessor.getAverageDistanceToGearTargets(),				//Gets distance to center of lift
     		   angleFromTargetTape = Robot.visionProcessor.getAnglesFromGearTargets()[targetTape],	//Angle from closer tape
     		   liftAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION),		//Angle from center of lift
     		   targetTapeDistance = Robot.visionProcessor.getDistancesToGearTargets()[targetTape];	//Distance from target tape
    	
    	//
    	double distanceFromRetroHorizontal = 
    			(Math.pow(targetTapeDistance, 2) - Math.pow(liftDistance, 2)) / (centerToCenterDistanceByTwo * 2) - 
    			(centerToCenterDistanceByTwo * 2) / 4;
    	SmartDashboard.putNumber("distanceFromRetroHorizontal", Utilities.round(distanceFromRetroHorizontal, 3));
    	
    	//
    	double distanceFromLiftFace =
    			Math.sqrt(Math.pow(targetTapeDistance,  2) - Math.pow(distanceFromRetroHorizontal, 2));
    	SmartDashboard.putNumber("distanceFromLiftFace", Utilities.round(distanceFromLiftFace, 3));
    	
    	theta = Math.atan(Math.abs(distanceFromRetroHorizontal) / Math.abs(distanceFromLiftFace)); //In radians
    	
    	distToMove = Math.sqrt(Math.pow(liftDistance, 2) + Math.pow(INCH_OFFSET_FROM_TARGET, 2) 
    	- 2 * liftDistance * INCH_OFFSET_FROM_TARGET * Math.toDegrees(Math.cos(theta))); //Law of cosines
    
    	angleToTurn = Math.toDegrees(Math.atan((INCH_OFFSET_FROM_TARGET * Math.sin(theta) / distToMove)));
    
    }
}
