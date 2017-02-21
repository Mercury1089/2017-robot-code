package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
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
		signedDirection = direction == Direction.FORWARD ? 1.0 : -1.0;
		distToMove = angleToTurn = theta = 0.0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		getAlignMovements();
		//getAlignMovementsOnAPoint();
    }
	
    public double getDistance() {
    	return distToMove * signedDirection;
    }
    
    public double getAngle() {
    	return angleToTurn * signedDirection;
    }

    public double getTheta() {
    	return theta * signedDirection;
    }
    
    private synchronized void getAlignMovements() {			//Where getAlignMovements()[0] is the Move distance 
    												//and getAlignMovements()[1] is the turn angle
    	
    	//Getting the closer tape
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] 
    					 <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1;//Coming in from right
    	int reversalFactor = targetTape == 0 ? -1 : 1;
    	SmartDashboard.putNumber("TargetTape", targetTape);
    	
    	double liftDistance = Robot.visionProcessor.getAverageDistanceToGearTargets(),				//Gets distance to center of lift
    		   angleFromTargetTape = Robot.visionProcessor.getAnglesFromGearTargets()[targetTape],	//Angle from closer tape
    		   liftAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION),		//Angle from center of lift
    		   targetTapeDistance = Robot.visionProcessor.getDistancesToGearTargets()[targetTape];	//Distance from target tape
    	SmartDashboard.putNumber("liftDistance", liftDistance);
    	SmartDashboard.putNumber("angleFromTargetTape", angleFromTargetTape);
    	SmartDashboard.putNumber("liftAngle", liftAngle);
    	SmartDashboard.putNumber("targetTapeDistance", targetTapeDistance);
    	
    	//
    	double distanceFromRetroHorizontal = 
    			(Math.pow(targetTapeDistance, 2) - Math.pow(liftDistance, 2)) / (centerToCenterDistanceByTwo * 2) - 
    			(centerToCenterDistanceByTwo * 2) / 4;
    	SmartDashboard.putNumber("distanceFromRetroHorizontal", Utilities.round(distanceFromRetroHorizontal, 3));
    	
    	//
    	double distanceFromLiftFace =
    			Math.sqrt(Math.pow(targetTapeDistance,  2) - Math.pow(distanceFromRetroHorizontal, 2));
    	SmartDashboard.putNumber("distanceFromLiftFace", Utilities.round(distanceFromLiftFace, 3));
    	
    	//
    	double phi = 
    			Math.toDegrees(Math.atan(distanceFromRetroHorizontal / distanceFromLiftFace));
    	SmartDashboard.putNumber("phi", Utilities.round(phi, 3));
    	
    	double theta = 
    			Math.abs(phi) + Math.abs(angleFromTargetTape);
    	SmartDashboard.putNumber("theta", theta);
    	
    	//Getting the distance to move
    	double distToMove = 
    			(distanceFromRetroHorizontal + centerToCenterDistanceByTwo) / Math.sin(Math.toRadians(theta));
    	SmartDashboard.putNumber("distToMove", Utilities.round(distToMove, 3));
    	
    	//Return. Congratulations! You have made it.
    	MercLogger.logMessage(Level.INFO, "Moving " + distToMove + " inches for gear delivery.");

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
