package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * This {@link InstantCommand} calculates the path needed for the robot
 * to take in the {@link DeliveryGear} command group.
 */
public class CalculateHighGoalPath extends InstantCommand {
	
	private double distToMove, angleToTurn;
	
	/**
	 * <pre>
	 * public CalculateHighGoalPath(Direction direction)
	 * </pre>
	 * Creates an {@code InstantCommand} to generate the gear path. This is an InstantCommand that will run once and complete.
	 * The command will calculate the distance and angles for optimal gear delivery. After this command runs, the values
	 * can be accessed via getDistance(), getAngle(), getTheta() 
	 *
	 * @param reverseMovement whether the approach is forward or reverse (e.g. front/back of robot)
	 */
	public CalculateHighGoalPath() {
		distToMove = angleToTurn = 0.0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		getAlignMovements();
		//getAlignMovementsOnAPoint();
		MercLogger.logMessage(Level.INFO, "CalculateHighGoalPath: Initialized");
    }
	
    public double getDistance() {
    	MercLogger.logMessage(Level.INFO, "getDistance() : returning " + distToMove + " feet.");
    	return distToMove;
    }
    
    public double getAngle() {
    	return angleToTurn;
    }
    
    private synchronized void getAlignMovements() {			//Where getAlignMovements()[0] is the Move distance 
    												//and getAlignMovements()[1] is the turn angle

    	MercLogger.logMessage(Level.INFO, "_______________________________________________________________________________");
    	MercLogger.logMessage(Level.INFO, "Entering getAlignMovements() calculations (High Goal): ");
    	    	
    	double highGoalDistance = Robot.visionProcessor.getDistance(TargetType.HIGH_GOAL),				//Gets distance to center of highGoal
    		   highGoalAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.HIGH_GOAL);		//Angle from center of highGoal
    	/*SmartDashboard.putNumber("highGoalDistance", highGoalDistance);
    	SmartDashboard.putNumber("angleFromTargetTape", angleFromTargetTape);
    	SmartDashboard.putNumber("highGoalAngle", highGoalAngle);
    	SmartDashboard.putNumber("targetTapeDistance", targetTapeDistance);*/
    	MercLogger.logMessage(Level.INFO, "highGoalDistance: " + highGoalDistance);
    	MercLogger.logMessage(Level.INFO, "highGoalAngle: " + highGoalAngle);
    	MercLogger.logMessage(Level.INFO, "Exiting getAlignMovements() (High Goal).");
    	MercLogger.logMessage(Level.INFO, "_______________________________________________________________________________");
    	distToMove = highGoalDistance;
    	angleToTurn = highGoalAngle;
    }
}