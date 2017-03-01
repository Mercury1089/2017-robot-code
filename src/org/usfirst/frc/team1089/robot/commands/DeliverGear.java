package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * This {@link Command} serves as a group that gets the robot to move towards the gear delivery and
 * drop the gear
 */
public class DeliverGear extends CommandGroup {
	
	/**
	 * <pre>
	 * public DeliverGear
	 * </pre>
	 * 
	 * Creates this {@code DeliverGear} command to do the following:
	 * <ol>
	 * <li>Calculate the path it needs to take to get to the gear rod</li>
	 * <li>Drive the distance in that path</li>
	 * <li>Rotate towards the gear delivery</li>
	 * </ol>
	 */
    public DeliverGear() {
    	
    	MercLogger.logMessage(Level.INFO, "DeliverGear CommandGroup: Started");

    	// Instantiate CalculateGearPath before adding it to the sequence so we have a reference to the
    	// distance and angle methods, so we can pass them to DriveDistance and DegreeRotate.
    	CalculateGearPath calculateGearPath = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);
    	addSequential(calculateGearPath);
    	
    	addSequential(new DriveDistance(calculateGearPath::getDistance, 5.0));
    	addSequential(new DegreeRotate(calculateGearPath::getTheta));
    	//addSequential(new DegreeRotate(Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION)));
    	
    	MercLogger.logMessage(Level.INFO, "DeliverGear CommandGroup: Completed");

    }
    
}
