package org.usfirst.frc.team1089.robot.commands;

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
    	// Instantiate CalculateGearPath before adding it to the sequence so we have a reference to the
    	// distance and angle methods, so we can pass them to DriveDistance and DegreeRotate.
    	CalculateGearPath calculateGearPath = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);
    	addSequential(calculateGearPath);
    	
    	//addSequential(new DegreeRotate(calculateGearPath:getTheta)); 
    	addSequential(new DriveDistance(calculateGearPath::getDistance));
    	addSequential(new DegreeRotate(calculateGearPath::getAngle));
    }
    
}
