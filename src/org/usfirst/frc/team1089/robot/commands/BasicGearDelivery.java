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
public class BasicGearDelivery extends CommandGroup {
	
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
    public BasicGearDelivery() {
    	// Instantiate CalculateGearPath before adding it to the sequence so we have a reference to the
    	// distance and angle methods, so we can pass them to DriveDistance and DegreeRotate.
    	CalculateGearPath calculateGearPath1 = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);
    	CalculateGearPath calculateGearPath2 = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);

    	addSequential(calculateGearPath1);
    	//addSequential(new DegreeRotate(calculateGearPath1::getBasicTurnToLiftAngle));
    	addSequential(new AutoAlign(TargetType.GEAR_VISION));
    	addSequential(new AutoAlign(TargetType.GEAR_VISION));
    	//addSequential(new AutoAlign(TargetType.GEAR_VISION));
    	addSequential(new DriveDistance(calculateGearPath1::getBasicLiftDistancePart1, 3.0));
    	
    	//addSequential(calculateGearPath2);
    	//addSequential(new DegreeRotate(calculateGearPath2::getBasicTurnToLiftAngle));    	
    	//addSequential(new AutoAlign(TargetType.GEAR_VISION));
    	//addSequential(new AutoAlign(TargetType.GEAR_VISION));
    	//addSequential(new DriveDistance(calculateGearPath2::getBasicLiftDistancePart2, 3.0));
    	//addSequential(new DriveToWall(1.75)); // TODO make sure we drive to the wall somehow
    	addSequential(new ToggleGearDelivery(true));
    }
}
