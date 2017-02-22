package org.usfirst.frc.team1089.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DeliverGear extends CommandGroup {
	

    public DeliverGear() {
    	
    	// Instantiate CalculateGearPath before adding it to the sequence so we have a reference to the
    	// distance and angle methods, so we can pass them to DriveDistance and DegreeRotate.
    	CalculateGearPath calculateGearPath = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);
    	addSequential(calculateGearPath);
    	
    	//addSequential(new DegreeRotate(calculateGearPath:getTheta)); 
    	addSequential(new DriveDistance(calculateGearPath::getDistance, 5.0));
    	addSequential(new DegreeRotate(calculateGearPath::getTheta));
    }
    
}
