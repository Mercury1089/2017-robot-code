package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.Config;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.Utilities;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DeliverGear extends CommandGroup {
	

    public DeliverGear() {
    	
    	// Instantiate CalculateGearPath before adding it to the sequence so we have a reference to the
    	// distance and angle methods, so we can pass them to DriveDistance and DegreeRotate.
    	CalculateGearPath calculateGearPath = new CalculateGearPath(CalculateGearPath.Direction.REVERSE)
    	addSequential(calculateGearPath);
    	
    	//addSequential(new DegreeRotate(calculateGearPath:getTheta)); 
    	addSequential(new DriveDistance(calculateGearPath:getDistance));
    	addSequential(new DegreeRotate(calculateGearPath:getAngle));
    }
    
}
