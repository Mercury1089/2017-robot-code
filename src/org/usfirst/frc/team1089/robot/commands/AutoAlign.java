package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *	{@link AutoAlign} rotates the robot to minimize
 *  the angle between robot and specified target type
 */
public class AutoAlign extends DegreeRotate {

	TargetType target;
	
    public AutoAlign(TargetType t) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		super();
		target = t;
		
		MercLogger.logMessage(Level.INFO, "AutoAlign: Constructed using AutoAlign(TargetType t)");

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (target.equals(TargetType.GEAR_VISION))
			_heading = Robot.visionProcessor.angleGear - 6;
		else if (target.equals(TargetType.HIGH_GOAL))
			_heading = Robot.visionProcessor.angleHigh;
		else
			_heading = 0;
    	
    	super.initialize();
    	
    	MercLogger.logMessage(Level.INFO, "AutoAlign: Initialized with heading: " + _heading);
    	MercLogger.logMessage(Level.INFO, "AutoAlign: Initialized with targetType: " + target);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	/*if(Robot.visionProcessor.isRecent(target))
    		super.updateHeading(Robot.visionProcessor.getAngleFromCenter(target));*/
    	super.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//    	return super.isFinished();
    	if (super.isFinished() && Robot.visionProcessor.isRecent(target)) {
        	
    		if (Robot.visionProcessor.isOnTarget(target)) {
        		return true;
        	} else {
        		return true;
        	}
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	super.end();
    	
    	if (target.equals(TargetType.GEAR_VISION))
			MercLogger.logMessage(Level.INFO, "AutoAlign: Ending with angle gear: " + Robot.visionProcessor.angleGear);
		else if (target.equals(TargetType.HIGH_GOAL))
			MercLogger.logMessage(Level.INFO, "AutoAlign: Ending with angle high: " + Robot.visionProcessor.angleHigh);
    	
    	MercLogger.logMessage(Level.INFO, "AutoAlign: Completed");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	super.interrupted();
    	MercLogger.logMessage(Level.INFO, "AutoAlign: Interrupted");
    }
}
