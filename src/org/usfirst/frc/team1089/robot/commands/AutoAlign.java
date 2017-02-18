package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

/**
 *
 */
public class AutoAlign extends DegreeRotate {

	TargetType target;
	
    public AutoAlign(TargetType t) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		super();
		target = t;
		
		if (t.equals(TargetType.GEAR_VISION))
			_heading = Robot.visionProcessor.angleGear;
		else if (t.equals(TargetType.HIGH_GOAL))
			_heading = Robot.visionProcessor.angleHigh;
		else
			_heading = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	super.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	super.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.visionProcessor.getAngleFromCenter(target)) <= 1.5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	super.end();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	super.interrupted();
    }
}
