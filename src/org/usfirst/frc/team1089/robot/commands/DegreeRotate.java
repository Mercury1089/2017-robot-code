package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DegreeRotate extends Command {

	private double _heading;
	
    public DegreeRotate(double heading) {
    	requires(Robot.driveTrain);
    	_heading = heading;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.enable();
    	Robot.driveTrain.setSetpoint(_heading);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.driveTrain.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.disable();
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
