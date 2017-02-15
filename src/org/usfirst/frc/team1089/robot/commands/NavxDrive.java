package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 *
 */
public class NavxDrive extends PIDCommand {

	private double position;
	private double _heading;
	
    public NavxDrive(double pos, double heading) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(.1,0,.6);
    	Robot.driveTrain.setToVbus();
    	Robot.driveTrain.disableRobotDrive();
    	requires(Robot.driveTrain);
    	getPIDController().setContinuous(true);
    	getPIDController().setAbsoluteTolerance(.1);
    	position = pos;
    	//_heading = heading;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getNAVX().reset();
    	//getPIDController().setSetpoint(_heading);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.enableRobotDrive();
    	Robot.driveTrain.setToPosition();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return Robot.driveTrain.getNAVX().getYaw();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		//Robot.driveTrain.pidWrite(_speed, output);
	}
}
