package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 *
 */
public class DegreeRotate extends PIDCommand {

	private double _heading;
	private DriverStation ds;
	
    public DegreeRotate(double heading) {
    	super(0.5, 0.0, 0.0);
    	requires(Robot.driveTrain);
    	_heading = heading;
    	getPIDController().setContinuous(true);
    	getPIDController().setAbsoluteTolerance(0.1);
    	getPIDController().setInputRange(-180, 180);
    	getPIDController().setOutputRange(-1, 1);
    	ds = DriverStation.getInstance();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getGyro().reset();
    	DriverStation.reportError("Init", true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	DriverStation.reportError("Done", true);
    	return getPIDController().onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		DriverStation.reportError("INPUT", true);
		return Robot.driveTrain.getGyro().getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
    	DriverStation.reportError("Wrote Output", true);
		Robot.driveTrain.pidWrite(output);
	}
}
