package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DegreeRotate extends PIDCommand {

	private double _heading;
	
    public DegreeRotate(double heading) {
    	super(0.4, 0, 0.2);
    	requires(Robot.driveTrain);
    	_heading = heading;
    	getPIDController().setContinuous(true);
    	getPIDController().setAbsoluteTolerance(0.15);
    	getPIDController().setInputRange(-180, 180);
    	getPIDController().setOutputRange(-.4, .4);   //was at -.5,.5
    	LiveWindow.addActuator("Robot.driveTrain", "DegreeRotate", getPIDController());
    }
    

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getGyro().reset();
    	Robot.driveTrain.getNAVX().reset();
    	getPIDController().setSetpoint(_heading);
    	MercLogger.logMessage(Level.INFO, "Rotating to " + _heading + " degrees.");
		//Debug.logMessage(Level.INFO, "The Degree Rotate Command has been initialized.");

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return getPIDController().onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
		MercLogger.logMessage(Level.INFO, "The Degree Rotate Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		//return Robot.driveTrain.getGyro().getAngle();
		return Robot.driveTrain.getNAVX().getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		Robot.driveTrain.pidWrite(output);
	}
}
