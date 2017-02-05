package org.usfirst.frc.team1089.robot.auton;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.Debug;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class AutonDriveOnCurve extends Command {

	private double _headingAngle, _headingDisplacement;
	private PIDController pid1, pid2;
	
    public AutonDriveOnCurve(double headingAng, double headingDisp) {
    	_headingAngle = headingAng;
    	_headingDisplacement = headingDisp;
    	pid1 = new PIDController(.1, 0.0, 1.0, Robot.driveTrain.getNAVX(), Robot.driveTrain.getLeft());    	
    	pid2 = new PIDController(.1, 0, 1, Robot.driveTrain.getNAVX(), Robot.driveTrain.getRight());
    	pid1.setContinuous(true);
    	pid2.setContinuous(false);
    	pid1.setAbsoluteTolerance(0.1);
    	pid2.setAbsoluteTolerance(0.1);
    	pid1.setInputRange(-180, 180);
    	pid2.setInputRange(-200, 200);
    	pid1.setOutputRange(-.4, .4);
    	pid2.setOutputRange(-.4, .4);
    	LiveWindow.addActuator("Robot.driveTrain", "AutonDegreeRotate", pid1);
    	LiveWindow.addActuator("Robot.driveTrain", "AutonDriveDistance", pid2);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getGyro().reset();
    	Robot.driveTrain.getNAVX().reset();
    	pid1.setSetpoint(_headingAngle);
    	pid2.setSetpoint(_headingDisplacement);
		Debug.logMessage(Level.INFO, "The Auton Drive On Curve Command has been initialized.");

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return pid1.onTarget() && pid2.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
		Debug.logMessage(Level.INFO, "The Auton Drive On Curve Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	
	protected double returnPID1Input() {
		return Robot.driveTrain.getNAVX().getAngle();
	}

	protected double returnPID2Input() {
		return Robot.driveTrain.getNAVXDisplacementMagnitude();
	}

	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
//		Robot.driveTrain.getAutonRotatePidValue();
//		Robot.driveTrain.getAutonDriveDistancePidValue(output);
	}
}
