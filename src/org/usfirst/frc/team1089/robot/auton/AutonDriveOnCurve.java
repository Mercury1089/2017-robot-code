package org.usfirst.frc.team1089.robot.auton;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonDriveOnCurve extends Command {

	private double _headingXDirection, _headingYDirection;
	private PIDController anglePID, displacementPID;
	private double xDisplacement = 0;
	private double yDisplacement = 0;
	private double moveValue = 0;
	private double rotateValue = 0;
	
    public AutonDriveOnCurve(double headingXFeet, double headingYFeet) {
        requires(Robot.driveTrain);

    	PIDProxy angleProxy = new PIDProxy(this, PIDType.ANGLE);	 	
    	PIDProxy distanceProxy = new PIDProxy(this, PIDType.DISTANCE);
    	double headingXMeters = headingXFeet / 3.28084;			     //NavX takes values in meters, but they are given to the method in feet
    	double headingYMeters = headingYFeet / 3.28084;
    	_headingXDirection = headingXMeters;
    	_headingYDirection = headingYMeters;
    	
    	anglePID = new PIDController(.1, 0.0, 0, angleProxy, angleProxy); 	//How much is left to turn(right stick on arcadeDrive)   	
    	displacementPID = new PIDController(.1, 0.0, 0, distanceProxy, distanceProxy);	//How much is left to move(left stick on arcadeDrive)

    	LiveWindow.addActuator("Robot.driveTrain", "AutonDegreeRotate", anglePID);
    	LiveWindow.addActuator("Robot.driveTrain", "AutonDriveDistance", displacementPID);
    	MercLogger.logMessage(Level.INFO, "The Auton Drive On Curve Command has been constructed.");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getNAVX().reset();
    	Robot.driveTrain.setToVbus();
    	Robot.driveTrain.disableRobotDrive();
    	
    	anglePID.setContinuous(true);
    	displacementPID.setContinuous(false);
    	anglePID.setAbsoluteTolerance(1);
    	displacementPID.setAbsoluteTolerance(1);
    	anglePID.setInputRange(-180, 180);
    	displacementPID.setInputRange(-18, 18);
    	anglePID.setOutputRange(-1, 1);
    	displacementPID.setOutputRange(-.1, .1);    	
    	
    	anglePID.setSetpoint(0);		//Setpoints have to be 0 so that the error that is calculated is always the total error from setpoint
    	displacementPID.setSetpoint(0);
    	anglePID.enable();
    	displacementPID.enable();
    	
		MercLogger.logMessage(Level.INFO, "The Auton Drive On Curve Command has been initialized.");

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putData(Scheduler.getInstance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	/*return anglePID.get() == _headingXDirection && displacementPID.get() == _headingYDirection;
    	*/
    	if (xDisplacement >= _headingXDirection || yDisplacement >= _headingYDirection)
    		return true;
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	anglePID.disable();
    	displacementPID.disable();
    	Robot.driveTrain.stop();
    	Robot.driveTrain.getNAVX().reset();
    	Robot.driveTrain.enableRobotDrive();

    	MercLogger.logMessage(Level.INFO, "The Auton Drive On Curve Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	MercLogger.logMessage(Level.INFO, "The Auton Drive On Curve Command has entered interrupted.");
    	end();
    }
	
	public double getInitialAngleError() {
		return Math.toDegrees(Math.atan(_headingYDirection / _headingXDirection));	//Total angle error when we start
	}
	
	public double getNewAngleError() {
		xDisplacement += Robot.driveTrain.getNAVX().getDisplacementX();
		yDisplacement += Robot.driveTrain.getNAVX().getDisplacementY();
		Robot.driveTrain.getNAVX().resetDisplacement();
		double heading = Robot.driveTrain.getNAVX().getAngle();
		double complementMinusHeading = Math.toDegrees(Math.atan((_headingXDirection - xDisplacement) /	//Equal to complement of Angle Error minus Heading 
													  			 (_headingYDirection - yDisplacement)));
		return 90 - heading - complementMinusHeading;
	}
	
	public double getNewDisplacementValue() {		//New displacement needed after a move
		xDisplacement += Robot.driveTrain.getNAVX().getDisplacementX();
		yDisplacement += Robot.driveTrain.getNAVX().getDisplacementY();
		Robot.driveTrain.getNAVX().resetDisplacement();
		return Math.sqrt(Math.pow((_headingXDirection - xDisplacement), 2) + Math.pow((_headingYDirection - yDisplacement), 2));
	}
	
	public void setMoveValue(double mv) {
		moveValue = mv;
		Robot.driveTrain.arcadeDrive(moveValue, rotateValue);
	}
	
	public void setRotateValue(double rv) {
		rotateValue = rv;
		Robot.driveTrain.arcadeDrive(moveValue, rotateValue);

	}
	
	protected enum PIDType { ANGLE, DISTANCE }
	
	private class PIDProxy implements PIDSource, PIDOutput {
		
		private AutonDriveOnCurve m_ADOC;
		private PIDSourceType m_pidSource = PIDSourceType.kDisplacement;
		private PIDType m_pidType;
		
		public PIDProxy(AutonDriveOnCurve adoc, PIDType pt) {
			m_ADOC = adoc;
			m_pidType = pt;
		}
		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			if (m_pidType == PIDType.ANGLE)
				m_ADOC.setRotateValue(output);
			else {
				m_ADOC.setMoveValue(output);
			}
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			m_pidSource = pidSource;
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return m_pidSource;
		}

		@Override
		public double pidGet() {
			// TODO Auto-generated method stub
			return m_pidType == PIDType.ANGLE ? m_ADOC.getNewAngleError() : m_ADOC.getNewDisplacementValue();
		}
		
	}
}
