package org.usfirst.frc.team1089.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DegreeRotate extends PIDCommand {

	protected double _heading;
	private DoubleSupplier _angleSupplier = null;
	
    protected DegreeRotate() {
    	super(0.4, 0, 0.2);
    	MercLogger.logMessage(Level.INFO, "Entering DegreeRotate.DegreeRotate()");
    	_heading = 0.0;
    	requires(Robot.driveTrain);
    	LiveWindow.addActuator("Robot.driveTrain", "DegreeRotate", getPIDController());
    	
    }
    
    /**
     * Construct a DegreeRotate command that gets it angle during initialize by calling
     * the provided DoubleSupplier method
     * @param angleSupplier The DoubleSupplier method to output the angle
     */
    public DegreeRotate(DoubleSupplier angleSupplier) {
    	this();
    	MercLogger.logMessage(Level.INFO, "Entering DegreeRotate.DegreeRotate(DoubleSupplier angleSupplier)");
    	_angleSupplier = angleSupplier;
	}
    
    public DegreeRotate (double heading) {
    	this();
    	MercLogger.logMessage(Level.INFO, "Entering DegreeRotate.DegreeRotate(double heading)");
    	_heading = heading;
    }
    

    // Called just before this Command runs the first time
    protected void initialize() {
    	MercLogger.logMessage(Level.INFO, "Entering DegreeRotate.initialize()");
    	if (_angleSupplier != null) {
    		_heading = _angleSupplier.getAsDouble();
    	}
    	Robot.driveTrain.disableRobotDrive();
    	getPIDController().setContinuous(true);
    	getPIDController().setAbsoluteTolerance(0.15);
    	getPIDController().setInputRange(-180, 180);
    	getPIDController().setOutputRange(-.4, .4);   //was at -.5,.5
    	
    	MercLogger.logMessage(Level.INFO, "Before reset - Gyro reads: " + Robot.driveTrain.getGyro().getAngle() + " degrees.");
    	Robot.driveTrain.getGyro().reset();
    	MercLogger.logMessage(Level.INFO, "After reset - Gyro reads: " + Robot.driveTrain.getGyro().getAngle() + " degrees.");
    	//Timer.delay(10);
    	
    	MercLogger.logMessage(Level.INFO, "Previous Set Point" + getPIDController().getSetpoint() + " degrees.");
    	
    	getPIDController().setSetpoint(_heading);
    	MercLogger.logMessage(Level.INFO, "New Set Point " + getPIDController().getSetpoint() + " degrees.");
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
    	MercLogger.logMessage(Level.INFO, "Entering DegreeRotate.end()");
    	MercLogger.logMessage(Level.INFO, "Current setpoint: " + getPIDController().getSetpoint());
    	Robot.driveTrain.stop();
    	MercLogger.logMessage(Level.INFO, "Gyro reads: " + Robot.driveTrain.getGyro().getAngle() + " degrees.");
		Robot.driveTrain.enableRobotDrive();
		MercLogger.logMessage(Level.INFO, "The Degree Rotate Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	@Override
	protected double returnPIDInput() {
		return Robot.driveTrain.getGyro().getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		Robot.driveTrain.pidWrite(output);
	}
}
