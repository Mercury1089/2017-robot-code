package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootWithDistance extends RunShooter {

	private Shooter shooter;
	private double distance;
	private double speed;
	private double offset;
	
    public ShootWithDistance(Shooter s) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(s);
    	shooter = s;
    	distance = 0;
    	speed = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	super.initialize();
    	//SmartDashboard.putNumber("Shooter ID " + shooter.getMotor().getDeviceID() + ": distance", 0.0);
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Initialized");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	distance = SmartDashboard.getBoolean("Shooter ID " + shooter.getMotor().getDeviceID() + ": shooterIsRunning", false) ? SmartDashboard.getNumber("Shooter ID " + shooter.getMotor().getDeviceID() + ": distance", 0) : 0.0;
    	//distance = Robot.visionProcessor.getDistance(TargetType.HIGH_GOAL);
    	//distance -= 0.2;
    	offset = (14 - distance) / 28;
    	distance -= offset;
    	
    	speed = -0.00283 * Math.pow(distance, 2) + 0.914 * distance + 16.926;
    	speed /= 29.1585*10051;
    	
    	if(distance < 6)
    		speed = 0;
    	
    	if (speed == 0.0)
    		shooter.getMotor().disableControl();
    	else 
    		shooter.getMotor().enableControl();
    	
    	shooter.getMotor().set(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	super.end();
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Completed");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	super.interrupted();
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Interrupted");
    }
}