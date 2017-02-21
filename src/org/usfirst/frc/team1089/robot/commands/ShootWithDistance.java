package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.subsystems.Shooter;

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
	
    public ShootWithDistance(Shooter s/*, double distanceFromTarget*/) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(s);
    	shooter = s;
    	distance = /*distanceFromTarget*/0;
    	speed = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	super.initialize();
    	//SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": distance", 0.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	distance = SmartDashboard.getBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": shooterIsRunning", false) ? SmartDashboard.getNumber("Shooter ID " + shooter.motor.getDeviceID() + ": distance", 0) : 0.0;
    	//distance -= 0.2;
    	offset = (14 - distance) / 28;
    	distance -= offset;
    	speed = -0.00283*Math.pow(distance, 2)+0.914*distance+16.926;
    	speed = speed/29.1585*10051;
    	shooter.motor.set(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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