package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootWithDistance extends Command {

	private Shooter shooterSystem;
	private double distance;
	private double speed;
	private double offset;
	private final int SPEED_THRESHOLD = 750;
	
    public ShootWithDistance(Shooter s) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(s);
    	shooterSystem = s;
    	distance = /*distanceFromTarget*/0;
    	speed = 0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Constructed using ShootWithDistance(Shooter s)");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooterSystem.setToSpeed();
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Initialized");
    	//SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": distance", 0.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	distance = SmartDashboard.getBoolean("Shooter ID " + shooterSystem.motor.getDeviceID() + ": shooterIsRunning",
    			false) ? SmartDashboard.getNumber("Shooter ID " + shooterSystem.motor.getDeviceID() + ": distance", 0) : 0.0;
    	//distance -= 0.2;
    	offset = (14 - distance) / 28;
    	distance -= offset;
    	
    	speed = -0.00283*Math.pow(distance, 2)+0.914*distance+16.926;
    	speed = speed/29.1585*10051;
    	if(distance < 6)
    		speed = 0;
    	
    	if (speed == 0) {
    		shooterSystem.motor.disableControl();
    		shooterSystem.feederMotor.set(0);
    	}
    	else {
    		shooterSystem.motor.enableControl();
    	}
    	
    	SmartDashboard.putNumber("Shooter ID " + shooterSystem.motor.getDeviceID() + ": Encoder Set Speed", speed);
    	shooterSystem.motor.set(speed);
    	
    	
    	//TODO Test what the encoder gets vs. what we set
    	if (Math.abs(shooterSystem.motor.getSpeed()) > (/*Math.pow(0.000004*speed,2) + */.8737 * speed + 20.877 - SPEED_THRESHOLD)
    			&& Math.abs(shooterSystem.motor.getSpeed()) < (/*Math.pow(0.000004*speed,2) +*/ .8737 * speed + 20.877 + SPEED_THRESHOLD)) { 
    	
    		shooterSystem.feederMotor.set(1);
    	}
    	
    	else
    		shooterSystem.feederMotor.set(0);
    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Completed");
    	//super.end();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		MercLogger.logMessage(Level.INFO, "ShootWithDistance: Shooter ID #" + shooterSystem.getMotor().getDeviceID() + " Interrupted.");
    	//super.interrupted();
    }
}