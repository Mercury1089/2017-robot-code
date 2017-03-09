package org.usfirst.frc.team1089.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Command} calculates the speed to set the shooters
 * depending on distance from the boiler and runs the shooters
 */
public class ShootWithDistance extends Command {

	private Shooter shooterSystem;
	private double distance;
	private double speed;
	private double offset;
	private final int SPEED_THRESHOLD = 200;
	private DoubleSupplier distanceSupplier;

	
    public ShootWithDistance(Shooter s) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(s);
    	shooterSystem = s;
    	distance = 0;
    	speed = 0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Constructed using ShootWithDistance(Shooter s)");
    }
    
    public ShootWithDistance(Shooter s, DoubleSupplier d) {
    	requires(s);
    	shooterSystem = s;
    	distanceSupplier = d;
    	speed = 0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Constructed using ShootWithDistance(Shooter s)");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooterSystem.setToSpeed();
    	if (distanceSupplier != null)
    		distance = distanceSupplier.getAsDouble();
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Initialized");
    	//SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": distance", 0.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (distance == 0) {
    		distance = SmartDashboard.getBoolean("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": shooterIsRunning",
        			false) ? SmartDashboard.getNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": distance", 0) : 0.0;
    	}
/*    	shooterSystem.shooterMotor.set(1);
    	shooterSystem.feederMotor.set(0);*/
    	
    	//distance -= 0.2;
    	//offset = (14 - distance) / 28;
    	//distance -= offset;
    	
/*    	speed = -0.00283*Math.pow(distance, 2)+0.914*distance+16.926;
    	speed = speed/29.1585*10051(.9459*6021);		//XXX Attempt at converting big boy to little enc
*/    	
  	
    	if(distance < 6)
    		speed = 0;
    	else {
    		speed=(distance-6)/8*5000;
    	}
    	
    	if(speed != shooterSystem.getSetSpeed()) {
        	if (speed == 0) {
        		shooterSystem.runFeeder(false);;
        	}
        	SmartDashboard.putNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": Encoder Set Speed", speed);
        	shooterSystem.setSpeed(speed);    		
    	}
    	
    	
    	//TODO Test what the encoder gets vs. what we set
//    	if (Math.abs(shooterSystem.shooterMotor.getSpeed()) > /*(Math.pow(0.000004*speed,2)*/ + .8737 * speed + 20.877 - SPEED_THRESHOLD
//    			&& Math.abs(shooterSystem.shooterMotor.getSpeed()) < /*(Math.pow(0.000004*speed,2)*/ + .8737 * speed + 20.877 + SPEED_THRESHOLD) { 
    	
    	if (Math.abs(shooterSystem.getSpeed()) > speed - SPEED_THRESHOLD
    			&& Math.abs(shooterSystem.getSpeed()) < speed + SPEED_THRESHOLD) { 
    	
    		shooterSystem.runFeeder(true);
    	}
    	
    	else
    		shooterSystem.runFeeder(false);
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