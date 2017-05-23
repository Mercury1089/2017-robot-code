package org.usfirst.frc.team1089.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.ITargetProvider;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

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
	private final int SPEED_THRESHOLD = 300;
	//private DoubleSupplier distanceSupplier;
	private ITargetProvider targetProvider;
	
    public ShootWithDistance(Shooter s) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(s);
    	shooterSystem = s;
    	distance = 0;
    	speed = 0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Constructed using ShootWithDistance(Shooter s)");
    }
    
    /*public ShootWithDistance(Shooter s, DoubleSupplier dp) {
    	requires(s);
    	shooterSystem = s;
    	distanceSupplier = dp;
    	speed = 0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Constructed using ShootWithDistance(Shooter s)");
    }*/
    
    public ShootWithDistance(Shooter s, ITargetProvider tP) {
    	requires(s);
    	shooterSystem = s;
    	targetProvider = tP;
    	speed = 0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Constructed using ShootWithDistance(Shooter s)");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooterSystem.setToSpeed();
    	distance = 0.0;
    	
    	MercLogger.logMessage(Level.INFO, "ShootWithDistance: Initialized");
    	SmartDashboard.putNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": Encoder Set", 2000.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//distance = Robot.visionProcessor.getDistanceUsingVerticalInformation(TargetType.HIGH_GOAL);
    	distance = targetProvider.getDistance();
    	
    	if ( distance >= 6 && distance <= 14) {
    		speed = -0.00283*Math.pow(distance, 2)+0.914*distance+16.926;	//temporary, will work for 12 ft and under
        	speed = speed/29.1585 * 3600;		//XXX Attempt at converting big boy to little enc	
    	} else {
    		speed = 0;
    	}
    	
    	if(speed != shooterSystem.getSetSpeed()) {
        	SmartDashboard.putNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": Encoder Set Speed", speed);
        	shooterSystem.setSpeed(speed);    		
    	}
    	   	
    	if (	targetProvider.isOnTarget() &&
    			Math.abs(shooterSystem.getSetSpeed()) > 0 &&
    			Math.abs(shooterSystem.getSpeed()) > speed - SPEED_THRESHOLD) { 
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