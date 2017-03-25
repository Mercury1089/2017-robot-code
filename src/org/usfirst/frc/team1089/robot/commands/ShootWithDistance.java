package org.usfirst.frc.team1089.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
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
    	SmartDashboard.putNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": Encoder Set", 2000.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	if (distance == 0) {
//    		distance = SmartDashboard.getBoolean("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": shooterIsRunning",
//        			false) ? SmartDashboard.getNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": distance", 0) : 0.0;
    	
    	//distance = (Math.abs(distance) > 16 || Math.abs(distance) < 6 ? 8 : Robot.visionProcessor.getDistanceUsingVerticalInformation(TargetType.HIGH_GOAL));
    	distance = Robot.visionProcessor.getDistanceUsingVerticalInformation(TargetType.HIGH_GOAL);
    	
    	//speed = (Math.abs(distance) > 14 || Math.abs(distance) < 6 ? 3000 : (distance-6)/8*5000);
    	
//    	speed = 2000;
    	
    	/*speed = SmartDashboard.getBoolean("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": shooterIsRunning",
       		false) ? SmartDashboard.getNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": Encoder Set", 0) : 0.0;*/
    	
/*    	shooterSystem.shooterMotor.set(1);
    	shooterSystem.feederMotor.set(0);*/
    	
//    	distance += 20.5/12;
    	/*offset = (14 - distance) / 28;
    	distance -= offset;*/
    	
    	if ( distance >= 6 && distance <= 14) {
    		speed = -0.00283*Math.pow(distance, 2)+0.914*distance+16.926;	//temporary, will work for 12 ft and under
        	speed = speed/29.1585 * 3600;		//XXX Attempt at converting big boy to little enc	
    	} else {
    		speed = 0;
    	}
    	
    	//(.9459*6000)
    	
    	if(speed != shooterSystem.getSetSpeed()) {
        	SmartDashboard.putNumber("Shooter ID " + shooterSystem.shooterMotor.getDeviceID() + ": Encoder Set Speed", speed);
        	shooterSystem.setSpeed(speed);    		
    	}
    	   	
    	if (Math.abs(shooterSystem.getSetSpeed()) > 0 &&
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