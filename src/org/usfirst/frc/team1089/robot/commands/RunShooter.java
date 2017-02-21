	package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Command} runs a {@link Shooter} using SmartDashboard input.
 */
public class RunShooter extends Command {

	private double highest, lowest;
	private Shooter shooter;
	private final double LOWEST_RPM = 3500;		//TODO Change this
	private final double HIGHEST_RPM  = 5000;   //TODO CHange this
	
	/**
	 * <pre>
	 * public RunShooter(Shooter s)
	 * </pre>
	 * Creates this {@code RunShooter} command with the specifie
	 * {@code Shooter} to control
	 * 
	 * @param s the {@code Shooter} that this command will control 
	 */
	public RunShooter(Shooter s) {
    	requires(s);
    	shooter = s;
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	shooter.motor.changeControlMode(CANTalon.TalonControlMode.Speed);
    	shooter.motor.enableBrakeMode(false);
    	shooter.motor.setPID(0.7, 0.0, 0.2);
		shooter.motor.configPeakOutputVoltage(12, -12);
		shooter.motor.configNominalOutputVoltage(0,0);
		shooter.motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooter.motor.enableControl();
    	shooter.motor.reverseSensor(false);
		SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": shooterRPM", 0.0);
    	SmartDashboard.putBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": shooterIsRunning", false);
    	SmartDashboard.putBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": enableHighLow", false);
    	SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": distance", 0.0);
    	//Selectable
    	resetHighLow();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	double speed = SmartDashboard.getBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": shooterIsRunning", false) ? SmartDashboard.getNumber("Shooter ID " + shooter.motor.getDeviceID() + ": shooterRPM", 0) : 0.0;
    	
    	if (SmartDashboard.getBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": shooterIsRunning", false)) {
    		System.out.println(speed);
    	}
    	
    	double magVal = SmartDashboard.getNumber("Shooter ID " + shooter.motor.getDeviceID() + ": Encoder Value", 2900) * -1; // temp inverse
    	
    	
    	if (magVal < lowest) {
    		lowest = magVal;
    	}
    	if (magVal > highest) {
    		highest = magVal;
    	}
    	
    	if (!SmartDashboard.getBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": enableHighLow", false)) {
    		resetHighLow();
    	}

    	SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": LOWEST", lowest);
		SmartDashboard.putNumber("Shooter ID " + shooter.motor.getDeviceID() + ": HIGHEST", highest);
    		
    	shooter.motor.set(speed);
    	if (!inRange(speed))
    		end();
    	
    	System.out.println(shooter.motor.get());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    public void resetHighLow() {
		highest = 0;
		lowest = 20000;
	}
    
    public boolean inRange(double speed) {
    	return (speed > LOWEST_RPM && speed < HIGHEST_RPM);
    }
    
}
