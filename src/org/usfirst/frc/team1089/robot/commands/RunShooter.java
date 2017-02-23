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
    	//Selectable
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	double speed = SmartDashboard.getBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": shooterIsRunning", false) ? SmartDashboard.getNumber("Shooter ID " + shooter.motor.getDeviceID() + ": shooterRPM", 0) : 0.0;
    		

    	if (!SmartDashboard.getBoolean("Shooter ID " + shooter.motor.getDeviceID() + ": enableHighLow", false)) {
    		shooter.resetHighLow();
    	}
 		
    	shooter.motor.set(speed);
    	if (!inRange(speed))
    		end();
    	
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
    

    
    public boolean inRange(double speed) {
    	return (speed > LOWEST_RPM && speed < HIGHEST_RPM);
    }
    
}
