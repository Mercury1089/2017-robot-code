package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Test the shooter with voltage instead of RPMs
 */
public class TestShooter extends Command {

	Shooter shooter;
	double pctVBus;
	
    public TestShooter(Shooter s) {
    	requires(s);
        shooter = s;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooter.setToVbus();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double volts = SmartDashboard.getBoolean("Shooter ID " + shooter.getMotor().getDeviceID() + ": shooterIsRunning", false) ? SmartDashboard.getNumber("Shooter ID " + shooter.getMotor().getDeviceID() + ": shooterVolts", 0) : 0.0;
    	
    	shooter.getMotor().set(volts);
   	}

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		MercLogger.logMessage(Level.INFO, "The Run Shooter Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }


}