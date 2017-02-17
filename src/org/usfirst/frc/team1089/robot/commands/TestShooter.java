package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TestShooter extends Command {

	Shooter shooter;
	double pctVBus;
	
    public TestShooter(Shooter s, double pct) {
    	requires(s);
        shooter = s;
        pctVBus = pct;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooter.motor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	shooter.motor.set(pctVBus);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
   	}

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		//MercLogger.logMessage(Level.INFO, "The Run Shooter Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }


}