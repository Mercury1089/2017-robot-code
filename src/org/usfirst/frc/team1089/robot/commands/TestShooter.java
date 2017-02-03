package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TestShooter extends Command {

    public TestShooter() {
       requires(Robot.shooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.motor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	SmartDashboard.putNumber("shooterVolts", 0.0);
    	SmartDashboard.putBoolean("shooterIsRunning", false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double volts = SmartDashboard.getBoolean("shooterIsRunning", false) ? SmartDashboard.getNumber("shooterVolts", 0.0) : 0.0;
    	
    	Robot.shooter.motor.set(volts);
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
}
