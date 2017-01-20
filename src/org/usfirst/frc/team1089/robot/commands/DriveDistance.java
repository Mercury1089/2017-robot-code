package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;

/**
 *
 */
public class DriveDistance extends Command {

    private double distance;
    private final double GEAR_RATIO = 10.75;
    private final double WHEEL_DIAMETER = 4.0;
    
    private final double DIST_PER_TICK_INCHES = 
    		1440 * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER);
    
    private Encoder encoderR, encoderL;
    private DriveTrain d;
	
	public DriveDistance(double distance) {
        super("DriveDistance");
        requires(Robot.driveTrain);
        this.distance = distance;
        encoderR = new Encoder(4, 0);
        encoderL = new Encoder(5, 0);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	encoderL.reset();
    	encoderR.reset();
    	//if ()
    	//	Robot.driveTrain.drive(0.75, 0.75);   // Need drive(double a, double b) method in DriveTrain
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//Robot.driveTrain.drive(0, 0);
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
