package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MotionProfile;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This {@link Command} will fill the top buffer of the CANTalons with
 * trajectory points
 */
public class PushTrajectoryPoints extends Command {
	CANTalon leftTalon, rightTalon;
	
	CANTalon.MotionProfileStatus statusLeft, statusRight;
	
    public PushTrajectoryPoints() {
    	requires(Robot.driveTrain);
    	
    	leftTalon = Robot.driveTrain.getLeft();
    	rightTalon = Robot.driveTrain.getRight();
    	
    	statusLeft = new CANTalon.MotionProfileStatus();
    	statusRight = new CANTalon.MotionProfileStatus();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// Change the control mode of both talons to MotionProfile
    	leftTalon.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
    	rightTalon.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
    	
    	Robot.driveTrain.resetMotionProfiling();
    	
    	CANTalon.TrajectoryPoint[] pointsLeft = MotionProfile.POINTS_L;
    	CANTalon.TrajectoryPoint[] pointsRight = MotionProfile.POINTS_R;
    	
    	// First points should have the zeroPos flag ticked
    	pointsLeft[0].zeroPos = true;
    	pointsRight[0].zeroPos = true;
    	
    	// Last points should have the isLastPoint flag ticked
    	pointsLeft[MotionProfile.NUM_POINTS - 1].isLastPoint = true;
    	pointsRight[MotionProfile.NUM_POINTS - 1].isLastPoint = true;
    	
    	// Push points into top buffer of talons
    	for (int i = 0; i < MotionProfile.NUM_POINTS; i++) {
    		leftTalon.pushMotionProfileTrajectory(pointsLeft[i]);
    		rightTalon.pushMotionProfileTrajectory(pointsRight[i]);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Process points in top buffer to move to bottom buffer
    	leftTalon.processMotionProfileBuffer();
    	rightTalon.processMotionProfileBuffer();
    	
    	// Update motion profile statuses
    	leftTalon.getMotionProfileStatus(statusLeft);
    	rightTalon.getMotionProfileStatus(statusRight);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// Only finish if the bottom buffer has at least 5 points OR
    	// if the buffer has all of our points
        return statusLeft.btmBufferCnt >= MotionProfile.MIN_POINTS && statusRight.btmBufferCnt >= MotionProfile.MIN_POINTS ||
    		   statusLeft.btmBufferCnt == MotionProfile.NUM_POINTS && statusRight.btmBufferCnt == MotionProfile.NUM_POINTS;
        
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
