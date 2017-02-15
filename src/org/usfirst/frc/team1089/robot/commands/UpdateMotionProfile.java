package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This {@link Command} handles controlling the talons during MotionProfile control mode.
 */
public class UpdateMotionProfile extends Command {

	CANTalon leftTalon, rightTalon;
	
	CANTalon.MotionProfileStatus statusLeft, statusRight;
	
	//MotionProfileExample example;
	
	private final double F_VAL;
	private final double RPM = 999;
	private final double NATIVE_UNITS_PER_ROTATION = 4096;
	private final double MAX_MOTOR_OUTPUT = 1023;
	
	
    public UpdateMotionProfile() {	
    	requires(Robot.driveTrain);
    	
    	leftTalon = Robot.driveTrain.getLeft();
    	rightTalon = Robot.driveTrain.getRight();
    	
    	statusLeft = new CANTalon.MotionProfileStatus();
    	statusRight = new CANTalon.MotionProfileStatus();
    	
    	F_VAL = MAX_MOTOR_OUTPUT / (RPM / 600 * NATIVE_UNITS_PER_ROTATION);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.driveTrain.resetEncoders();
		Robot.driveTrain.disableRobotDrive();
		
		MercLogger.logMessage(Level.INFO, "Motion Profile: Reset Encoders and Disabled Robot Drive");
		
		// Assume that control mode is set to motion profile mode
    	/*l.changeControlMode(TalonControlMode.MotionProfile);
    	r.changeControlMode(TalonControlMode.MotionProfile);*/
    	
    	leftTalon.setF(F_VAL);
    	rightTalon.setF(F_VAL);
    	
    	leftTalon.setPID(0.4, 0, 0);
    	rightTalon.setPID(0.4, 0, 0);
    	
    	leftTalon.configPeakOutputVoltage(10, -10);
		leftTalon.configNominalOutputVoltage(0, 0);
		rightTalon.configPeakOutputVoltage(10, -10);
		rightTalon.configNominalOutputVoltage(0, 0);

    	// Enables PID control on both talons
		leftTalon.enableControl();
    	rightTalon.enableControl();
    	
    	MercLogger.logMessage(Level.INFO, "Motion Profile: Enabled PID Contol");
    	
    	leftTalon.changeMotionControlFramePeriod(20);
    	rightTalon.changeMotionControlFramePeriod(20);
    	
    	leftTalon.set(CANTalon.SetValueMotionProfile.Enable.value);
    	rightTalon.set(CANTalon.SetValueMotionProfile.Enable.value);
    	
    	DriverStation.reportError("Value is initialized", true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Continue processing top buffer points to lower buffer points
    	leftTalon.processMotionProfileBuffer();
    	rightTalon.processMotionProfileBuffer();
    	
    	// Get the profile statuses for testing
    	leftTalon.getMotionProfileStatus(statusLeft);
    	rightTalon.getMotionProfileStatus(statusRight);

		MercLogger.logMessage(Level.INFO, "Motion Profile: Processed Motion Profile Buffer");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // If either of these blocks ran, this means that
        // there are points in the top buffer that have not yet been put into the bottom buffer
        // and therefore we are processing points faster than we are putting in.
        if (statusLeft.isUnderrun)
            MercLogger.logMessage(Level.INFO, "Motion Profile: Left Buffer Is Underrunning");

        if (statusRight.isUnderrun)
            MercLogger.logMessage(Level.INFO, "Motion Profile: Right Buffer Is Underrunning");
        
        return statusLeft.activePointValid && statusRight.activePointValid && statusLeft.activePoint.isLastPoint && statusRight.activePoint.isLastPoint;
    }

    // Called once after isFinished returns true
    protected void end() {
		MercLogger.logMessage(Level.INFO, "Motion Profile: ENDED");
    	
    	// Disable both talons
    	leftTalon.set(CANTalon.SetValueMotionProfile.Disable.value);
    	rightTalon.set(CANTalon.SetValueMotionProfile.Disable.value);
    	
    	// Go back to driving in arcade
    	Robot.driveTrain.enableRobotDrive();
    	leftTalon.changeControlMode(TalonControlMode.PercentVbus);
    	rightTalon.changeControlMode(TalonControlMode.PercentVbus);
    	
		MercLogger.logMessage(Level.INFO, "Motion Profile: Control Mode Changed Back to Percent VBus");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		MercLogger.logMessage(Level.INFO, "Motion Profile: Interupted");
    	end();
    }
    
    /*private void startFilling(TrajectoryPoint[] pointsL, TrajectoryPoint[] pointsR,  int totalCnt) {

		MercLogger.logMessage(Level.INFO, "Motion Profile: Started Filling");
		
		 This is fast since it's just into our TOP buffer 
		for (int i = 0; i < totalCnt; ++i) {
			CANTalon.TrajectoryPoint pointL = new CANTalon.TrajectoryPoint();
			CANTalon.TrajectoryPoint pointR = new CANTalon.TrajectoryPoint();
			
			 for each point, fill our structure and pass it to API 
			pointL.position = pointsL[i][0];
			pointL.velocity = pointsL[i][1];
			pointL.timeDurMs = (int) pointsL[i][2];
			pointL.profileSlotSelect = 0; 
			pointL.velocityOnly = false;  set true to not do any position
										 * servo, just velocity feedforward
										 		
			
			pointL.zeroPos = false;
			
			pointR.position = -pointsR[i][0];
			pointR.velocity = pointsR[i][1];
			pointR.timeDurMs = (int) pointsR[i][2];
			pointR.profileSlotSelect = 0;  which set of gains would you like to use? 
			pointR.velocityOnly = false;  set true to not do any position
										 * servo, just velocity feedforward
										 		
			pointR.zeroPos = false;
			
			// Zero position on first point only
			if (i == 0) {
				pointsL[i].zeroPos = true;
				pointsR[i].zeroPos = true;
			}
			
			/*pointsL.isLastPoint = false;
			pointR.isLastPoint = false;
			
			// Check flag isLastPoint on the last point
			if ((i + 1) == totalCnt) {
				pointsL[i].isLastPoint = true;
				pointsR[i].isLastPoint = true;
			}

			// Push the trajectory point to the top buffer
			leftTalon.pushMotionProfileTrajectory(pointsL[i]);
			rightTalon.pushMotionProfileTrajectory(pointsR[i]);
			
			MercLogger.logMessage(Level.INFO, "Motion Profile: Pushed Point");
			
			leftTalon.getMotionProfileStatus(statusLeft);
        	rightTalon.getMotionProfileStatus(statusRight);
        	
        	MercLogger.logMessage(Level.INFO, "Motion Profile: Left Talon Bottom Buffer Count = " + statusLeft.topBufferCnt);
        	MercLogger.logMessage(Level.INFO, "Motion Profile: Right Talon Bottom Buffer Count = " + statusRight.topBufferCnt);
			
			l.processMotionProfileBuffer();

			//DriverStation.reportError("Trajectory point status: ", true);
			DriverStation.reportError("Loop iteration completed", true);
		} */
}
