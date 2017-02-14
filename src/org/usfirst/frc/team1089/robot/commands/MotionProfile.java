package org.usfirst.frc.team1089.robot.commands;

import java.awt.Point;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.MotionProfileExample;
import org.usfirst.frc.team1089.robot.util.MotionProfileValues;

import com.ctre.CANTalon;
import com.ctre.CanTalonJNI;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotionProfile extends Command {

	CANTalon l, r;
	
	CANTalon.MotionProfileStatus lStatus, rStatus;
	
	//MotionProfileExample example;
	
	private final double F_VAL;
	private final double RPM = 999;
	private final double NATIVE_UNITS_PER_ROTATION = 4096;
	private final double MAX_MOTOR_OUTPUT = 1023;
	
	
    public MotionProfile() {
    	
    	requires(Robot.driveTrain);
    	l = Robot.driveTrain.getLeft();
    	r = Robot.driveTrain.getRight();
    	
    	lStatus = new CANTalon.MotionProfileStatus();
    	rStatus = new CANTalon.MotionProfileStatus();
    	
    	F_VAL = MAX_MOTOR_OUTPUT / (RPM / 600 * NATIVE_UNITS_PER_ROTATION);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.driveTrain.resetEncoders();
		Robot.driveTrain.disableRobotDrive();
		Robot.driveTrain.resetMotionProfiling();
		
		MercLogger.logMessage(Level.INFO, "Motion Profile: Reset Encoders and Disabled Robot Drive");
		
    	l.changeControlMode(TalonControlMode.MotionProfile);
    	r.changeControlMode(TalonControlMode.MotionProfile);
    	
    	l.setF(F_VAL);
    	r.setF(F_VAL);
    	
    	l.setPID(0.4,0,0);
    	r.setPID(0.4,0,0);
    	
    	l.configPeakOutputVoltage(10, -10);
		l.configNominalOutputVoltage(0, 0);
		r.configPeakOutputVoltage(10, -10);
		r.configNominalOutputVoltage(0, 0);
		
		// Fill top buffer with trajectory points
    	startFilling(MotionProfileValues.POINTS_L, MotionProfileValues.POINTS_R, MotionProfileValues.NUM_POINTS);
    	
    	// As long as the bottom buffer has not yet been filled, continue processing the top buffer.
    	while(lStatus.btmBufferCnt < MotionProfileValues.NUM_POINTS || rStatus.btmBufferCnt < MotionProfileValues.NUM_POINTS) {
    		MercLogger.logMessage(Level.INFO, "Filling bottom buffer...");
    		
        	l.processMotionProfileBuffer();
        	r.processMotionProfileBuffer();
        	
        	l.getMotionProfileStatus(lStatus);
        	r.getMotionProfileStatus(rStatus);
    	}
    	
    	MercLogger.logMessage(Level.INFO, "Bottom buffer filled!");

    	// Enables PID control on both talons
		l.enableControl();
    	r.enableControl();
    	
    	MercLogger.logMessage(Level.INFO, "Motion Profile: Enabled PID Contol");
    	
    	l.set(CANTalon.SetValueMotionProfile.Enable.value);
    	r.set(CANTalon.SetValueMotionProfile.Enable.value);
    	
    	DriverStation.reportError("Value is initialized", true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Continue processing top buffer points to lower buffer points
    	l.processMotionProfileBuffer();
    	r.processMotionProfileBuffer();
    	
    	// Get the profile status for testing
    	l.getMotionProfileStatus(lStatus);
    	r.getMotionProfileStatus(rStatus);

		MercLogger.logMessage(Level.INFO, "Motion Profile: Processed Motion Profile Buffer");
    	
		//MercLogger.logMessage(Level.INFO, "execute()");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        l.getMotionProfileStatus(lStatus);
        r.getMotionProfileStatus(rStatus);
        
        // If either of these blocks ran, this means that
        // there are points in the top buffer that have not yet been put into the bottom buffer
        // and therefore we are processing points faster than we are putting in.
        if (lStatus.isUnderrun)
            MercLogger.logMessage(Level.INFO, "Motion Profile: Left Buffer Is Underrunning");

        if (rStatus.isUnderrun)
            MercLogger.logMessage(Level.INFO, "Motion Profile: Right Buffer Is Underrunning");
        
        return lStatus.activePointValid && rStatus.activePointValid && lStatus.activePoint.isLastPoint && rStatus.activePoint.isLastPoint;
    }

    // Called once after isFinished returns true
    protected void end() {
		MercLogger.logMessage(Level.INFO, "Motion Profile: ENDED");
    	
    	// Disable both talons
    	l.set(CANTalon.SetValueMotionProfile.Disable.value);
    	r.set(CANTalon.SetValueMotionProfile.Disable.value);
    	
    	// Go back to driving in arcade
    	Robot.driveTrain.enableRobotDrive();
    	l.changeControlMode(TalonControlMode.PercentVbus);
    	r.changeControlMode(TalonControlMode.PercentVbus);
    	
		MercLogger.logMessage(Level.INFO, "Motion Profile: Control Mode Changed Back to Percent VBus");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		MercLogger.logMessage(Level.INFO, "Motion Profile: Interupted");
    	end();
    }
    
    private void startFilling(TrajectoryPoint[] pointsL, TrajectoryPoint[] pointsR,  int totalCnt) {

		MercLogger.logMessage(Level.INFO, "Motion Profile: Started Filling");
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
/*			CANTalon.TrajectoryPoint pointL = new CANTalon.TrajectoryPoint();
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
										 		
			pointR.zeroPos = false;*/
			
			// Zero position on first point only
			if (i == 0) {
				pointsL[i].zeroPos = true;
				pointsR[i].zeroPos = true;
			}
			
			/*pointsL.isLastPoint = false;
			pointR.isLastPoint = false;*/
			
			// Check flag isLastPoint on the last point
			if ((i + 1) == totalCnt) {
				pointsL[i].isLastPoint = true;
				pointsR[i].isLastPoint = true;
			}

			// Push the trajectory point to the top buffer
			l.pushMotionProfileTrajectory(pointsL[i]);
			r.pushMotionProfileTrajectory(pointsR[i]);
			
			MercLogger.logMessage(Level.INFO, "Motion Profile: Pushed Point");
			
			l.getMotionProfileStatus(lStatus);
        	r.getMotionProfileStatus(rStatus);
        	
        	MercLogger.logMessage(Level.INFO, "Motion Profile: Left Talon Bottom Buffer Count = " + lStatus.topBufferCnt);
        	MercLogger.logMessage(Level.INFO, "Motion Profile: Right Talon Bottom Buffer Count = " + rStatus.topBufferCnt);
			
/*			l.processMotionProfileBuffer();
*/
			//DriverStation.reportError("Trajectory point status: ", true);
			DriverStation.reportError("Loop iteration completed", true);
		}
    }
}
