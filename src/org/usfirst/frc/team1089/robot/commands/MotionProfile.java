package org.usfirst.frc.team1089.robot.commands;

import java.awt.Point;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MotionProfileExample;
import org.usfirst.frc.team1089.robot.util.MotionProfileValues;

import com.ctre.CANTalon;
import com.ctre.CanTalonJNI;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotionProfile extends Command {

	CANTalon l, r;
	
	CANTalon.MotionProfileStatus lStatus, rStatus;
	
	MotionProfileExample example;
	
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
		
    	l.changeControlMode(TalonControlMode.MotionProfile);
    	r.changeControlMode(TalonControlMode.MotionProfile);
    	
    	l.setF(F_VAL);
    	r.setF(F_VAL);
    	
    	l.setPID(0.1,0,1);
    	r.setPID(0.1,0,1);
    	
    	l.configPeakOutputVoltage(10, -10);
		l.configNominalOutputVoltage(0, 0);
		r.configPeakOutputVoltage(10, -10);
		r.configNominalOutputVoltage(0, 0);
		
		
		l.enableControl();
    	r.enableControl();
    	
    	
    	startFilling(MotionProfileValues.Points, MotionProfileValues.kNumPoints);
    	
    	
    	DriverStation.reportError("Starting code after startFilling()", true);
    	l.processMotionProfileBuffer();
    	r.processMotionProfileBuffer();
    	
    	l.set(CANTalon.SetValueMotionProfile.Enable.value);
    	r.set(CANTalon.SetValueMotionProfile.Enable.value);
    	
    	DriverStation.reportError("Value is initialized", true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//DriverStation.reportError("Execute", true);
/*    	l.set(CANTalon.SetValueMotionProfile.Enable.value);
    	r.set(CANTalon.SetValueMotionProfile.Enable.value);
    	*/
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        l.getMotionProfileStatus(lStatus);
        r.getMotionProfileStatus(rStatus);
    	return lStatus.activePoint.isLastPoint && rStatus.activePoint.isLastPoint;
    }

    // Called once after isFinished returns true
    protected void end() {
    	l.changeControlMode(TalonControlMode.PercentVbus);
    	r.changeControlMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    
    private void startFilling(double[][] profile, int totalCnt) {

		/* create an empty point */
		CANTalon.TrajectoryPoint pointL = new CANTalon.TrajectoryPoint();
		CANTalon.TrajectoryPoint pointR = new CANTalon.TrajectoryPoint();

		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		l.clearMotionProfileTrajectories();
		r.clearMotionProfileTrajectories();

		DriverStation.reportError("About to enter loop", true);
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			/* for each point, fill our structure and pass it to API */
			pointL.position = 1024 * profile[i][0];
			pointL.velocity = profile[i][1];
			pointL.timeDurMs = (int) profile[i][2];
			pointL.profileSlotSelect = 0; /* which set of gains would you like to use? */
			pointL.velocityOnly = false; /* set true to not do any position
										 * servo, just velocity feedforward
										 */		
			
			pointL.zeroPos = false;
			if (i == 0)
				pointL.zeroPos = true; /* set this to true on the first point */

			pointL.isLastPoint = false;
			if ((i + 1) == totalCnt)
				pointL.isLastPoint = true; /* set this to true on the last point  */

			l.pushMotionProfileTrajectory(pointL);
			//r.pushMotionProfileTrajectory(point);
			
			DriverStation.reportError("Loop iteration completed", true);
		}
		
		for (int j = 0; j < totalCnt; ++j) {
			/* for each point, fill our structure and pass it to API */
			pointR.position = 1024 * -profile[j][0];
			pointR.velocity = profile[j][1];
			pointR.timeDurMs = (int) profile[j][2];
			pointR.profileSlotSelect = 0; /* which set of gains would you like to use? */
			pointR.velocityOnly = false; /* set true to not do any position
										 * servo, just velocity feedforward
										 */		
			
			pointR.zeroPos = false;
			if (j == 0)
				pointR.zeroPos = true; /* set this to true on the first point */

			pointR.isLastPoint = false;
			if ((j + 1) == totalCnt)
				pointR.isLastPoint = true; /* set this to true on the last point  */

			r.pushMotionProfileTrajectory(pointR);
			//r.pushMotionProfileTrajectory(point);
			
		
		
		DriverStation.reportError("startFilling() has ended", true);
	}

    }
}
