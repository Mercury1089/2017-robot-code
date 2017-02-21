package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon.TalonControlMode;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Command} uses the combination of PID and the CANTalon's
 * mag encoders to drive a specified set of inches.
 */
public class DriveDistance extends Command {

    private double distance;
    private double endPosL, endPosR;
    private double waitTime;
    private DoubleSupplier distanceSupplier = null;
    
    /**
     * <pre>
     * public DriveDistance(double distance,
     *                      double waitTime)
     * </pre>
     * Creates this {@code Command} to drive the specified distance
     * and to wait the specified amount of milliseconds after the distance has been traveled.
     * 
     * @param distance the distance in inches to travel
     * @param waitTime the amount of time to wait in milliseconds
     *                 after the distance has been driven
     */
	public DriveDistance(double distance, double waitTime) {
        requires(Robot.driveTrain);
        this.distance = distance;
        this.waitTime = waitTime;
        endPosL = endPosR = Robot.driveTrain.inchesToEncoderTicks(distance);
    }

	/**
     * <pre>
     * public DriveDistance(double distance)
     * </pre>
     * Creates this {@code Command} to drive the specified distance
     * and wait 0 milliseconds after travelling the specified distance.
     * 
     * @param distance the distance in inches to travel
     */
	public DriveDistance(double distance) {
		this(distance, 0);
    }
	
	/**
	 * <pre>
	 * public DriveDistance(DoubleSupplier distanceSupplier)
	 * </pre>
	 * Creates this {@code DriveDistance} with a {@link DoubleSupplier} that supplies
	 * the distance, and sets the wait time to 0 milliseconds.
	 * 
	 * @param distanceSupplier the {@code DoubleSupplier} to use to get the distance to travel
	 */
	public DriveDistance(DoubleSupplier distanceSupplier) {
		this(0, 0);
		this.distanceSupplier = distanceSupplier;	
	}
	
    // Called just before this Command runs the first time
    protected void initialize() {
    	if (distanceSupplier != null) {
    		distance = distanceSupplier.getAsDouble();
    	}

		Robot.driveTrain.setToPosition();
		Robot.driveTrain.resetEncoders();
		Robot.driveTrain.disableRobotDrive();
    	
		Robot.driveTrain.getLeft().setPID(1, 0, 0);
		Robot.driveTrain.getRight().setPID(1, 0, 0);
		
		Robot.driveTrain.getLeft().configPeakOutputVoltage(10, -10);
		Robot.driveTrain.getLeft().configNominalOutputVoltage(0, 0);
		Robot.driveTrain.getRight().configPeakOutputVoltage(10, -10);
		Robot.driveTrain.getRight().configNominalOutputVoltage(0, 0);
		

		Robot.driveTrain.getLeft().set(endPosL);
		Robot.driveTrain.getRight().set(-endPosR);
		
		Robot.driveTrain.getLeft().enableControl();
		Robot.driveTrain.getRight().enableControl();
		SmartDashboard.putString("EndPosVals", endPosL + ", " + endPosR);
		SmartDashboard.putString("DriveDistance: ", "Initialize");
		
		SmartDashboard.putNumber("Left Enc Inches", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()) - SmartDashboard.getNumber("SetLeftChange", 0));
		SmartDashboard.putNumber("Right Enc Inches", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) - SmartDashboard.getNumber("SetRightChange", 0));
		
		SmartDashboard.putNumber("Left Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.driveTrain.getRightEncoder());
		MercLogger.logMessage(Level.INFO, "The Drive Distance Command has been initialized. Moving " + distance + " inches.");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
		SmartDashboard.putString("DriveDistance: ", "Execute");
    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
   		double leftPos = Robot.driveTrain.getLeftEncoder();
   		double rightPos = Robot.driveTrain.getRightEncoder();
   		
		SmartDashboard.putString("leftPos, rightPos", leftPos + ", " + rightPos);
		SmartDashboard.putString("DriveDistance: ", "isFinished");

   		if ((distance > 0 && leftPos < endPosL && rightPos < endPosR) 
   				|| (distance < 0 && leftPos > endPosL && rightPos > endPosR)) {
   			return false;
   		}
   		
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Timer.delay(waitTime);
    	Robot.driveTrain.setToVbus();
    	Robot.driveTrain.stop();
    	Robot.driveTrain.enableRobotDrive();
    	SmartDashboard.putNumber("EncRFinal", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()));
    	SmartDashboard.putNumber("EncLFinal", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()));
    	Robot.driveTrain.resetEncoders();
		SmartDashboard.putString("DriveDistance: ", "end");
		MercLogger.logMessage(Level.INFO, "The Drive Distance Command has ended.");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
		MercLogger.logMessage(Level.INFO, "The DriveDistance Command has been interrupted.");
    }
}
