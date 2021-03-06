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

	private static double MOVE_THRESHOLD = 0.1;
	private static final double MAX_DIST_FEET = 20.0;
	
	private double distance;
    private double endPosL, endPosR;
    private double waitTime;
    private DoubleSupplier distanceSupplier = null;
    private double maxV = 7.0;
    private double errorLeft, errorRight;
    private int noProgressCountdown;
    
    
    /**
     * <pre>
     * public DriveDistance(double distance,
     * </pre>
     * Creates this {@code Command} to drive the specified distance
     * and to wait the specified amount of milliseconds after the distance has been traveled.
     * 
     * @param distance the distance in inches to travel
     *                 after the distance has been driven
     */
	public DriveDistance(double distance) {
        this(distance, 7.0);
        MercLogger.logMessage(Level.INFO, "DriveDistance: Constructed using DriveDistance(double distance)");
	}
	
	/**
     * <pre>
     * public DriveDistance(double distance,
     *                      double maxV)
     * </pre>
     * Creates this {@code Command} to drive the specified distance
     * and to wait the specified amount of milliseconds after the distance has been traveled
     * while moving with a certain maximum voltage.
     * 
     * @param distance the distance in inches to travel
     * @param maxV the maximum voltage of the talons while driving forward, 
     *        set in initialize()
     */
	public DriveDistance(double distance, double maxV) {
        this.distance = distance;
        this.maxV = maxV;
        
        errorLeft =  errorRight = 0;
        noProgressCountdown = 100;
        
        MercLogger.logMessage(Level.INFO, "DriveDistance: Constructed using DriveDistance(double distance, double waitTime, double maxV)");
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
	public DriveDistance(DoubleSupplier distanceSupplier, double maxV) {
		this(0, maxV);
    	this.distanceSupplier = distanceSupplier;	
		
    	MercLogger.logMessage(Level.INFO, "DriveDistance: Constructed using DriveDistance(DoubleSupplier distanceSupplier, double maxV)");
	}
	
    // Called just before this Command runs the first time
    protected void initialize() {
    	if (distanceSupplier != null) {
    		distance = distanceSupplier.getAsDouble();
    	}
    	if (Math.abs(distance) > MAX_DIST_FEET)
			distance = 0;
    	
        endPosL = -Robot.driveTrain.feetToRevolutions(distance);
        endPosR = -endPosL;

		Robot.driveTrain.disableRobotDrive();
		Robot.driveTrain.setToPosition();
		Robot.driveTrain.resetEncoders();
    	
		Robot.driveTrain.getLeft().setPID(0.1, 0, 0.05);
		Robot.driveTrain.getRight().setPID(0.1, 0, 0.05);
		
		Robot.driveTrain.getLeft().configPeakOutputVoltage(maxV, -maxV);
		Robot.driveTrain.getLeft().configNominalOutputVoltage(0, 0);
		Robot.driveTrain.getRight().configPeakOutputVoltage(maxV, -maxV);
		Robot.driveTrain.getRight().configNominalOutputVoltage(0, 0);

		Robot.driveTrain.getLeft().enableControl();
		Robot.driveTrain.getRight().enableControl();
		
		Robot.driveTrain.getLeft().set(endPosL);
		Robot.driveTrain.getRight().set(endPosR);
		
		SmartDashboard.putString("EndPosVals", endPosL + ", " + endPosR);
		SmartDashboard.putString("DriveDistance: ", "Initialize");
		
		SmartDashboard.putNumber("Left Enc Inches", Robot.driveTrain.revolutionsToFeet(Robot.driveTrain.getLeftEncoder()) - SmartDashboard.getNumber("SetLeftChange", 0));
		SmartDashboard.putNumber("Right Enc Inches", Robot.driveTrain.revolutionsToFeet(Robot.driveTrain.getRightEncoder()) - SmartDashboard.getNumber("SetRightChange", 0));
		
		SmartDashboard.putNumber("Left Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.driveTrain.getRightEncoder());

        errorLeft = Math.abs(endPosL);
        errorRight = Math.abs(endPosR);
        noProgressCountdown = 100;
		
		MercLogger.logMessage(Level.INFO, "DriveDistance: Initialized with distance: " + distance + "feet.");
		MercLogger.logMessage(Level.INFO, "DriveDistance: Initialized with waitTime: " + waitTime + "milliseconds.");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putString("DriveDistance: ", "Execute");
    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
   		double leftPos = Robot.driveTrain.getLeft().get();
   		double rightPos = Robot.driveTrain.getRight().get();
   		
   		double newErrorLeft = Math.abs(endPosL) - Math.abs(leftPos);
   		double newErrorRight = Math.abs(endPosR) - Math.abs(rightPos);
   		
		SmartDashboard.putString("leftPos, rightPos", leftPos + ", " + rightPos);
		SmartDashboard.putString("DriveDistance: ", "isFinished");

		if ((leftPos > endPosL - MOVE_THRESHOLD && leftPos < endPosL + MOVE_THRESHOLD)
				&& (rightPos > endPosR - MOVE_THRESHOLD && rightPos < endPosR + MOVE_THRESHOLD)) {
   			return true;
   		}
		
		if ((newErrorLeft - errorLeft < MOVE_THRESHOLD && newErrorRight - errorRight < MOVE_THRESHOLD)) {
			noProgressCountdown--;
			if(noProgressCountdown <= 0) {
				return true;
			}
		} else {
			errorLeft = newErrorLeft;
			errorRight = newErrorRight;
		}
		return false;
    }
    // Called once after isFinished returns true
    protected void end() {
    	//MercLogger.logMessage(Level.INFO, "Entering DriveDistance.end()");
    	
    	Robot.driveTrain.setToVbus();
    	Robot.driveTrain.stop();
    	//Timer.delay(waitTime);
    	Robot.driveTrain.enableRobotDrive();
    	
    	SmartDashboard.putNumber("EncRFinal", Robot.driveTrain.encoderTicksToFeet(Robot.driveTrain.getRightEncoder()));
    	SmartDashboard.putNumber("EncLFinal", Robot.driveTrain.encoderTicksToFeet(Robot.driveTrain.getLeftEncoder()));
    	MercLogger.logMessage(Level.INFO, "LeftEnc Reads: " + Robot.driveTrain.encoderTicksToFeet(Robot.driveTrain.getLeftEncoder()) + 
    			", RightEnc Reads: " + Robot.driveTrain.encoderTicksToFeet(Robot.driveTrain.getRightEncoder()));
    	Robot.driveTrain.resetEncoders();
		SmartDashboard.putString("DriveDistance: ", "end");
		
		MercLogger.logMessage(Level.INFO, "DriveDistance: Completed");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	MercLogger.logMessage(Level.INFO, "DriveDistance: Interrupted");
    	end();
    }
}
