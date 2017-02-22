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
 *
 */
public class DriveDistance extends Command {

    private double distance;
    private double endPosL, endPosR;
    private double waitTime;
    private DoubleSupplier distanceSupplier = null;
    private double maxV = 10.0;
    
	public DriveDistance(double distance, double waitTime) {
        requires(Robot.driveTrain);
        this.distance = distance;
        this.waitTime = waitTime;
        endPosL = endPosR = Robot.driveTrain.inchesToEncoderTicks(distance);
    }

	public DriveDistance(double distance) {
		this(distance, 0);
    }
	
	public DriveDistance(DoubleSupplier distanceSupplier, double maxV) {
		this(0, 0.4);
    	MercLogger.logMessage(Level.INFO, "Entering DriveDistance.DriveDistance()");
		this.distanceSupplier = distanceSupplier;	
		this.maxV = maxV;
	}

	
    // Called just before this Command runs the first time
    protected void initialize() {
    	MercLogger.logMessage(Level.INFO, "Entering DriveDistance.initialize()");
    	if (distanceSupplier != null) {
    		distance = distanceSupplier.getAsDouble();
    		endPosL = endPosR = Robot.driveTrain.inchesToEncoderTicks(distance);
    	}

		Robot.driveTrain.setToPosition();
		Robot.driveTrain.resetEncoders();
		Robot.driveTrain.disableRobotDrive();
    	
		Robot.driveTrain.getLeft().setPID(1, 0, 0);
		Robot.driveTrain.getRight().setPID(1, 0, 0);
		
		Robot.driveTrain.getLeft().configPeakOutputVoltage(maxV, -maxV);
		Robot.driveTrain.getLeft().configNominalOutputVoltage(0, 0);
		Robot.driveTrain.getRight().configPeakOutputVoltage(maxV, -maxV);
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

		MercLogger.logMessage(Level.INFO, "DriveDistance.isFinished() will return true.");
   		
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	MercLogger.logMessage(Level.INFO, "Entering DriveDistance.end()");
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
