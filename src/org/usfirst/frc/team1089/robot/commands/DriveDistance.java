package org.usfirst.frc.team1089.robot.commands;

//import org.usfirst.frc.team1089.robot.Logger;
import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveDistance extends Command {

    private double distance;
    private double endPosL, endPosR;
	
	public DriveDistance(double d) {
        
        requires(Robot.driveTrain);
        distance = d;
        endPosL = endPosR = Robot.driveTrain.inchesToEncoderTicks(distance);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.driveTrain.setToPosition();
		Robot.driveTrain.resetEncoders();
		Robot.driveTrain.disableRobotDrive();
    	
		Robot.driveTrain.getLeft().setPID(0.7, 0, 0.8);
		Robot.driveTrain.getRight().setPID(0.7, 0, 0.8);
		
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
    	Robot.driveTrain.setToVbus();
    	Robot.driveTrain.stop();
    	Robot.driveTrain.enableRobotDrive();
    	SmartDashboard.putNumber("EncRFinal", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()));
    	SmartDashboard.putNumber("EncLFinal", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()));
    	Robot.driveTrain.resetEncoders();
		SmartDashboard.putString("DriveDistance: ", "end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
		SmartDashboard.putString("DriveDistance: ", "interrupted");
    }
}
