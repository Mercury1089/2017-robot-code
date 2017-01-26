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
    	//double changePos = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) + distance;
		//double startPosL = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder());
		//double startPosR = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder());
    	
		Robot.driveTrain.getLeft().setPID(1, 0, 0);
		Robot.driveTrain.getRight().setPID(1, 0, 0);
		Robot.driveTrain.getLeft().configPeakOutputVoltage(0.5, -0.5);
		Robot.driveTrain.getLeft().configNominalOutputVoltage(0, 0);
		Robot.driveTrain.getRight().configPeakOutputVoltage(0.5, -0.5);
		Robot.driveTrain.getRight().configNominalOutputVoltage(0.0, 0.0);

        Robot.driveTrain.resetEncoders();
		Robot.driveTrain.setToPosition();
		
		Robot.driveTrain.getLeft().enableControl();
		Robot.driveTrain.getRight().enableControl();
		Robot.driveTrain.getLeft().set(endPosL);
		Robot.driveTrain.getRight().set(endPosR);
		SmartDashboard.putString("EndPosVals", endPosL + ", " + endPosR);
		SmartDashboard.putString("DriveDistance: ", "Initialize");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	/*encoderL.reset();
    	encoderR.reset();
    	if ()
    		Robot.driveTrain.drive(0.75, 0.75);*/   // Need drive(double a, double b) method in DriveTrain
    	/*
    	Robot.driveTrain.setRight(0.75);
		Robot.driveTrain.setLeft(0.75);*/
		SmartDashboard.putString("DriveDistance: ", "Execute");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
   		double leftPos = Robot.driveTrain.getLeftEncoder();
   		double rightPos = Robot.driveTrain.getRightEncoder();
   		/*double leftVel = leftFrontTalon.getEncVelocity();
   		double rightVel = rightFrontTalon.getEncVelocity();*/
   		//boolean isMoving = false;
   		
   		/*double endPosL = Robot.driveTrain.encoderTicksTInches(Robot.driveTrain.getLeftEncoder()) + distance;
   		double endPosR = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) + distance;*/
   		
    	
   		/*SmartDashboard.putNumber("left velocity", leftVel);
   		SmartDashboard.putNumber("right velocity", rightVel);
   		SmartDashboard.putNumber("left pos", leftPos);
   		SmartDashboard.putNumber("right pos", rightPos);*/
		SmartDashboard.putString("leftPos, rightPos", leftPos + ", " + rightPos);
		SmartDashboard.putString("DriveDistance: ", "isFinished");

   		if ((leftPos < endPosL) && (rightPos < endPosR)
   				/*&& Math.abs(leftVel) <= TURN_THRESH_VELOCITY && Math.abs(rightVel) <= TURN_THRESH_VELOCITY*/) {
   			
   			return false;
   		}
   		
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.setToVbus();
    	Robot.driveTrain.stop();
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
