package org.usfirst.frc.team1089.robot.commands;

//import org.usfirst.frc.team1089.robot.Logger;
import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;

/**
 *
 */
public class DriveDistance extends Command {

    private double distance;
    /*private static final double GEAR_RATIO = 10.75;
    private static final double WHEEL_DIAMETER = 4.0;
    
    private static final double DIST_PER_TICK_INCHES = 
    		1440 * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER);
    
    private Encoder encoderR, encoderL;
    private DriveTrain d;*/
    
    private double endPosL, endPosR;
	
	public DriveDistance(double distance) {
        super("DriveDistance");
        requires(Robot.driveTrain);
        this.distance = distance;
        /*encoderR = new Encoder(4, 0);
        encoderL = new Encoder(5, 0);*/
        
        endPosL = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()) + distance;
        endPosR = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) + distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	double changePos = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) + distance;
		double startPosL = Robot.driveTrain.getLeftEncoder();
		double startPosR = Robot.driveTrain.getRightEncoder();
		/*leftFrontTalon.setPID(p, i, d);
		rightFrontTalon.setPID(p, i, d);
		leftFrontTalon.configPeakOutputVoltage(maxV, -maxV);
		leftFrontTalon.configNominalOutputVoltage(0, 0);
		rightFrontTalon.configPeakOutputVoltage(maxV, -maxV);
		rightFrontTalon.configNominalOutputVoltage(0.0, 0.0);
		setToAuto();*/
		
		Robot.driveTrain.setRight(0.75);
		Robot.driveTrain.setLeft(0.75);
		
		/*leftFrontTalon.enableControl();
		rightFrontTalon.enableControl();
		leftFrontTalon.set(endPosL);
		rightFrontTalon.set(endPosR);*/
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	/*encoderL.reset();
    	encoderR.reset();
    	if ()
    		Robot.driveTrain.drive(0.75, 0.75);*/   // Need drive(double a, double b) method in DriveTrain
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
   		double leftPos = Robot.driveTrain.getLeftEncoder();
   		double rightPos = Robot.driveTrain.getRightEncoder();/*
   		double leftVel = leftFrontTalon.getEncVelocity();
   		double rightVel = rightFrontTalon.getEncVelocity();*/
   		//boolean isMoving = false;
   		
   		/*double endPosL = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()) + distance;
   		double endPosR = Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) + distance;*/
   		
    	
   		/*SmartDashboard.putNumber("left velocity", leftVel);
   		SmartDashboard.putNumber("right velocity", rightVel);
   		SmartDashboard.putNumber("left pos", leftPos);
   		SmartDashboard.putNumber("right pos", rightPos);*/
    			
   		if ((leftPos > endPosL - distance && leftPos < endPosL + distance)
   				&& (rightPos > endPosR - distance && rightPos < endPosR + distance)
   				/*&& Math.abs(leftVel) <= TURN_THRESH_VELOCITY && Math.abs(rightVel) <= TURN_THRESH_VELOCITY*/) {
   			return true;
   		}
   		
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
