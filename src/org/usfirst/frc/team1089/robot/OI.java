package org.usfirst.frc.team1089.robot;

import org.usfirst.frc.team1089.robot.auton.AutonEnum;
import org.usfirst.frc.team1089.robot.auton.AutonFirstAction;
import org.usfirst.frc.team1089.robot.auton.AutonFirstMovement;
import org.usfirst.frc.team1089.robot.auton.AutonPosition;
import org.usfirst.frc.team1089.robot.auton.AutonSecondAction;
import org.usfirst.frc.team1089.robot.auton.AutonSecondMovement;
import org.usfirst.frc.team1089.robot.commands.*;
import org.usfirst.frc.team1089.robot.commands.CalculateGearPath.Direction;
import org.usfirst.frc.team1089.robot.subsystems.Shooter.ShooterEnum;
import org.usfirst.frc.team1089.robot.util.Utilities;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public static final double JS_DEADZONE_LIMIT = 0.3; // Deadzone limit for the stick	
	
	public AutonEnum step3;
	
	SendableChooser startPosition, shooterType, firstAction, secondAction,
		movementOffStart, nextMovement;
	
	Alliance allianceColor;

    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	public Joystick leftStick;
//    public JoystickButton leftBtn1;
//    public JoystickButton leftBtn3;

    public Joystick rightStick;
//    public JoystickButton rightBtn1;
//    public JoystickButton rightBtn2;

    public Joystick gamePad;
    public JoystickButton gamePadBtnA;
    public JoystickButton gamePadBtnB;
    public JoystickButton gamePadBtnY;
    public JoystickButton gamePadBtnX;
	public JoystickButton rightBack;
	public JoystickButton leftBack;
	public JoystickButton rightStick4;
	public JoystickButton rightStick5;
    public JoystickButton rightStick2;
    public OI() {
    	
        leftStick = new Joystick(RobotMap.DS_USB.LEFT_STICK);
        rightStick = new Joystick(RobotMap.DS_USB.RIGHT_STICK);
        gamePad = new Joystick(RobotMap.DS_USB.GAMEPAD);
        gamePadBtnA = new JoystickButton(gamePad, RobotMap.GamepadButtons.A);
        gamePadBtnA.whenPressed(new DriveWithJoysticks());
        //gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        //gamePadBtnB.whenPressed(Robot.driveTrain.);
        gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        gamePadBtnB.whenPressed(new DriveDistance(2));
        gamePadBtnY = new JoystickButton(gamePad, RobotMap.GamepadButtons.Y);
        gamePadBtnY.whenPressed(new  ToggleGearDelivery(true));
        
        gamePadBtnX = new JoystickButton(gamePad, RobotMap.GamepadButtons.X);
        /*gamePadBtnX.whenPressed(new RunMotionProfile());     */
        gamePadBtnX.whenPressed(new  ToggleGearDelivery(false));
        //gamePadBtnX.whenPressed(new AutonDriveOnCurve(2, 3));
        /*gamePadBtnX.whenPressed(new MotionProfile());*/
        
        //gamePadBtnX.whenPressed(new AutonDriveOnCurve(5, 7));
        //gamePadBtnX.whenPressed(new RunMotionProfile());     
        
        rightBack = new JoystickButton(gamePad, RobotMap.GamepadButtons.RB);
        rightBack.whenPressed(new DegreeRotate(60));
        
        leftBack = new JoystickButton(gamePad, RobotMap.GamepadButtons.LB);
        leftBack.whenPressed(new DeliverGear());        
        
/*        rightStick4 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN4);
        rightStick4.whenPressed(new SetRoller(Robot.rightFeeder, 1));
*/
        rightStick5 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN5);
        rightStick5.whenPressed(new CalculateGearPath(Direction.REVERSE));
        
/*        rightStick2 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN2);
        rightStick2.whenPressed(new SetRoller(Robot.rightFeeder, 0));*/
    	//// TRIGGERING COMMANDS WITH BUTTONS
    	// Once you have a button, it's trivial to bind it to a button in one of
    	// three ways:

    	// Start the command when the button is pressed and let it run the command
    	// until it is finished as determined by it's isFinished method.
    	// button.whenPressed(new ExampleCommand());

    	// Run the command while the button is being held down and interrupt it once
    	// the button is released.
    	// button.whileHeld(new ExampleCommand());

    	// Start the command when the button is released and let it run the command
    	// until it is finished as determined by it's isFinished method.
    	// button.whenReleased(new ExampleCommand());
        
        startPosition = new SendableChooser();
		startPosition.addDefault("Left Corner: 1", AutonPosition.POSITION_1);
		startPosition.addObject("Left, Left Line: 2", AutonPosition.POSITION_2);
		startPosition.addObject("Right, Left Line: 3", AutonPosition.POSITION_3);
		startPosition.addObject("Left, Midline: 4", AutonPosition.POSITION_4);
		startPosition.addObject("Mid, Midline: 5", AutonPosition.POSITION_5);
		startPosition.addObject("Right, Midline: 6", AutonPosition.POSITION_6);
		startPosition.addObject("Left, Right Line: 7", AutonPosition.POSITION_7);
		startPosition.addObject("Right, Right Line: 8", AutonPosition.POSITION_8);
		startPosition.addObject("Right Corner: 9", AutonPosition.POSITION_9);
		SmartDashboard.putData("Starting Position", startPosition);
		
		movementOffStart = new SendableChooser();
		movementOffStart.addDefault("Do nothing", AutonFirstMovement.DO_NOTHING);
		movementOffStart.addObject("Drive forward", AutonFirstMovement.DRIVE_FORWARD);
		movementOffStart.addObject("Drive to gear station", AutonFirstMovement.GO_TO_LIFT);
		movementOffStart.addObject("Move to shooting range", AutonFirstMovement.GO_TO_SHOOTING_RANGE);
		SmartDashboard.putData("Step 1", movementOffStart);
		
		firstAction = new SendableChooser();
		firstAction.addDefault("Stop", AutonFirstAction.DO_NOTHING);
		firstAction.addObject("Deliver gear", AutonFirstAction.DELIVER_GEAR);
		firstAction.addObject("Shoot", AutonFirstAction.SHOOT);
		SmartDashboard.putData("Step 2", firstAction);
		
		nextMovement = new SendableChooser();
		nextMovement.addDefault("Stop", AutonSecondMovement.STOP);
		nextMovement.addObject("Near hopper", AutonSecondMovement.NEAR_HOPPER);
		nextMovement.addObject("Far hopper", AutonSecondMovement.FAR_HOPPER);
		nextMovement.addObject("Shooting range", AutonSecondMovement.SHOOTING_RANGE);
		SmartDashboard.putData("Step 3", nextMovement);
		
		secondAction = new SendableChooser();
		secondAction.addDefault("Stop", AutonSecondAction.STOP);
		secondAction.addObject("Shoot", AutonSecondAction.SHOOT);
		SmartDashboard.putData("Step 4", secondAction);
		
		shooterType = new SendableChooser();
		shooterType.addDefault("None", ShooterEnum.NO_SHOOTER);
		shooterType.addObject("Left", ShooterEnum.LEFT_SHOOTER);
		shooterType.addObject("Right", ShooterEnum.RIGHT_SHOOTER);
		shooterType.addObject("Dual", ShooterEnum.DUAL_SHOOTER);
		shooterType.addObject("Dual Staggered", ShooterEnum.DUAL_STAGGERED_SHOOTER);
		/*SmartDashboard.putData("Shot Selection : None", ShooterEnum.NO_SHOOTER);
		SmartDashboard.putData("Shot Selection : Left", ShooterEnum.LEFT_SHOOTER);
		SmartDashboard.putData("Shot Selection : Right", ShooterEnum.RIGHT_SHOOTER);
		SmartDashboard.putData("Shot Selection : Dual", ShooterEnum.DUAL_SHOOTER);
		SmartDashboard.putData("Shot Selection : Dual Staggered", ShooterEnum.DUAL_STAGGERED_SHOOTER);*/
		// Update the network tables with a notifier.
		// This will update the table every 50 milliseconds, during every stage of the game. OISlow() updates every 500 milliseconds
		new Notifier(() -> updateOI()).startPeriodic(0.050);
		//new Notifier(() -> updateOISlow()).startPeriodic(0.500);
    }
	
	/**
	 * <pre>
	 * public void updateOI()
	 * </pre>
	 * 
	 * Update block for the OI
	 */
	public void updateOI() {
		SmartDashboard.putNumber("Gyro", Robot.driveTrain.getGyro().getAngle());
		SmartDashboard.putNumber("Left Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.driveTrain.getRightEncoder());
		SmartDashboard.putNumber("NAV-X", Robot.driveTrain.getNAVX().getAngle());
		SmartDashboard.putNumber("Left Enc Feet", Robot.driveTrain.encoderTicksToFeet(Robot.driveTrain.getLeftEncoder()) - SmartDashboard.getNumber("SetLeftChange", 0));
		SmartDashboard.putNumber("Right Enc Feet", Robot.driveTrain.encoderTicksToFeet(Robot.driveTrain.getRightEncoder()) - SmartDashboard.getNumber("SetRightChange", 0));
		SmartDashboard.putNumber("Shooter ID 7: Encoder Speed", Robot.rightShooter.motor.getSpeed());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Speed", Robot.leftShooter.motor.getSpeed());
		SmartDashboard.putNumber("Shooter ID 7: Encoder GET", Robot.rightShooter.motor.get());
		SmartDashboard.putNumber("Shooter ID 8: Encoder GET", Robot.leftShooter.motor.get());
		SmartDashboard.putNumber("Shooter ID 7: Encoder Position", Robot.rightShooter.motor.getPosition());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Position", Robot.leftShooter.motor.getPosition());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Velocity", Robot.leftShooter.motor.getEncVelocity());
		SmartDashboard.putNumber("Shooter ID 7: Encoder Velocity", Robot.rightShooter.motor.getEncVelocity());
		SmartDashboard.putNumber("Shooter ID 7: Voltage", Robot.rightShooter.motor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 8: Voltage", Robot.leftShooter.motor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 7: Current", Robot.rightShooter.motor.getOutputCurrent());
		SmartDashboard.putNumber("Shooter ID 8: Current", Robot.leftShooter.motor.getOutputCurrent());
		SmartDashboard.putNumber("Ultrasonic", Robot.ultrasonic.getRange());
	}
	
	/*public void updateOISlow() {
		//CalculateGearPath calculateGearPath = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);
		
		SmartDashboard.putNumber("Distance to gear lift using vertical values", Utilities.round(Robot.visionProcessor.getDistanceUsingVerticalInformation(TargetType.GEAR_VISION), 3));
		SmartDashboard.putNumber("Distance to gear lift using horizontal values", Utilities.round(Robot.visionProcessor.getDistanceUsingHorizontalInformation(TargetType.GEAR_VISION), 3));
		SmartDashboard.putNumber("Distance to high goal using vertical values", Utilities.round(Robot.visionProcessor.getDistanceUsingVerticalInformation(TargetType.HIGH_GOAL), 3));
		SmartDashboard.putNumber("Distance to high goal using horizontal values", Utilities.round(Robot.visionProcessor.getDistanceUsingHorizontalInformation(TargetType.HIGH_GOAL), 3));
		SmartDashboard.putNumber("Distance to target1", Utilities.round(Robot.visionProcessor.getDistancesToGearTargets()[0], 3));
		SmartDashboard.putNumber("Distance to target2", Utilities.round(Robot.visionProcessor.getDistancesToGearTargets()[1], 3));
		SmartDashboard.putNumber("Angle to gear lift using horizontal values", Utilities.round(Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION), 3));
		SmartDashboard.putNumber("Angle to high goal using horizontal values", Utilities.round(Robot.visionProcessor.getAngleFromCenter(TargetType.HIGH_GOAL), 3));
		SmartDashboard.putNumber("Angle to target1", Utilities.round(Robot.visionProcessor.getAnglesFromGearTargets()[0], 3));
		SmartDashboard.putNumber("Angle to target2", Utilities.round(Robot.visionProcessor.getAnglesFromGearTargets()[1], 3));
		SmartDashboard.putNumber("Distance to gear lift using horizontal and vertical average", Utilities.round(Robot.visionProcessor.getAverageDistanceUsingHorAndVerDistances(TargetType.GEAR_VISION), 3));
		SmartDashboard.putNumber("Distance to high goal using horizontal and vertical average", Utilities.round(Robot.visionProcessor.getAverageDistanceUsingHorAndVerDistances(TargetType.HIGH_GOAL), 3));
		SmartDashboard.putNumber("Distance to gear lift using average of both targets", Utilities.round(Robot.visionProcessor.getAverageDistanceToGearTargets(), 3));
		SmartDashboard.putNumber("Gear Path Distance", calculateGearPath.getDistance());
		SmartDashboard.putNumber("Gear Path Theta", calculateGearPath.getTheta());
	}*/


    public Joystick getLeftStick() {
        return leftStick;
    }

    public Joystick getRightStick() {
        return rightStick;
    }

    public Joystick getGamePad() {
        return gamePad;
    }
    
	/**
	 * <pre>
	 * public boolean applyDeadzone(double val, double dzLimit)
	 * </pre>
	 * Applies a deadzone limit to a joystick axis value
	 *
	 * @param val
	 *            the axis value to check
	 * @param dzLimit
	 *            the deadzone limit to apply
	 * @return
	 *           the original value, if it is outside the deadzone limit,
	 *           0.0 otherwise
	 */
    public double applyDeadzone(double val, double dzLimit) {
    	return (Math.abs(val) > Math.abs(dzLimit)) ? val : 0.0;
    	/*double newVal = (Math.abs(val) - dzLimit * Math.signum(val)) / (1 - dzLimit * Math.signum(val));
    	return (Math.abs(val) > dzLimit) ? newVal : 0.0;*/ 
    }
    
    public AutonPosition getStartPosition() {
    	return (AutonPosition) startPosition.getSelected();
    }
    
    public AutonFirstMovement getFirstMovement() {
    	return (AutonFirstMovement) movementOffStart.getSelected();
    }
    
    public AutonFirstAction getFirstAction() {
    	return (AutonFirstAction) firstAction.getSelected();
    }
    
    public AutonSecondMovement getSecondMovement() {
    	return (AutonSecondMovement) nextMovement.getSelected();
    }
    
    public AutonSecondAction getSecondAction() {
    	return (AutonSecondAction) secondAction.getSelected();
    }
    
    public ShooterEnum getShot() {
    	return (ShooterEnum) shooterType.getSelected();
    }
}
