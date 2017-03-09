package org.usfirst.frc.team1089.robot;

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
	
	SendableChooser startPosition, shooterType, firstAction, secondAction,
		firstMovement, secondMovement;
	
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
    
    //Buttons
    public JoystickButton gamePadBtnA;
    public JoystickButton gamePadBtnB;
    public JoystickButton gamePadBtnY;
    public JoystickButton gamePadBtnX;
    public JoystickButton start;
    public JoystickButton gamePadBtnLB;
    public JoystickButton gamePadBtnRB;
    public JoystickButton gamePadBtnRT;
    
    //Dhruver

    //Right Stick
    public JoystickButton rightStick1;
	public JoystickButton rightStick3;
	
	//Left Stick
    public JoystickButton leftStick1;
    public JoystickButton leftStick2;
    public JoystickButton leftStick3;
    public JoystickButton leftStick7;
    
    
    
    public OI() {
    	
        leftStick = new Joystick(RobotMap.DS_USB.LEFT_STICK);
        rightStick = new Joystick(RobotMap.DS_USB.RIGHT_STICK);
        gamePad = new Joystick(RobotMap.DS_USB.GAMEPAD);
        gamePadBtnA = new JoystickButton(gamePad, RobotMap.GamepadButtons.A);
        gamePadBtnA.whenPressed(new DriveWithJoysticks());
        gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        gamePadBtnB.whenPressed(new DriveDistance(2));
        gamePadBtnY = new JoystickButton(gamePad, RobotMap.GamepadButtons.Y);
        gamePadBtnY.whenPressed(new  ToggleGearDelivery(true));
        
        gamePadBtnX = new JoystickButton(gamePad, RobotMap.GamepadButtons.X);
        gamePadBtnX.whenPressed(new  ToggleGearDelivery(false));
        
        gamePadBtnRB = new JoystickButton(gamePad, RobotMap.GamepadButtons.RB);
        gamePadBtnRB.whenPressed(new DegreeRotate(60));
        
        gamePadBtnLB = new JoystickButton(gamePad, RobotMap.GamepadButtons.LB);
        gamePadBtnLB.whenPressed(new BasicGearDelivery());
        
        start = new JoystickButton(gamePad, RobotMap.GamepadButtons.START);
        start.whenPressed(new CalibrateGyro());
        
        rightStick1 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN1);
        rightStick1.whenPressed(new ShootWithDistance(Robot.rightShooter));

        leftStick1 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN1);
        leftStick1.whenPressed(new ShootWithDistance(Robot.leftShooter));        
        
        rightStick3 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN3);
        rightStick3.whenPressed(new AutoAlign(TargetType.GEAR_VISION));
        
        leftStick2 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN2);
        leftStick2.whenPressed(new DeliverGear());
        
        leftStick3 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN3);
        leftStick3.whenPressed(new CalculateGearPath(Direction.REVERSE));
        
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
		
		firstMovement = new SendableChooser();
		firstMovement.addDefault("Do nothing", AutonFirstMovement.DO_NOTHING);
		firstMovement.addObject("Drive forward", AutonFirstMovement.DRIVE_FORWARD);
		firstMovement.addObject("Drive to gear station", AutonFirstMovement.GO_TO_LIFT);
		firstMovement.addObject("Move to shooting range", AutonFirstMovement.GO_TO_SHOOTING_RANGE);
		SmartDashboard.putData("Step 1", firstMovement);
		
		firstAction = new SendableChooser();
		firstAction.addDefault("Stop", AutonFirstAction.DO_NOTHING);
		firstAction.addObject("Deliver gear", AutonFirstAction.DELIVER_GEAR);
		firstAction.addObject("Shoot", AutonFirstAction.SHOOT);
		SmartDashboard.putData("Step 2", firstAction);
		
		secondMovement = new SendableChooser();
		secondMovement.addDefault("Stop", AutonSecondMovement.STOP);
		secondMovement.addObject("Near hopper", AutonSecondMovement.NEAR_HOPPER);
		secondMovement.addObject("Far hopper", AutonSecondMovement.FAR_HOPPER);
		secondMovement.addObject("Shooting range", AutonSecondMovement.SHOOTING_RANGE);
		SmartDashboard.putData("Step 3", secondMovement);
		
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
		SmartDashboard.putNumber("Shooter ID 7: Encoder RPM", Robot.rightShooter.getSpeed());
		SmartDashboard.putNumber("Shooter ID 8: Encoder RPM", Robot.leftShooter.getSpeed());
		SmartDashboard.putNumber("Shooter ID 7: Encoder Native Units", Robot.rightShooter.shooterMotor.get());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Native Units", Robot.leftShooter.shooterMotor.get());
		SmartDashboard.putNumber("Shooter ID 7: Encoder Position", Robot.rightShooter.shooterMotor.getPosition());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Position", Robot.leftShooter.shooterMotor.getPosition());
		SmartDashboard.putNumber("Shooter ID 7: Voltage", Robot.rightShooter.shooterMotor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 8: Voltage", Robot.leftShooter.shooterMotor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 7: Current", Robot.rightShooter.shooterMotor.getOutputCurrent());
		SmartDashboard.putNumber("Shooter ID 8: Current", Robot.leftShooter.shooterMotor.getOutputCurrent());
		SmartDashboard.putNumber("Ultrasonic", Robot.ultrasonic.getRange());
		SmartDashboard.putNumber("Angle Gear", Robot.visionProcessor.angleGear);
	}
/*	
	public void updateOISlow() {
		CalculateGearPath calculateGearPath = new CalculateGearPath(CalculateGearPath.Direction.REVERSE);
		
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
    	return (AutonFirstMovement) firstMovement.getSelected();
    }
    
    public AutonFirstAction getFirstAction() {
    	return (AutonFirstAction) firstAction.getSelected();
    }
    
    public AutonSecondMovement getSecondMovement() {
    	return (AutonSecondMovement) secondMovement.getSelected();
    }
    
    public AutonSecondAction getSecondAction() {
    	return (AutonSecondAction) secondAction.getSelected();
    }
    
    public ShooterEnum getShot() {
    	return (ShooterEnum) shooterType.getSelected();
    }
}
