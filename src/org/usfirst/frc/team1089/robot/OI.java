package org.usfirst.frc.team1089.robot;

import org.usfirst.frc.team1089.robot.auton.AutonEnum;
import org.usfirst.frc.team1089.robot.commands.*;
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
	
	SendableChooser startPosition, step3Chooser, shooterType;
	
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
    
    public OI() {
    	
        leftStick = new Joystick(RobotMap.DS_USB.LEFT_STICK);
        rightStick = new Joystick(RobotMap.DS_USB.RIGHT_STICK);
        gamePad = new Joystick(RobotMap.DS_USB.GAMEPAD);
        gamePadBtnA = new JoystickButton(gamePad, RobotMap.GamepadButtons.A);
        gamePadBtnA.whenPressed(new DriveWithJoysticks());
        //gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        //gamePadBtnB.whenPressed(Robot.driveTrain.);
        gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        gamePadBtnB.whenPressed(new DriveDistance(24));
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
		startPosition.addDefault("Left Corner: 1", 1);
		startPosition.addObject("Left, Left Line: 2", 2);
		startPosition.addObject("Right, Left Line: 3", 3);
		startPosition.addObject("Left, Midline: 4", 4);
		startPosition.addObject("Mid, Midline: 5", 5);
		startPosition.addObject("Right, Midline: 6", 6);
		startPosition.addObject("Left, Right Line: 7", 7);
		startPosition.addObject("Right, Right Line: 8", 8);
		startPosition.addObject("Right Corner: 9", 9);
		SmartDashboard.putData("Starting Pos", startPosition);
		
		step3Chooser = new SendableChooser();
		step3Chooser.addDefault("STOP", AutonEnum.STOP);
		step3Chooser.addObject("Turn and Shoot", AutonEnum.TURN_SHOOT);
		step3Chooser.addObject("Far Hopper", AutonEnum.FAR_HOPPER);
		step3Chooser.addObject("Near Hopper", AutonEnum.NEAR_HOPPER);
		SmartDashboard.putData("Step 3 (After delivering gear)", step3Chooser);
		
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
		new Notifier(() -> updateOISlow()).startPeriodic(0.500);
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
		SmartDashboard.putNumber("Left Enc Inches", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()) - SmartDashboard.getNumber("SetLeftChange", 0));
		SmartDashboard.putNumber("Right Enc Inches", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) - SmartDashboard.getNumber("SetRightChange", 0));
		SmartDashboard.putNumber("Shooter ID 7: Encoder Value", Robot.rightShooter.motor.getSpeed());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Value", Robot.leftShooter.motor.getSpeed());
		SmartDashboard.putNumber("Shooter ID 8: Encoder Velocity", Robot.leftShooter.motor.getEncVelocity());
		SmartDashboard.putNumber("Shooter ID 7: Encoder Velocity", Robot.rightShooter.motor.getEncVelocity());
		SmartDashboard.putNumber("Shooter ID 7: Voltage", Robot.rightShooter.motor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 8: Voltage", Robot.leftShooter.motor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 7: Current", Robot.rightShooter.motor.getOutputCurrent());
		SmartDashboard.putNumber("Shooter ID 8: Current", Robot.leftShooter.motor.getOutputCurrent());
		//SmartDashboard.putNumber("Encoder Value", Robot.shooter.motor.getSpeed());
		//SmartDashboard.putString("Mag Enc MODE", " " + Robot.shooter.motor.getControlMode());
		SmartDashboard.putNumber("Ultrasonic", Robot.ultrasonic.getRange());
	}
	
	public void updateOISlow() {
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
	}


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
    
    public AutonEnum getStep3() {
    	return (AutonEnum) step3Chooser.getSelected();
    }
    
    public int getStartPos() {
    	return (int) startPosition.getSelected();
    }
    
    public ShooterEnum getShot() {
    	return (ShooterEnum) shooterType.getSelected();
    }
}
