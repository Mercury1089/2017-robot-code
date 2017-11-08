package org.usfirst.frc.team1089.robot;

import org.usfirst.frc.team1089.robot.auton.AutonFirstAction;
import org.usfirst.frc.team1089.robot.auton.AutonFirstMovement;
import org.usfirst.frc.team1089.robot.auton.AutonPosition;
import org.usfirst.frc.team1089.robot.auton.AutonSecondAction;
import org.usfirst.frc.team1089.robot.auton.AutonSecondMovement;
import org.usfirst.frc.team1089.robot.commands.AutoAlign;
import org.usfirst.frc.team1089.robot.commands.AutoShoot;
import org.usfirst.frc.team1089.robot.commands.BasicGearDelivery;
import org.usfirst.frc.team1089.robot.commands.CalibrateGyro;
import org.usfirst.frc.team1089.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1089.robot.commands.ManualShoot;
import org.usfirst.frc.team1089.robot.commands.OpenLatch;
import org.usfirst.frc.team1089.robot.commands.ReverseIntake;
import org.usfirst.frc.team1089.robot.commands.RunAllShooters;
import org.usfirst.frc.team1089.robot.commands.RunClimber;
import org.usfirst.frc.team1089.robot.commands.RunIntake;
import org.usfirst.frc.team1089.robot.commands.StopAllShooters;
import org.usfirst.frc.team1089.robot.commands.ToggleGearDelivery;
import org.usfirst.frc.team1089.robot.subsystems.Shooter.ShooterEnum;
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

	public static final double LEFT_JS_DEADZONE_LIMIT = 0.4; // Deadzone limit for the left stick	
	public static final double RIGHT_JS_DEADZONE_LIMIT = 0.4; // Deadzone limit for the right stick	
	
	
	SendableChooser<AutonPosition> startPosition; 
	SendableChooser<ShooterEnum> shooterType; 
	SendableChooser<AutonFirstAction> firstAction;
	SendableChooser<AutonSecondAction> secondAction;
	SendableChooser<AutonFirstMovement> firstMovement; 
	SendableChooser<AutonSecondMovement> secondMovement;
	
	
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
    public JoystickButton gamePadBtnR3;
    
    //Dhruver

    //Right Stick
    public JoystickButton rightStick1;
	public JoystickButton rightStick3;
	public JoystickButton rightStick6;
	public JoystickButton rightStick7;
	
	//Left Stick
    public JoystickButton leftStick1;
    public JoystickButton leftStick2;
    public JoystickButton leftStick3;
    public JoystickButton leftStick6;
    public JoystickButton leftStick7;
    public JoystickButton leftStick11;
    
    
    public OI() {
    	
        leftStick = new Joystick(RobotMap.DS_USB.LEFT_STICK);
        rightStick = new Joystick(RobotMap.DS_USB.RIGHT_STICK);
        gamePad = new Joystick(RobotMap.DS_USB.GAMEPAD);
        
        //Drive W/ Joysticks
        gamePadBtnA = new JoystickButton(gamePad, RobotMap.GamepadButtons.A);
        gamePadBtnA.whenPressed(new DriveWithJoysticks());
        
        //StopFeeder	(only needed if feeder/shooter combo is not working properly)
        //gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        //gamePadBtnB.whenPressed(new AutoAlign(TargetType.HIGH_GOAL));
        
        //Run Feeder	(only needed if feeder/shooter combo is not working properly)
        gamePadBtnX = new JoystickButton(gamePad, RobotMap.GamepadButtons.X);
        gamePadBtnX.whenPressed(new ToggleGearDelivery(true));
        
        gamePadBtnY = new JoystickButton(gamePad, RobotMap.GamepadButtons.Y);
        gamePadBtnY.whenPressed(new ToggleGearDelivery(false));
        
        gamePadBtnRB = new JoystickButton(gamePad, RobotMap.GamepadButtons.RB);
        gamePadBtnRB.whenPressed(new RunAllShooters());
        
        gamePadBtnLB = new JoystickButton(gamePad, RobotMap.GamepadButtons.LB);
        gamePadBtnLB.whenPressed(new StopAllShooters());
        
        gamePadBtnR3 = new JoystickButton(gamePad, RobotMap.GamepadButtons.R3);
        gamePadBtnR3.whenPressed(new ReverseIntake(true));
        
        start = new JoystickButton(gamePad, RobotMap.GamepadButtons.START);
        start.whenPressed(new CalibrateGyro());
        
        rightStick1 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN1);
        rightStick1.whenPressed(new AutoShoot());

        leftStick1 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN1);
        leftStick1.whenPressed(new BasicGearDelivery());        
        
        leftStick2 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN2);
        leftStick2.whenPressed(new RunIntake(true));
        
        leftStick3 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN3);
        leftStick3.whenPressed(new RunIntake(false));
        
        leftStick6 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN6);
        leftStick6.whenPressed(new RunClimber(true));
        
        leftStick7 = new JoystickButton(leftStick, RobotMap.JoystickButtons.BTN7);
        leftStick7.whenPressed(new RunClimber(false));
        
      /*rightStick3 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN3);
        rightStick3.whenPressed(new AutoAlign(TargetType.HIGH_GOAL));*/
        
        rightStick6 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN6);
        rightStick6.whenPressed(new OpenLatch());
        
        rightStick7 = new JoystickButton(rightStick, RobotMap.JoystickButtons.BTN7);
        rightStick7.whenPressed(new ManualShoot());
        
        
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
        
        
        selectAuton();

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
		SmartDashboard.putNumber("Shooter ID 7: Encoder V", Robot.rightShooter.shooterMotor.getEncVelocity());
		SmartDashboard.putNumber("Shooter ID 8: Encoder V", Robot.leftShooter.shooterMotor.getEncVelocity());
		SmartDashboard.putNumber("Shooter ID 7: Voltage", Robot.rightShooter.shooterMotor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 8: Voltage", Robot.leftShooter.shooterMotor.getOutputVoltage());
		SmartDashboard.putNumber("Shooter ID 7: Current", Robot.rightShooter.shooterMotor.getOutputCurrent());
		SmartDashboard.putNumber("Shooter ID 8: Current", Robot.leftShooter.shooterMotor.getOutputCurrent());
		SmartDashboard.putNumber("Shooter ID 7: Current", Robot.rightShooter.shooterMotor.getOutputCurrent());
		SmartDashboard.putNumber("Ultrasonic", Robot.ultrasonic.getRange());
		SmartDashboard.putNumber("Angle Gear", Robot.visionProcessor.angleGear);
		SmartDashboard.putNumber("Gear Distance", Robot.visionProcessor.distGear);
		SmartDashboard.putNumber("High Distance", Robot.visionProcessor.distHigh);
		
		/*SmartDashboard.putData("Starting Position", startPosition);
		SmartDashboard.putData("Step 1", firstMovement);
		SmartDashboard.putData("Step 2", firstAction);
		SmartDashboard.putData("Step 3", secondMovement);
		SmartDashboard.putData("Step 4", secondAction);
		*/
		
		//selectAuton();
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
    
    public void selectAuton() {
    	System.out.println("Adding sendables boi");
    	
    	startPosition = new SendableChooser<AutonPosition>();
		//startPosition.addDefault("Left Corner: 1", AutonPosition.POSITION_1);
		//startPosition.addObject("Left, Left Line: 2", AutonPosition.POSITION_2);
		startPosition.addObject("Left Line: 1", AutonPosition.LEFT);
		//startPosition.addObject("Left, Midline: 4", AutonPosition.POSITION_4);
		startPosition.addObject("Mid, Midline: 2", AutonPosition.MIDDLE);
		//startPosition.addObject("Right, Midline: 6", AutonPosition.POSITION_6);
		startPosition.addObject("Right Line: 3", AutonPosition.RIGHT);
		//startPosition.addObject("Right, Right Line: 8", AutonPosition.POSITION_8);
		//startPosition.addObject("Right Corner: 9", AutonPosition.POSITION_9);
		SmartDashboard.putData("S  P", startPosition);
		
		firstMovement = new SendableChooser<AutonFirstMovement>();
		firstMovement.addDefault("Do nothing", AutonFirstMovement.DO_NOTHING);
		firstMovement.addObject("Drive forward", AutonFirstMovement.DRIVE_FORWARD);
		firstMovement.addObject("Drive to gear station", AutonFirstMovement.GO_TO_LIFT);
		firstMovement.addObject("Move to shooting range", AutonFirstMovement.GO_TO_SHOOTING_RANGE);
		SmartDashboard.putData("S1", firstMovement);
		
		firstAction = new SendableChooser<AutonFirstAction>();
		firstAction.addDefault("Stop", AutonFirstAction.DO_NOTHING);
		firstAction.addObject("Deliver gear", AutonFirstAction.DELIVER_GEAR);
		firstAction.addObject("Shoot", AutonFirstAction.SHOOT);
		SmartDashboard.putData("S2", firstAction);
		
		secondMovement = new SendableChooser<AutonSecondMovement>();
		secondMovement.addDefault("Stop", AutonSecondMovement.STOP);
		secondMovement.addObject("Near hopper", AutonSecondMovement.NEAR_HOPPER);
		secondMovement.addObject("Far hopper", AutonSecondMovement.FAR_HOPPER);
		secondMovement.addObject("Shooting range", AutonSecondMovement.SHOOTING_RANGE);
		SmartDashboard.putData("S3", secondMovement);
		
		secondAction = new SendableChooser<AutonSecondAction>();
		secondAction.addDefault("Stop", AutonSecondAction.STOP);
		secondAction.addObject("Shoot", AutonSecondAction.SHOOT);
		SmartDashboard.putData("S4", secondAction);
		
		/*shooterType = new SendableChooser<ShooterEnum>();
		shooterType.addDefault("None", ShooterEnum.NO_SHOOTER);
		shooterType.addObject("Left", ShooterEnum.LEFT_SHOOTER);
		shooterType.addObject("Right", ShooterEnum.RIGHT_SHOOTER);
		shooterType.addObject("Dual", ShooterEnum.DUAL_SHOOTER);
		shooterType.addObject("Dual Staggered", ShooterEnum.DUAL_STAGGERED_SHOOTER);
		*/
		System.out.println("Boio");
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
    	return startPosition.getSelected();
    }
    
    public AutonFirstMovement getFirstMovement() {
    	return firstMovement.getSelected();
    }
    
    public AutonFirstAction getFirstAction() {
    	return firstAction.getSelected();
    }
    
    public AutonSecondMovement getSecondMovement() {
    	return secondMovement.getSelected();
    }
    
    
    public AutonSecondAction getSecondAction() {
    	return secondAction.getSelected();
    }
    
    public ShooterEnum getShot() {
    	return shooterType.getSelected();
    }
}
