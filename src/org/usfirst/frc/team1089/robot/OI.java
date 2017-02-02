package org.usfirst.frc.team1089.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1089.robot.commands.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public static final double JS_DEADZONE_LIMIT = 0.3; // Deadzone limit for the sticks

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
	public OI() {
    	
        leftStick = new Joystick(RobotMap.DS_USB.LEFT_STICK);
        rightStick = new Joystick(RobotMap.DS_USB.RIGHT_STICK);
        gamePad = new Joystick(RobotMap.DS_USB.GAMEPAD);
        gamePadBtnA = new JoystickButton(gamePad, RobotMap.GamepadButtons.A);
        gamePadBtnA.whenPressed(new DriveDistance(105));
        //gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        //gamePadBtnB.whenPressed(Robot.driveTrain.);
        gamePadBtnB = new JoystickButton(gamePad, RobotMap.GamepadButtons.B);
        gamePadBtnB.whenPressed(new DegreeRotate(45));
        gamePadBtnY = new JoystickButton(gamePad, RobotMap.GamepadButtons.Y);
        //gamePadBtnY.whenPressed(new TestCommandGroup());
        
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
}
