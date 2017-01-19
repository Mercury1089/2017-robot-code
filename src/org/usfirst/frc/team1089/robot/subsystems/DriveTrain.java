package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.OI;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.DriveWithJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DriveTrain extends Subsystem {

	private OI oi;

    private AnalogGyro gyro;
    private CANTalon leftBack;
    private CANTalon rightBack;
    private CANTalon leftFront;
    private CANTalon rightFront;
    private RobotDrive robotDrive;
    
	public DriveTrain() {
		oi = Robot.oi;

        gyro = new AnalogGyro(RobotMap.Analog.GYRO);
        gyro.setSensitivity(0.007); // TODO Move this to Config

        leftBack = new CANTalon(RobotMap.CAN.LEFT_BACK_TALON_ID);
        leftFront = new CANTalon(RobotMap.CAN.LEFT_FRONT_TALON_ID);
        rightBack = new CANTalon(RobotMap.CAN.RIGHT_BACK_TALON_ID);
        rightFront = new CANTalon(RobotMap.CAN.RIGHT_FRONT_TALON_ID);

        // Enable brake mode on all Talons
		leftFront.enableBrakeMode(true);
		rightFront.enableBrakeMode(true);
		leftBack.enableBrakeMode(true);
		rightBack.enableBrakeMode(true);
		leftFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);

		// Configure back talons as followers.
		leftBack.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightBack.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftBack.set(leftFront.getDeviceID());
		rightBack.set(rightFront.getDeviceID());

        
        robotDrive = new RobotDrive(leftFront, rightFront);
        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setSensitivity(0.5);
        robotDrive.setMaxOutput(1.0);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);

        LiveWindow.addSensor("DriveTrain", "Gyro", gyro);
        LiveWindow.addActuator("DriveTrain", "LeftFront", leftFront);
        LiveWindow.addActuator("DriveTrain", "RightFront", rightFront);

	}

    public void initDefaultCommand() {
        // By default, drive with the joysticks
        setDefaultCommand(new DriveWithJoysticks());
    }
    

    /**
     * <pre>
	 * public void joystickDrive(Joystick leftStick, Joystick rightStick)
	 * </pre>
	 * Control the drive train with two joystics. Currently this uses
	 * two-stick arcade drive with deadzones.
     * @param leftStick  The left joystick
     * @param rightStick The right joystick
     */
    public void joystickDrive(Joystick leftStick, Joystick rightStick) {
    	// Apply the joystick deadzones to the move and rotate values
    	double moveValue = oi.applyDeadzone(leftStick.getX(), oi.JS_DEADZONE_LIMIT);
    	double rotateValue = oi.applyDeadzone(rightStick.getY(), oi.JS_DEADZONE_LIMIT);
    	robotDrive.arcadeDrive(moveValue, rotateValue);
    }
    
    public void stop() {
    	robotDrive.stopMotor();
    }
}

