package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.DriveWithJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This {@link Subsystem} encapsulates everything we need to drive:
 * motor controllers, gyros, and the {@link RobotDrive} class we need to interface
 * with the system.
 * 
 * @see PIDOutput
 * @see AnalogGyro
 * @see CANTalon
 */
public class DriveTrain extends Subsystem implements PIDOutput{

	//Q: How are computers like men?
	//Scroll down for the answer!
		
    private AnalogGyro gyro;
    private CANTalon leftBack;
    private CANTalon rightBack;
    private CANTalon leftFront;
    private CANTalon rightFront;
    private RobotDrive robotDrive;
    /*leftFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	rightFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);leftFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	rightFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);*/
	private AHRS navx;
	
	private static final int MAG_ENCODER_TICKS_PER_REVOLUTION = 4096;
    
    //private Encoder rightEnc, leftEnc;
    
    private static final double GEAR_RATIO = 1, WHEEL_DIAMETER = 4.0 / 12.0;
    
	public DriveTrain() {
		
/*		super("DriveTrain", 0.05, 0.0, 0.0);
		setAbsoluteTolerance(0.1);
		getPIDController().setContinuous(true);
*/
		gyro = new AnalogGyro(RobotMap.Analog.GYRO);
        gyro.setSensitivity((1.1*5/3.38)/1000); // TODO Move this to Config
        
        navx = new AHRS(SerialPort.Port.kUSB1);

        leftBack = new CANTalon(RobotMap.CAN.LEFT_BACK_TALON_ID);
        leftFront = new CANTalon(RobotMap.CAN.LEFT_FRONT_TALON_ID);
        rightBack = new CANTalon(RobotMap.CAN.RIGHT_BACK_TALON_ID);
        rightFront = new CANTalon(RobotMap.CAN.RIGHT_FRONT_TALON_ID);

        // Enable brake mode on all Talons
		leftFront.enableBrakeMode(true);
		rightFront.enableBrakeMode(true);
		leftBack.enableBrakeMode(true);
		rightBack.enableBrakeMode(true);
		leftFront.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		rightFront.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		leftFront.reverseOutput(true);
		rightFront.reverseOutput(true);

		// Configure back talons as followers.
		leftBack.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightBack.changeControlMode(CANTalon.TalonControlMode.Follower);
		
		leftBack.set(leftFront.getDeviceID());
		rightBack.set(rightFront.getDeviceID());

		//robotDrive = new RobotDrive(0,1);
        robotDrive = new RobotDrive(leftFront, rightFront);
        robotDrive.setSafetyEnabled(true); // TODO: Make sure to change this back to true
        robotDrive.setExpiration(0.1);
        robotDrive.setSensitivity(0.5);
        robotDrive.setMaxOutput(1.0);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);

        LiveWindow.addSensor("DriveTrain", "Gyro", gyro);
        LiveWindow.addSensor("DriveTrain", "Nav-X", navx);
        LiveWindow.addActuator("DriveTrain", "LeftFront", leftFront);
        LiveWindow.addActuator("DriveTrain", "RightFront", rightFront);
	}

	//A: You need to turn them on to get their attention!
	
	/**
     * <pre>
	 * public void enableRobotDrive()
	 * </pre>
	 * Enables safety mode.
     */
	public void enableRobotDrive() {
		robotDrive.setSafetyEnabled(true);
	}
	
	/**
     * <pre>
	 * public void disableRobotDrive()
	 * </pre>
	 * Disables safety mode.
     */
	public void disableRobotDrive() {
		robotDrive.setSafetyEnabled(false);
	}
	
	/**
     * <pre>
	 * public void isSafetyEnabled()
	 * </pre>
	 * @return If SafetyMode is enabled or disabled (see enableRobotDrive() and disableRobotDrive())
     */
	public boolean isSafetyEnabled() {
		return robotDrive.isSafetyEnabled();
	}
	
	/**
     * <pre>
	 * public void initDefaultCommand()
	 * </pre>
	 * The default command for DriveTrain.
     */
    public void initDefaultCommand() {
        // By default, drive with the joysticks
        setDefaultCommand(new DriveWithJoysticks());
    }
    

    /**
     * <pre>
	 * public void joystickDrive(Joystick leftStick, Joystick rightStick)
	 * </pre>
	 * Control the drive train with two joysticks. Currently this uses
	 * two-stick arcade drive with deadzones.
     * @param leftStick  The left joystick
     * @param rightStick The right joystick
     */
    public void joystickDrive(Joystick leftStick, Joystick rightStick) {
    	// Apply the joystick deadzones to the move and rotate values
    	double moveValue = Robot.oi.applyDeadzone(rightStick.getX(), Robot.oi.JS_DEADZONE_LIMIT);
    	double rotateValue = Robot.oi.applyDeadzone(leftStick.getY(), Robot.oi.JS_DEADZONE_LIMIT);
    	arcadeDrive(moveValue, rotateValue);
    }
    
    /**
     * <pre>
	 * public void arcadeDrive(double moveValue, double rotateValue)
	 * </pre>
	 * Without joystick control, just the raw movement values for arcade drive.
     * @param moveValue  Voltage to move forward, in percent
     * @param rotateValue To turn in a certain direction while moving forward
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
    	robotDrive.arcadeDrive(moveValue, rotateValue);
    }
    
    /**
     * <pre>
	 * public void stop()
	 * </pre>
	 * Stops the drive train.
     */
    public void stop() {
    	robotDrive.stopMotor();
    }

    /**
     * <pre>
	 * public AnalogGyro getGyro()
	 * </pre>
	 * Returns the gyro that is currently being used. Beware of null pointers!!!
	 * @return The Gyro
     */
    public AnalogGyro getGyro() { 	
    	return gyro;
    }
    
    /**
     * <pre>
	 * public AHRS getNAVX()
	 * </pre>
	 * Returns the NavX that is currently being used. Beware of null pointers!!!
	 * @return The NavX
     */
    public AHRS getNAVX() {
    	return navx;
    }

    /**
     * <pre>
	 * public double getLeftEncoder()
	 * </pre>
	 * Returns the current value in ticks of the encoder 
	 * on the left drive shaft.
	 * @return The value of the left encoder
     */
    public double getLeftEncoder() {
    	return leftFront.getEncPosition();
    }
    
    /**
     * <pre>
	 * public double getRightEncoder()
	 * </pre>
	 * Returns the current value in ticks of the encoder 
	 * on the right drive shaft.
	 * @return The value of the right encoder
     */
    public double getRightEncoder() {
    	return -rightFront.getEncPosition();
    }
    
    /**
     * <pre>
	 * public void resetEncoders()
	 * </pre>
	 * Resets the value of both encoders to 0.
     */
    public void resetEncoders() {
    	leftFront.setEncPosition(0);
    	rightFront.setEncPosition(0);
    }
    
    public void resetMotionProfiling() {
    	leftFront.clearMotionProfileHasUnderrun();
    	leftFront.clearMotionProfileTrajectories();
    	rightFront.clearMotionProfileHasUnderrun();
    	rightFront.clearMotionProfileTrajectories();
    }
    
	@Override
	public void pidWrite(double output) {
		robotDrive.tankDrive(output, output);
	}
	
	public void writePID(double output) {
		robotDrive.tankDrive(output, -output);
	}
	
	public double getAutonRotatePidValue(double output) {
		return output;
	}
	
	public double getAutonDriveDistancePidValue(double output) {
		return output;
	}
	
	/**
     * <pre>
	 * public double encoderTicksToFeet(double ticks)
	 * </pre>
	 * Returns a value in feet based on a certain value in ticks using
	 * the Magnetic Encoder.
	 * @param ticks The value in ticks
	 * @return The value in feet
     */
	public double encoderTicksToFeet(double ticks) {
		return (Math.PI * WHEEL_DIAMETER) / (MAG_ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO) * ticks;
	}
	
	/**
     * <pre>
	 * public double feetToEncoderTicks(double feet)
	 * </pre>
	 * Returns a value in ticks based on a certain value in feet using
	 * the Magnetic Encoder.
	 * @param feet The value in feet
	 * @return The value in ticks
     */
	public double feetToEncoderTicks(double feet) {
		return (MAG_ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER) * feet;
	}
	
	/**
     * <pre>
	 * public double revolutionsToFeet(double revolutions)
	 * </pre>
	 * Returns a value in feet based on a certain value in revolutions using
	 * the Magnetic Encoder, because the when set to position, the Magnetic Encoder
	 * takes a value in revolutions rather than ticks.
	 * @param revolutions The number of revolutions needed to be converted.
	 * @return The value in feet
     */
	public double revolutionsToFeet(double revolutions) {
		return revolutions * (Math.PI * WHEEL_DIAMETER);
	}
	
	/**
     * <pre>
	 * public double feetToRevolutions(double feet)
	 * </pre>
	 * Returns a value in revolutions based on a certain value in feet using
	 * the Magnetic Encoder, because the when set to position, the Magnetic Encoder
	 * takes a value in revolutions rather than ticks.
	 * @param feet The distance in feet needed to be converted.
	 * @return The value in revolutions
     */
	public double feetToRevolutions(double feet) {
		return feet / (Math.PI * WHEEL_DIAMETER);
	}
	
	/**
     * <pre>
	 * public void setToVbus()
	 * </pre>
	 * Sets the talons to a mode which allows it to take a value between -1 and 1, a percent,
	 * and sets it to a percent of the maximum voltage of the motor
     */
	public void setToVbus() {
		leftFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	rightFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	/**
     * <pre>
	 * public void setToVbus()
	 * </pre>
	 * Sets the talons to a mode which allows it to take an encoder value, and will set the encoder position to that value.
     */
	public void setToPosition() {
		leftFront.changeControlMode(CANTalon.TalonControlMode.Position);
    	rightFront.changeControlMode(CANTalon.TalonControlMode.Position);
	}
	
	/**
     * <pre>
	 * public CANTalon getRight()
	 * </pre>
	 * @return The right lead CANTalon
     */
	public CANTalon getRight() {
		return rightFront;
	}
	
	/**
     * <pre>
	 * public CANTalon getLeft()
	 * </pre>
	 * @return The left lead CANTalon
     */
	public CANTalon getLeft() {
		return leftFront;
	}
	
	/**
     * <pre>
	 * public double getRightStickVal()
	 * </pre>
	 * @return The X-value of the right stick, which controls turn sharpness in arcade drive.
     */
	public double getRightStickVal() {
		return Robot.oi.rightStick.getX();
	}
	
	/**
     * <pre>
	 * public double getLeftStickVal()
	 * </pre>
	 * @return The Y-value of the right stick, which controls throttle in arcade drive.
     */
	public double getLeftStickVal() {
		return Robot.oi.leftStick.getY();
	}
}