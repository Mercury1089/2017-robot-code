package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.DriveWithJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DriveTrain extends Subsystem implements PIDOutput{

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
    
    //private Encoder rightEnc, leftEnc;
    
    private final double GEAR_RATIO, WHEEL_DIAMETER;
    
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
		leftFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);

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
        
        GEAR_RATIO = 1;
        WHEEL_DIAMETER = 4;
        /*getPIDController().disable();*/
	}
	
	
	/**
	 * <pre>
	 * public void enableRobotDrive()
	 * </pre>
	 * Enables the safety on the robot drive
	 */
	public void enableRobotDrive() {
		robotDrive.setSafetyEnabled(true);
	}
	
	/**
	 * <pre>
	 * public void disableRobotDrive()
	 * </pre>
	 * Disables the safety on the robot drive
	 */
	public void disableRobotDrive() {
		robotDrive.setSafetyEnabled(false);
	}

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
    	robotDrive.arcadeDrive(moveValue, rotateValue);
    }
    
    /**
     * <pre>
     * public void stop()
     * </pre>
     * Emergency stops the robot drive motors
     */
    public void stop() {
    	robotDrive.stopMotor();
    }

    public AnalogGyro getGyro() { 	
    	return gyro;
    }
    
    public AHRS getNAVX() {
    	return navx;
    }

    /**
     * <pre>
     * public double getLeftEncoder()
     * </pre>
     * Gets the value of the left encoder
     * @return the position of the left front encoder
     */
    public double getLeftEncoder() {
    	return leftFront.getEncPosition();
    }
    
    /**
     * <pre>
     * public double getRightEncoder()
     * </pre>
     * Gets the value of the right encoder
     * @return the negative position of the right front encoder
     */
    public double getRightEncoder() {
    	return -rightFront.getEncPosition();
    }
    
    /**
     * <pre>
     * public void resetEncoders()
     * </pre>
     * Resets both sides' encoders' position to 0
     */
    public void resetEncoders() {
    	leftFront.setEncPosition(0);
    	rightFront.setEncPosition(0);
    }
    
    
    /*public Encoder rightEncoder() {
    	return rightFront;
    }
    
    public Encoder leftEncoder() {
    	return leftFront;
    }*/
    
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		robotDrive.tankDrive(output, output);
	}
	
	/**
	 * <pre>
	 * public double encoderTicksToInches(double ticks)
	 * </pre>
	 * Converts the specified amount of ticks to inches, 
	 * based on our specifications of the wheels and gear ratios
	 * @param ticks the amount of ticks accumulated by an encoder
	 * @return the amount of inches accumulated from the encoder
	 */
	public double encoderTicksToInches(double ticks) {
		return (Math.PI * WHEEL_DIAMETER) / (1440 * GEAR_RATIO) * ticks;
	}
	
	/**
	 * <pre>
	 * public double inchesToEncoderTicks(double inches)
	 * </pre>
	 * Converts the specified amount of inches to ticks, 
	 * based on our specifications of the wheels and gear ratios
	 * @param inches the amount of inches accumulated by an encoder
	 * @return the amount of ticks accumulated from the encoder
	 */
	public double inchesToEncoderTicks(double inches) {
		return (1440 * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER) * inches;
	}
	
	/**
	 * <pre>
	 * public void setToVbus()
	 * </pre>
	 * Sets both master {@link CANTalon}s' control modes' to 
	 * {@link CANTalon.TalonControlMode.PercentVbus}
	 */
	public void setToVbus() {
		leftFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	rightFront.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	/**
	 * <pre>
	 * public void setToPosition()
	 * </pre>
	 * Sets both master {@link CANTalon}s' control modes' to 
	 * {@link CANTalon.TalonControlMode.Position}
	 */
	public void setToPosition() {
		leftFront.changeControlMode(CANTalon.TalonControlMode.Position);
    	rightFront.changeControlMode(CANTalon.TalonControlMode.Position);
	}
	
	public CANTalon getRight() {
		return rightFront;
	}
	public CANTalon getLeft() {
		return leftFront;
	}
	
}

