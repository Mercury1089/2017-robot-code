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

        
        robotDrive = new RobotDrive(leftFront, rightFront);
        robotDrive.setSafetyEnabled(true);
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
    	double moveValue = Robot.oi.applyDeadzone(rightStick.getX(), Robot.oi.JS_DEADZONE_LIMIT);
    	double rotateValue = Robot.oi.applyDeadzone(leftStick.getY(), Robot.oi.JS_DEADZONE_LIMIT);
    	robotDrive.arcadeDrive(moveValue, rotateValue);
    }
    
    public void stop() {
    	robotDrive.stopMotor();
    }

    public AnalogGyro getGyro() { 	
    	return gyro;
    }
    
    public AHRS getNAVX() {
    	return navx;
    }

    public double getLeftEncoder() {
    	return leftFront.getEncPosition();
    }
    
    public double getRightEncoder() {
    	return -rightFront.getEncPosition();
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
	
	public double encoderTicksToInches(double ticks) {
		return (Math.PI * WHEEL_DIAMETER) / (1440 * GEAR_RATIO) * ticks;
	}
	
	public void setRight(double v) {
		rightFront.set(v);
	}
	public void setLeft(double v) {
		leftFront.set(v);
	}
}

