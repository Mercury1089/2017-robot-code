package org.usfirst.frc.team1089.robot;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.auton.AutonCommand;
import org.usfirst.frc.team1089.robot.commands.CalibrateGyro;
import org.usfirst.frc.team1089.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1089.robot.subsystems.Agitator;
import org.usfirst.frc.team1089.robot.subsystems.Climber;
import org.usfirst.frc.team1089.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1089.robot.subsystems.ExampleSubsystem;
import org.usfirst.frc.team1089.robot.subsystems.Gear;
import org.usfirst.frc.team1089.robot.subsystems.Intake;
import org.usfirst.frc.team1089.robot.subsystems.Feeder;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.subsystems.Shooter.ShooterEnum;
import org.usfirst.frc.team1089.robot.subsystems.Ultrasonic;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

//Q: How many programmers does it take to change a light bulb?
//Scroll down to find out!

public class Robot extends IterativeRobot {

	// Declare subsystems (public static so there is only ever one instance)
	public static ExampleSubsystem exampleSubsystem;
	public static DriveTrain driveTrain;
	public static VisionProcessor visionProcessor;
	public static Ultrasonic ultrasonic;
	public static Shooter leftShooter, rightShooter, shooter;
	public static Feeder rightFeeder, leftFeeder;
	public static Agitator rumbler;
	public static Gear gear;
	public static Climber climber;
	public static Intake intake;
	public static Servo latchServo;
	public static OI oi;
	
	AutonCommand autonomousCommand;
	
	private double changeRightEnc;
	private double changeLeftEnc;
	
	private ShooterEnum choice;
	Alliance allianceColor;
	
	/**
	 * This function is ho when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Flush the NetworkTables
		NetworkTable.flush();
		
		// Instantiate the subsystems
		exampleSubsystem = new ExampleSubsystem();
		//shooter = new Shooter();
		visionProcessor = new VisionProcessor();
		driveTrain = new DriveTrain();
		gear = new Gear();
		ultrasonic = new Ultrasonic();
		leftShooter = new Shooter(RobotMap.CAN.LEFT_SHOOTER_TALON_ID, RobotMap.CAN.LEFT_FEEDER_TALON_ID, 0.06, 0, 0, 0.4, -1);			
		rightShooter = new Shooter(RobotMap.CAN.RIGHT_SHOOTER_TALON_ID, RobotMap.CAN.RIGHT_FEEDER_TALON_ID, 0.06, 0, 0, 0.4, 1);			
		//shooter = new Shooter(7);
		leftShooter.getMotor().setInverted(true);		//TODO Check if they are inverted or not
		rightShooter.getMotor().setInverted(true);
		climber = new Climber(RobotMap.CAN.CLIMBER_TALON_ID);
		intake = new Intake(RobotMap.CAN.INTAKE_TALON_ID);
		rumbler = new Agitator(RobotMap.CAN.AGITATOR_TALON_ID);
		// OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
		
		latchServo = new Servo(1);
		
		oi = new OI();
		
		// Put some good data onto the SmartDashboard
		// chooser.addObject("My Auto", new MyAutoCommand());
		//SmartDashboard.putData("Auto mode", chooser);
		
		changeLeftEnc = 0;
		changeRightEnc = 0;
			
		MercLogger.logMessage(Level.INFO, "Completed Robot Init");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		MercLogger.close();
		MercLogger.logMessage(Level.INFO, "Completed Disabled Init");
/*		CalibrateGyro cal = new CalibrateGyro();

		cal.start();*/	
	}

	private boolean gyroCalibrated = false;
	
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		if (!gyroCalibrated && Robot.oi.gamePad.getRawButton(RobotMap.GamepadButtons.START)) {	
			gyroCalibrated = true;
			Robot.driveTrain.calibrateGyro();
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		MercLogger.createLog("/home/lvuser/log/auton");
		autonomousCommand = new AutonCommand(oi.getStartPosition(), DriverStation.getInstance().getAlliance()/*Alliance.Blue*/, 
											 oi.getFirstMovement(), oi.getFirstAction(), oi.getSecondMovement(), oi.getSecondAction());
		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start(); 
		
		MercLogger.logMessage(Level.INFO, "Completed Auton Init");
	}
	

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		MercLogger.createLog("/home/lvuser/log/teleop");
		DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks();
		
		if (autonomousCommand != null) 
			autonomousCommand.cancel();
	
		
		if (driveWithJoysticks != null)
			driveWithJoysticks.start();
			
		MercLogger.logMessage(Level.INFO, "Completed Teleop Init");
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//A: None, that's a hardware problem!

		
		Scheduler.getInstance().run();
		
		oi.updateOI();
		//choice = oi.getShot();
		/*switch(choice){
		case NO_SHOOTER:
			new StopShooter(leftShooter);
			new StopShooter(rightShooter);
			break;
		case LEFT_SHOOTER:
			new RunShooter(leftShooter);
			new StopShooter(rightShooter);
			break;
		case RIGHT_SHOOTER:
			new RunShooter(rightShooter);
			new StopShooter(leftShooter);
			break;
		case DUAL_STAGGERED_SHOOTER:
		case DUAL_SHOOTER:
			new RunShooter(leftShooter);
			new RunShooter(rightShooter);
			break;*/
		
		
		System.out.println("GetDistance: " + visionProcessor.getDistance(VisionProcessor.TargetType.HIGH_GOAL));
		System.out.println("GetAngle: " + visionProcessor.getAngleFromCenter(VisionProcessor.TargetType.HIGH_GOAL));
		
		//SmartDashboard.putNumber("Gear Delivery Movements: distance", Utilities.round(DeliverGear.getAlignMovements()[0], 3));
		//SmartDashboard.putNumber("Gear Delivery Movements: angle", Utilities.round(DeliverGear.getAlignMovements()[1], 3));
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
