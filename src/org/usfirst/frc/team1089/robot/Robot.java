package org.usfirst.frc.team1089.robot;

import org.usfirst.frc.team1089.robot.auton.AutonCommand;
import org.usfirst.frc.team1089.robot.auton.AutonEnum;
import org.usfirst.frc.team1089.robot.subsystems.*;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// Declare subsystems (public static so there is only ever one instance)
	public static ExampleSubsystem exampleSubsystem;
	public static DriveTrain driveTrain;
	public static VisionProcessor visionProcessor;
	public static Sensors sensors;
	//public static Shooter shooter;
	public static Gear gear;
	public static OI oi;
	
	AutonCommand autonomousCommand;
	
	private double changeRightEnc;
	private double changeLeftEnc;
	
	private AutonEnum step3;
	
	Alliance allianceColor;
	/**
	 * This function is ho when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		MercLogger.init("/home/lvuser/log/");
		
		// Flush the NetworkTables
		NetworkTable.flush();
		
		// Instantiate the subsystems
		exampleSubsystem = new ExampleSubsystem();
		sensors = new Sensors();
		shooter = new Shooter();
		visionProcessor = new VisionProcessor();
		driveTrain = new DriveTrain();
		gear = new Gear();
		
		// OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
		
		oi = new OI();
		
		// Put some good data onto the SmartDashboard
		// chooser.addObject("My Auto", new MyAutoCommand());
		//SmartDashboard.putData("Auto mode", chooser);
		SmartDashboard.putData("PID", driveTrain);
		SmartDashboard.putNumber("Gyro", driveTrain.getGyro().getAngle());
		SmartDashboard.putNumber("Angle", 0);
		
		SmartDashboard.putNumber("Left Enc Inches", 0);
		SmartDashboard.putNumber("Right Enc Inches", 0);
		
		SmartDashboard.putNumber("SetRightChange", 0);
		SmartDashboard.putNumber("SetLeftChange", 0);
		
		changeLeftEnc = 0;
		changeRightEnc = 0;
			
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		driveTrain.getGyro().reset();
		driveTrain.getNAVX().reset();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.resetEncoders();
		SmartDashboard.putNumber("Gyro", driveTrain.getGyro().getAngle());
		SmartDashboard.putNumber("NAV-X", Robot.driveTrain.getNAVX().getAngle());
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
		
		autonomousCommand = new AutonCommand(oi.getStartPos(), DriverStation.getInstance().getAlliance()/*Alliance.Blue*/, oi.getStep3());
		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start(); 
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("NAV-X", Robot.driveTrain.getNAVX().getAngle());
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		oi.updateOI();
		
		System.out.println("GetDistance: " + visionProcessor.getDistance(VisionProcessor.TargetType.GEAR_VISION));
		System.out.println(visionProcessor.getAngleFromCenter(VisionProcessor.TargetType.GEAR_VISION));
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
