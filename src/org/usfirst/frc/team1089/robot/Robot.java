
package org.usfirst.frc.team1089.robot;

import org.usfirst.frc.team1089.robot.auton.AutonCommand;
import org.usfirst.frc.team1089.robot.auton.AutonEnum;
import org.usfirst.frc.team1089.robot.commands.ExampleCommand;
import org.usfirst.frc.team1089.robot.subsystems.*;
import org.usfirst.frc.team1089.robot.util.Debug;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	public static VisionSystem visionSystem;
	public static OI oi;
	
	AutonCommand autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	private double changeRightEnc;
	private double changeLeftEnc;

	Alliance allianceColor;
	int autonStartPos;
	
	AutonEnum autonChoice;					//TODO set equal to value from SmartDash
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Initialize log file
		Debug.init("/home/lvuser/log");
		
		// Flush the NetworkTables
		NetworkTable.flush();
		
		// Instantiate the subsystems
		exampleSubsystem = new ExampleSubsystem();
		visionSystem = new VisionSystem();
		driveTrain = new DriveTrain();
		
		// OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
		
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		SmartDashboard.putData("PID", driveTrain);
		SmartDashboard.putNumber("Gyro", driveTrain.getGyro().getAngle());
		SmartDashboard.putNumber("Angle", 0);
		
		SmartDashboard.putNumber("Left Enc Inches", 0);
		SmartDashboard.putNumber("Right Enc Inches", 0);
		
		SmartDashboard.putNumber("SetRightChange", 0);
		SmartDashboard.putNumber("SetLeftChange", 0);
		
		changeLeftEnc = 0;
		changeRightEnc = 0;
		
		allianceColor = DriverStation.getInstance().getAlliance();
		autonStartPos = 0;											//TODO get from SmartDash
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
		DriverStation.getInstance().getAlliance();
		autonomousCommand = new AutonCommand(4, Alliance.Blue, AutonEnum.FAR_HOPPER_1_2_3);
		
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
		SmartDashboard.putNumber("Gyro", driveTrain.getGyro().getAngle());
		SmartDashboard.putNumber("Left Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.driveTrain.getRightEncoder());
		SmartDashboard.putNumber("NAV-X", Robot.driveTrain.getNAVX().getAngle());
		SmartDashboard.putNumber("Left Enc Inches", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getLeftEncoder()) - SmartDashboard.getNumber("SetLeftChange", 0));
		SmartDashboard.putNumber("Right Enc Inches", Robot.driveTrain.encoderTicksToInches(Robot.driveTrain.getRightEncoder()) - SmartDashboard.getNumber("SetRightChange", 0));
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
