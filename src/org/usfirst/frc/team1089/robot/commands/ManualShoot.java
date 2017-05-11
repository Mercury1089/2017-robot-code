package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.BoilerTargetProvider;
import org.usfirst.frc.team1089.robot.util.ITargetProvider;
import org.usfirst.frc.team1089.robot.util.ManualTargetProvider;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ManualShoot extends CommandGroup {

    public ManualShoot() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	ITargetProvider targetProvider = new ManualTargetProvider();
    	
    	addParallel(new ShootWithDistance(Robot.leftShooter, targetProvider));
     	addSequential(new ShootWithDistance(Robot.rightShooter, targetProvider));
    }
}