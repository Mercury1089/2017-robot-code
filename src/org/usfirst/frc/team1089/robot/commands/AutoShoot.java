package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoShoot extends CommandGroup {

    public AutoShoot() {
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
    	
    	CalculateHighGoalPath CalculateHighGoalPath = new CalculateHighGoalPath();
    	addSequential(CalculateHighGoalPath);
    	
    	//addSequential(new DriveDistance(CalculateHighGoalPath::getDistance, 5.0));
    	//addSequential(new DegreeRotate(CalculateHighGoalPath::getAngle));
    	addSequential(new AutoAlign(TargetType.HIGH_GOAL));
    	addSequential(new ShootWithDistance(Robot.leftShooter, CalculateHighGoalPath::getDistance));
     	addSequential(new ShootWithDistance(Robot.rightShooter, CalculateHighGoalPath::getDistance));
    }
}
