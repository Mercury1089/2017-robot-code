package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DeliverGear extends CommandGroup {
	
	private double tapeWidthFeetByTwo = 5.125 / 12;

    public DeliverGear() {
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
    	addSequential(new DriveDistance(60));
    	addSequential(new DegreeRotate(60));
    	addSequential(new DriveDistance(60));
    }
    
    public double[] getAlignMovements() {			//Where getAlignMovements()[0] is the Move distance 
    												//and getAlignMovements()[1] is the turn angle
    	
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] 
    					 <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1;//Coming in from right
    	
    	double liftDistance = Robot.visionProcessor.getAverageDistanceToGearTargets(),
    		   angleFromTargetTape = Robot.visionProcessor.getAnglesFromGearTargets()[targetTape],
    		   liftAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION),
    		   targetTapeDistance = Robot.visionProcessor.getDistancesToGearTargets()[targetTape];
    	
    	double delta = Math.toDegrees(Math.asin(liftDistance 
    			* (Math.sin(Math.toRadians(angleFromTargetTape - liftAngle))
    			/ (tapeWidthFeetByTwo))));
    	
    	double epsilon = 180 - delta;
    	double distanceFromRetro = targetTapeDistance / Math.cos(Math.toRadians(epsilon));
    	double beta = 90 - epsilon;
    	double theta = angleFromTargetTape + beta;
    	double distToMove = (tapeWidthFeetByTwo + distanceFromRetro) / Math.sin(Math.toRadians(theta));
    	
    	return new double[] {distToMove, theta};
    }
}
