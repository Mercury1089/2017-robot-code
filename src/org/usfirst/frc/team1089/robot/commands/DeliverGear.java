package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DeliverGear extends CommandGroup {
	
	private static double centerToCenterDistanceByTwo = 5.125 / 12 - 1;

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
    
    public static double[] getAlignMovements() {			//Where getAlignMovements()[0] is the Move distance 
    												//and getAlignMovements()[1] is the turn angle
    	
    	//Getting the closer tape
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] 
    					 <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1;//Coming in from right
    	SmartDashboard.putNumber("TargetTape", targetTape);
    	
    	double liftDistance = Robot.visionProcessor.getAverageDistanceToGearTargets(),				//Gets distance to center of lift
    		   angleFromTargetTape = Robot.visionProcessor.getAnglesFromGearTargets()[targetTape],	//Angle from closer tape
    		   liftAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION),		//Angle from center of lift
    		   targetTapeDistance = Robot.visionProcessor.getDistancesToGearTargets()[targetTape];	//Distance from target tape
    	SmartDashboard.putNumber("liftDistance", liftDistance);
    	SmartDashboard.putNumber("angleFromTargetTape", angleFromTargetTape);
    	SmartDashboard.putNumber("liftAngle", liftAngle);
    	SmartDashboard.putNumber("targetTapeDistance", targetTapeDistance);
    	
    	//
    	double distanceFromRetroHorizontal = 
    			(Math.pow(targetTapeDistance, 2) - Math.pow(liftDistance, 2)) / (centerToCenterDistanceByTwo * 2) - 
    			(centerToCenterDistanceByTwo * 2) / 4;
    	SmartDashboard.putNumber("distanceFromRetroHorizontal", distanceFromRetroHorizontal);
    	
    	//
    	double distanceFromLiftFace =
    			Math.sqrt(Math.pow(targetTapeDistance,  2) - Math.pow(distanceFromRetroHorizontal, 2));
    	
    	//
    	double phi = 
    			Math.toDegrees(Math.atan(distanceFromRetroHorizontal / distanceFromLiftFace));
    	
    	//Getting the distance to move
    	double distToMove = 
    			(distanceFromRetroHorizontal + centerToCenterDistanceByTwo) / Math.sin(Math.toRadians(phi + angleFromTargetTape));
    	
    	//Return. Congratulations! You have made it.
    	return new double[] {distToMove, phi};
    }
}
