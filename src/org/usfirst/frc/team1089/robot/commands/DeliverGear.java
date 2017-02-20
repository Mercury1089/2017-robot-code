package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.Config;
import org.usfirst.frc.team1089.robot.util.Utilities;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DeliverGear extends CommandGroup {
	
	private static double centerToCenterDistanceByTwo = 5.125 / 12 - 1;
	public static final double INCH_OFFSET_FROM_TARGET = 36; //TODO Maybe move this to a better place. Also, edit this value

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
    	double d[] = getAlignMovements();
    	addSequential(new DriveDistance(d[0] * 12 + Config.ROBOT_LENGTH_PROTO / 2.0));	//FIXME Should be able to pass in feet
    	addSequential(new DegreeRotate(d[1]));
    	//addSequential(new DriveDistance(60));
    	
    }
    
    public static double[] getAlignMovements() {			//Where getAlignMovements()[0] is the Move distance 
    												//and getAlignMovements()[1] is the turn angle
    	
    	//Getting the closer tape
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] 
    					 <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1;//Coming in from right
    	int reversalFactor = targetTape == 0 ? -1 : 1;
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
    	SmartDashboard.putNumber("distanceFromRetroHorizontal", Utilities.round(distanceFromRetroHorizontal, 3));
    	
    	//
    	double distanceFromLiftFace =
    			Math.sqrt(Math.pow(targetTapeDistance,  2) - Math.pow(distanceFromRetroHorizontal, 2));
    	SmartDashboard.putNumber("distanceFromLiftFace", Utilities.round(distanceFromLiftFace, 3));
    	
    	//
    	double phi = 
    			Math.toDegrees(Math.atan(distanceFromRetroHorizontal / distanceFromLiftFace));
    	SmartDashboard.putNumber("phi", Utilities.round(phi, 3));
    	
    	double theta = Math.abs(phi) + Math.abs(angleFromTargetTape);
    	SmartDashboard.putNumber("theta", theta);
    	
    	//Getting the distance to move
    	double distToMove = 
    			(distanceFromRetroHorizontal + centerToCenterDistanceByTwo) / Math.sin(Math.toRadians(theta));
    	SmartDashboard.putNumber("distToMove", Utilities.round(distToMove, 3));
    	
    	//Return. Congratulations! You have made it.
    	return new double[] {distToMove, theta};
    }
    
    public static double[] getAlignMovementsOnAPoint() {
    	double distToMove, angleToTurn, theta;
    	
    	int targetTape = Robot.visionProcessor.getDistancesToGearTargets()[0] <= Robot.visionProcessor.getDistancesToGearTargets()[1] ? 0 : 1;//Coming in from right
    	int reversalFactor = targetTape == 0 ? -1 : 1;
    	double liftDistance = Robot.visionProcessor.getAverageDistanceToGearTargets(),				//Gets distance to center of lift
     		   angleFromTargetTape = Robot.visionProcessor.getAnglesFromGearTargets()[targetTape],	//Angle from closer tape
     		   liftAngle = Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION),		//Angle from center of lift
     		   targetTapeDistance = Robot.visionProcessor.getDistancesToGearTargets()[targetTape];	//Distance from target tape
    	
    	//
    	double distanceFromRetroHorizontal = 
    			(Math.pow(targetTapeDistance, 2) - Math.pow(liftDistance, 2)) / (centerToCenterDistanceByTwo * 2) - 
    			(centerToCenterDistanceByTwo * 2) / 4;
    	SmartDashboard.putNumber("distanceFromRetroHorizontal", Utilities.round(distanceFromRetroHorizontal, 3));
    	
    	//
    	double distanceFromLiftFace =
    			Math.sqrt(Math.pow(targetTapeDistance,  2) - Math.pow(distanceFromRetroHorizontal, 2));
    	SmartDashboard.putNumber("distanceFromLiftFace", Utilities.round(distanceFromLiftFace, 3));
    	
    	theta = Math.atan(Math.abs(distanceFromRetroHorizontal) / Math.abs(distanceFromLiftFace)); //In radians
    	
    	distToMove = Math.sqrt(Math.pow(liftDistance, 2) + Math.pow(INCH_OFFSET_FROM_TARGET, 2) 
    	- 2 * liftDistance * INCH_OFFSET_FROM_TARGET * Math.toDegrees(Math.cos(theta))); //Law of cosines
    
    	angleToTurn = Math.toDegrees(Math.atan((INCH_OFFSET_FROM_TARGET * Math.sin(theta) / distToMove)));
    	
    	return new double[] {distToMove, angleToTurn};
    }
}
