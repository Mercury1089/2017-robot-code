package org.usfirst.frc.team1089.robot.auton;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.commands.AutoShoot;
import org.usfirst.frc.team1089.robot.commands.BasicGearDelivery;
import org.usfirst.frc.team1089.robot.commands.DegreeRotate;
import org.usfirst.frc.team1089.robot.commands.DeliverGear;
import org.usfirst.frc.team1089.robot.commands.DriveDistance;
import org.usfirst.frc.team1089.robot.commands.OpenLatch;
import org.usfirst.frc.team1089.robot.commands.StopAllShooters;
import org.usfirst.frc.team1089.robot.commands.ToggleGearDelivery;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Calls all necessary Commands for Auton.
 * Collects information from SmartDashboard to change Auton
 */
public class AutonCommand extends CommandGroup {
	
	/**
	 * @param startPos
	 * @param color - Alliance color determined by DriverStation.getInstance.getAlliance()
	 * @param firstMovement
	 * @param firstAction
	 * @param secondMovement
	 * @param secondAction
	 */
    public AutonCommand(AutonPosition startPos, Alliance color, AutonFirstMovement firstMovement,
    					AutonFirstAction firstAction, AutonSecondMovement secondMovement, 
    					AutonSecondAction secondAction) {
    	
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
    	
		DriverStation.getInstance().getAlliance();
		
		int reversalFactor = 1;
		
    	addParallel(new OpenLatch());
    	addParallel(new StopAllShooters());
    	
    	//AutonFirstMovement
    	switch(firstMovement) {
    	case DO_NOTHING:
    		break;
    	case DRIVE_FORWARD:
    		addSequential(new DriveDistance(-8));
    		break;
    	case GO_TO_LIFT:
    		//double[] distances = AutonMath.autonDistances(truePos);
    		
        	/*if(!(truePos >= 4 && truePos <= 6)) {
        		addSequential(new DriveDistance(-distances[0], 0.1, 3.0));
        		addSequential(new DegreeRotate(-60 * reversalFactor));	//Assuming that the gear delivery mechanism is in the back of the robot
//        		addSequential(new DriveDistance((distances[1] - 4), 0.1, 3.0));     	//-4 to be away from Gear Lift by 4 ft ~ARBITRARY VALUE~
        	}*/
        	
    		
    		if(startPos == AutonPosition.LEFT) {
    			addSequential(new DriveDistance(-5.5, 6.0));
        		addSequential(new DegreeRotate(60 * reversalFactor));	//Assuming that the gear delivery mechanism is in the back of the robot
        		//addSequential(new BasicGearDelivery());
    			
        		//addSequential(new DriveDistance((distances[1] - 4), 0.1, 3.0));     	//-4 to be away from Gear Lift by 4 ft ~ARBITRARY VALUE~
    		}
    		else if(startPos == AutonPosition.RIGHT) {
    			addSequential(new DriveDistance(-5.5, 6.0));
        		addSequential(new DegreeRotate(-50 * reversalFactor));
    		}
    		else {
        		addSequential(new DriveDistance(-3.16, 5.0));
        	}
    		break;
    	case GO_TO_SHOOTING_RANGE:
    		break; 															//TODO LATER
    	}
    	
    	//AutonFirstAction
    	switch(firstAction) {
    	case DO_NOTHING:
    		break;
    	case DELIVER_GEAR:
    		addSequential(new BasicGearDelivery());
//    		addSequential(new DeliverGear());
    		addSequential(new DriveDistance(4, 7.0));
   			addParallel(new ToggleGearDelivery(false));
    	case SHOOT:
    		if(firstMovement == AutonFirstMovement.GO_TO_SHOOTING_RANGE)
    			addSequential(new AutoShoot());
    		break;																//TODO fix AutoShoot
    	}    	

    	//AutonSecondMovement
    	switch(secondMovement) {
    	case STOP:
    		break;
    	case NEAR_HOPPER:
    		if(startPos == AutonPosition.LEFT) {
    			addSequential(new DegreeRotate(120 * reversalFactor));
    			addSequential(new DriveDistance(7.425));
    			addSequential(new DegreeRotate(-90 * reversalFactor));
    			addSequential(new DriveDistance(6.75));
    		}
    		else if(startPos == AutonPosition.RIGHT) {
    			addSequential(new DegreeRotate(-30 * reversalFactor));	
    			addSequential(new DriveDistance(6.75));
    			/*addSequential(new DriveDistance(5));
    			addSequential(new DegreeRotate(45));*/
    			//Shoot
    		}
    		else
    			break;//if its in the middle
    		break;
    	case FAR_HOPPER:
    		if(startPos == AutonPosition.LEFT) {
    			addSequential(new DegreeRotate(120 * reversalFactor));
    			addSequential(new DriveDistance(18.9 + 7.425));
    			addSequential(new DegreeRotate(-90 * reversalFactor));
    			addSequential(new DriveDistance(6.75));
    		}
    		else if(startPos == AutonPosition.RIGHT) {
    			addSequential(new DegreeRotate(-120 * reversalFactor));
    			addSequential(new DriveDistance(16.2));
    			addSequential(new DegreeRotate(90 * reversalFactor));
    			addSequential(new DriveDistance(6.75));
    		}
    		else
    			break;//if its in the middle
    		break;
    	case SHOOTING_RANGE:
    		if(startPos == AutonPosition.LEFT) {
    			addSequential(new DegreeRotate(125 * reversalFactor));		
    			addSequential(new DriveDistance(8.78));					// ARBITRARY VALUE
    		}
    		else if(startPos == AutonPosition.MIDDLE) {
    			if(color.equals(Alliance.Red))
    				addSequential(new DegreeRotate(-70));
    			else
    				addSequential(new DegreeRotate(70));
    		}
    		else if(startPos == AutonPosition.RIGHT) {
    			addSequential(new DegreeRotate(20 * reversalFactor)); //FIXME Not actually 180, needs to be fixed hence the FIXME xD
    		}
    		break;
    	}
    	
    	//AutonSecondAction
    	switch(secondAction) {
    	case SHOOT:
    		addSequential(new AutoShoot());
    		break;
    	case STOP:
    		break;
    	}
    }
}
