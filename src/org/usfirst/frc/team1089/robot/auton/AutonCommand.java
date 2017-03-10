package org.usfirst.frc.team1089.robot.auton;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.commands.AutoShoot;
import org.usfirst.frc.team1089.robot.commands.BasicGearDelivery;
import org.usfirst.frc.team1089.robot.commands.DegreeRotate;
import org.usfirst.frc.team1089.robot.commands.DeliverGear;
import org.usfirst.frc.team1089.robot.commands.DriveDistance;
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
    	
    	int truePos = startPos.ordinal(); // returns position as integer based on order in enum declaration
    	// it is important to note that this only works because an unused zero-position is declared in the enum 
    	
    	AutonFieldPosition fieldPos = AutonFieldPosition.MIDDLE;
		DriverStation.getInstance().getAlliance();
		//Red is switched; Blue is normal
		if (color.equals(Alliance.Blue)) 		 
			truePos = 10 - startPos.ordinal();			//XXX makes 9 to 1 and 1 to 9, etc=
    	
    	int reversalFactor = -1;
    	
    	if(truePos >= 7 && truePos <= 9) 	
    		reversalFactor = 1;
    	
    	if(truePos < 4)
    		fieldPos = AutonFieldPosition.LEFT; 					//Loading Station side
    	else if(truePos > 6)
    		fieldPos = AutonFieldPosition.RIGHT;
    	
    	//AutonFirstMovement
    	switch(firstMovement) {
    	case DO_NOTHING:
    		return;
    	case DRIVE_FORWARD:
    		addSequential(new DriveDistance(8));
    		return;
    	case GO_TO_LIFT:
    		double[] distances = AutonMath.autonDistances(truePos);
    		
        	/*if(!(truePos >= 4 && truePos <= 6)) {
        		addSequential(new DriveDistance(-distances[0], 0.1, 3.0));
        		addSequential(new DegreeRotate(-60 * reversalFactor));	//Assuming that the gear delivery mechanism is in the back of the robot
        		addSequential(new DriveDistance((distances[1] - 4), 0.1, 3.0));     	//-4 to be away from Gear Lift by 4 ft ~ARBITRARY VALUE~
        	}*/
        	
    		MercLogger.logMessage(Level.INFO, "True Pos: " +truePos);
    		
    		if(truePos == 3) {
    			addSequential(new DriveDistance(-5.5, 0.1, 3.0));
        		addSequential(new DegreeRotate(60));	//Assuming that the gear delivery mechanism is in the back of the robot
        		//addSequential(new BasicGearDelivery());
    			
        		//addSequential(new DriveDistance((distances[1] - 4), 0.1, 3.0));     	//-4 to be away from Gear Lift by 4 ft ~ARBITRARY VALUE~
    		}
    		else if(truePos == 7) {
    			addSequential(new DriveDistance(-5.5, 0.1, 3.0));
        		addSequential(new DegreeRotate(-60));
    		}
    		else {
        		addSequential(new DriveDistance(-2));
        	}
    		break;
    	case GO_TO_SHOOTING_RANGE:
    		return; 															//TODO LATER
    	}
    	
    	//AutonFirstAction
    	switch(firstAction) {
    	case DO_NOTHING:
    		return;
    	case DELIVER_GEAR:
    		addSequential(new BasicGearDelivery());
    		addSequential(new DriveDistance(5.4, 0.1, 3.0));
   			addParallel(new ToggleGearDelivery(false));
    	case SHOOT:
    		if(firstMovement == AutonFirstMovement.GO_TO_SHOOTING_RANGE)
    			addSequential(new AutoShoot());
    		return;																//TODO fix AutoShoot
    	}    	

    	//AutonSecondMovement
    	switch(secondMovement) {
    	case STOP:
    		return;
    	case NEAR_HOPPER:
    		if(fieldPos == AutonFieldPosition.LEFT) {
    			addSequential(new DegreeRotate(120 * reversalFactor));
    			addSequential(new DriveDistance(7.425));
    			addSequential(new DegreeRotate(-90 * reversalFactor));
    			addSequential(new DriveDistance(6.75));
    		}
    		else if(fieldPos == AutonFieldPosition.RIGHT) {
    			addSequential(new DegreeRotate(-30 * reversalFactor));	
    			addSequential(new DriveDistance(6.75));
    			/*addSequential(new DriveDistance(5));
    			addSequential(new DegreeRotate(45));*/
    			//Shoot
    		}
    		else
    			return;//if its in the middle
    		break;
    	case FAR_HOPPER:
    		if(fieldPos == AutonFieldPosition.LEFT) {
    			addSequential(new DegreeRotate(120 * reversalFactor));
    			addSequential(new DriveDistance(18.9 + 7.425));
    			addSequential(new DegreeRotate(-90 * reversalFactor));
    			addSequential(new DriveDistance(6.75));
    		}
    		else if(fieldPos == AutonFieldPosition.RIGHT) {
    			addSequential(new DegreeRotate(-120 * reversalFactor));
    			addSequential(new DriveDistance(16.2));
    			addSequential(new DegreeRotate(90 * reversalFactor));
    			addSequential(new DriveDistance(6.75));
    		}
    		else
    			return;//if its in the middle
    		break;
    	case SHOOTING_RANGE:
    		if(fieldPos == AutonFieldPosition.LEFT) {
    			addSequential(new DegreeRotate(125 * reversalFactor));		
    			addSequential(new DriveDistance(8.78));					// ARBITRARY VALUE
    		}
    		else if(fieldPos == AutonFieldPosition.MIDDLE) {
    			if(color.equals(Alliance.Red))
    				addSequential(new DegreeRotate(-70));
    			else
    				addSequential(new DegreeRotate(70));
    		}
    		else if(fieldPos == AutonFieldPosition.RIGHT) {
    			addSequential(new DegreeRotate(10 * reversalFactor)); //FIXME Not actually 180, needs to be fixed hence the FIXME xD
    		}
    		//Shoot
    		break;
    	}
    	
    	//AutonSecondAction
    	switch(secondAction) {
    	case SHOOT:
    		addSequential(new AutoShoot());
    		break;
    	case STOP:
    		return;
    	}
    }
}
