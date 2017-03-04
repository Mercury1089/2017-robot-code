package org.usfirst.frc.team1089.robot.auton;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.AutoAlign;
import org.usfirst.frc.team1089.robot.commands.DegreeRotate;
import org.usfirst.frc.team1089.robot.commands.DeliverGear;
import org.usfirst.frc.team1089.robot.commands.DriveDistance;
import org.usfirst.frc.team1089.robot.commands.ToggleGearDelivery;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

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
    	
    	int truePos = startPos.ordinal();
    	AutonFieldPosition fieldPos = AutonFieldPosition.MIDDLE;
		DriverStation.getInstance().getAlliance();
		//Red is switched; Blue is normal
		if (color.equals(Alliance.Red)) 		 
			truePos = 10 - startPos.ordinal();			//XXX makes 9 to 1 and 1 to 9, etc
		
		double[] distances = AutonMath.autonDistances(truePos);
    	
    	int reversalFactor = 1;
    	
    	if(truePos >= 7 && truePos <= 9) 	
    		reversalFactor = -1;
    	
    	if(truePos < 4)
    		fieldPos = AutonFieldPosition.LEFT; 					//Loading Station side
    	else if(truePos > 6)
    		fieldPos = AutonFieldPosition.RIGHT;
    	
    	//Auton Step 1
    	addSequential(new DriveDistance(distances[0], 0.3, 7.0));
    	if(!(truePos >= 4 && truePos <= 6)) {
    		addSequential(new DegreeRotate(-120 * reversalFactor));	//Assuming that the gear delivery mechanism is in the back of the robot
    		addSequential(new DriveDistance(-(distances[1] - 4), 0.2, 5.0));     	//-4 to be away from Gear Lift by 4 ft ~ARBITRARY VALUE~
    	}
    	
    	addSequential(new DeliverGear());

    	//Auton Step 2
    	addSequential(new DriveDistance(6, 0.1, 7.0));							// ARBITRARY VALUE
    	addParallel(new ToggleGearDelivery(false));
    	addSequential(new AutoAlign(TargetType.GEAR_VISION));

    	//Auton Step 3
    	//FIXME many of these values are wrong

    	/*switch(choice) {
	    	case FAR_HOPPER:								//TODO make a far hopper sequence and TODO Change some of these values 
	    	case NEAR_HOPPER:
	    		if(fieldPos == 1) {
	    			addSequential(new DegreeRotate(120 * reversalFactor));
	    			addSequential(new DriveDistance(20));						// ARBITRARY VALUE
	    			addSequential(new DegreeRotate(90 * reversalFactor));
	    			addSequential(new DriveDistance(-4));						// ARBITRARY VALUE
	    		}
	    		else if(fieldPos == 3){
	    			addSequential(new DegreeRotate(-30 * reversalFactor));	
	    			addSequential(new DriveDistance(-5, 2.5));					// ARBITRARY VALUE
	    			addSequential(new DriveDistance(5));						// ARBITRARY VALUE
	    			addSequential(new DegreeRotate(45));
	    			addSequential(new AutoAlign(TargetType.GEAR_VISION));
	    			//Shoot
	    		}
	    		break;
	    	case TURN_SHOOT:
	    		if(fieldPos == 1) {
	    			addSequential(new DegreeRotate(150 * reversalFactor));		
	    			addSequential(new DriveDistance(5, 0.5));					// ARBITRARY VALUE
	    		}
	    		else if(fieldPos == 2) {
	    			if(color.equals(Alliance.Red))
	    				addSequential(new DegreeRotate(110));
	    			else
	    				addSequential(new DegreeRotate(-110));
	    		}
	    		else if(fieldPos == 3) {
	    			addSequential(new DegreeRotate(180 * reversalFactor)); //FIXME Not actually 180, needs to be fixed hence the FIXME xD
	    		}
	    		addSequential(new AutoAlign(TargetType.GEAR_VISION));
	    		//Shoot
	    		break;
	    	case STOP:
	    		Robot.driveTrain.stop();
	    		break;
	    	default:
	    		Robot.driveTrain.stop();
	    		break;
    	}*/
    }
}
