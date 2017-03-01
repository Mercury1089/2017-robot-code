package org.usfirst.frc.team1089.robot.auton;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.AutoAlign;
import org.usfirst.frc.team1089.robot.commands.DegreeRotate;
import org.usfirst.frc.team1089.robot.commands.DeliverGear;
import org.usfirst.frc.team1089.robot.commands.DriveDistance;
import org.usfirst.frc.team1089.robot.commands.ToggleGearDelivery;
import org.usfirst.frc.team1089.robot.util.MercLogger;
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
	 * @param StartPos - from 1-9 as determined by AutonKid 
	 * @param color - Alliance color determined by DriverStation.getInstance.getAlliance()
	 */
    public AutonCommand(int startPos, Alliance color, AutonEnum choice) {
    	
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
    	
    	MercLogger.logMessage(Level.INFO, "AutonCommand CommandGroup: Started");
    	
    	int truePos = startPos, fieldPos = 2;
		DriverStation.getInstance().getAlliance();
		//Red is switched; Blue is normal
		if (color.equals(Alliance.Red)) 		 
			truePos = 10 - startPos;			//XXX makes 9 to 1 and 1 to 9, etc
		
		double[] distances = AutonMath.autonDistances(truePos);
    	
    	int reversalFactor = 1;
    	
    	if(truePos >= 7 && truePos <= 9) 	
    		reversalFactor = -1;
    	
    	if(truePos < 4)
    		fieldPos = 1; 					//Loading Station side
    	else if(truePos > 6)
    		fieldPos = 3;
    	
    	MercLogger.logMessage(Level.INFO, "AutonCommand: Auton Part 1 - Started");
    	//Auton Step 1
    	addSequential(new DriveDistance(distances[0], 0.3, 7.0));
    	if(!(truePos >= 4 && truePos <= 6)) {
    		addSequential(new DegreeRotate(-120 * reversalFactor));	//Assuming that the gear delivery mechanism is in the back of the robot
    		addSequential(new DriveDistance(-(distances[1] - 4), 0.2, 5.0));     	//-4 to be away from Gear Lift by 4 ft ~ARBITRARY VALUE~
    	}
    	
    	addSequential(new DeliverGear());
    	
    	MercLogger.logMessage(Level.INFO, "AutonCommand: Auton Part 1 - Completed");

    	//Auton Step 2
    	MercLogger.logMessage(Level.INFO, "AutonCommand: Auton Part 2 - Started");
    	addSequential(new DriveDistance(6, 0.1, 7.0));							// ARBITRARY VALUE
    	addParallel(new ToggleGearDelivery(false));
    	addSequential(new AutoAlign(TargetType.GEAR_VISION));
    	MercLogger.logMessage(Level.INFO, "AutonCommand: Auton Part 2 - Completed");

    	//Auton Step 3
    	//FIXME many of these values are wrong
    	MercLogger.logMessage(Level.INFO, "AutonCommand: Auton Part 3 - Started");

    	switch(choice) {
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
    	}
    	MercLogger.logMessage(Level.INFO, "AutonCommand CommandGroup: Completed");
    }
}
