package org.usfirst.frc.team1089.robot.auton;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.DegreeRotate;
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
    	
    	//Auton Step 1
    	addSequential(new DriveDistance(distances[0], 0.5));
    	if(!(truePos >= 4 && truePos <= 6)) {
    		addSequential(new DegreeRotate(-120/*60*/ * reversalFactor));	//Assuming that the gear delivery mechanism is in the back of the robot
    		addSequential(new DriveDistance(-(distances[1] - 2), 0.2));     	//-2 to be away from Gear Lift by 2 ft
    	}
    	addSequential(new DegreeRotate(Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION)));
    	addSequential(new DriveDistance(-2, 0.3));					   	    //After auto aligning, drive forward to deliver gear
    	addSequential(new ToggleGearDelivery(true));
    	
    	
    	//Auton Step 2
    	addSequential(new DriveDistance(40, 0.5));
    	addSequential(new DegreeRotate(Robot.visionProcessor.getAngleFromCenter(TargetType.GEAR_VISION)));
    	
    	//Auton Step 3
    	//FIXME many of these values are wrong
    	switch(choice) {
    	case FAR_HOPPER:								//TODO make a far hopper sequence
    	case NEAR_HOPPER:
    		if(fieldPos == 1) {
    			addSequential(new DegreeRotate(120 * reversalFactor));
    			addSequential(new DriveDistance(100));	
    			addSequential(new DegreeRotate(90 * reversalFactor));
    			addSequential(new DriveDistance(-30));
    		}
    		else if(fieldPos == 3){
    			addSequential(new DegreeRotate(-30 * reversalFactor));	
    			addSequential(new DriveDistance(-60, 2.5));				//TODO Change these values
    			addSequential(new DriveDistance(60));
    			addSequential(new DegreeRotate(120));
    			addSequential(new DegreeRotate(Robot.visionProcessor.getAngleFromCenter(TargetType.HIGH_GOAL)));
    			//Shoot
    		}
    		break;
    	case TURN_SHOOT:
    		if(fieldPos == 1) {
    			addSequential(new DegreeRotate(-110 * reversalFactor));
    			addSequential(new DriveDistance(36, 0.5));
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
    		addSequential(new DegreeRotate(Robot.visionProcessor.getAngleFromCenter(TargetType.HIGH_GOAL)));
    		//Shoot
    		break;
    		
    	}
    }
}
