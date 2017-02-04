package org.usfirst.frc.team1089.robot.auton;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.DegreeRotate;
import org.usfirst.frc.team1089.robot.commands.DriveDistance;

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
		//Blue is switched; Red is normal
		if (color.equals(Alliance.Blue)) 		 
			truePos = 10 - startPos;			//XXX makes 9 to 1 and 1 to 9, etc
		
    	
    	int reversalFactor = 1;
    	
    	if(truePos >= 7 && truePos <= 9) 	
    		reversalFactor = -1;
    	
    	if(truePos < 4)
    		fieldPos = 1; 					//Loading Station side
    	else if(truePos > 6)
    		fieldPos = 3;
    	
    	
    	//FIXME A LOT OF THIS COULD BE WRONG.    	
    	
    	//Auton Step 1
    	addSequential(new DriveDistance(90));	//TODO Change 0 to a value determined by SmartDashboard value
    	if(!(truePos >= 4 && truePos <= 6))
    		addSequential(new DegreeRotate(60 * reversalFactor));
    											//TODO Can only be 30 or -30; change to var based on Alliance color
    	//addSequential(new AutoAlign());		//TODO Code AutoAlign 
    	addSequential(new DriveDistance(60));	//TODO Change 0 to a value determined by SmartDashboard value
    	//addSequential(new DropGear());		//TODO Code DropGear 
    	
    	//Auton Step 2
    	addSequential(new DriveDistance(-40));	//TODO Change -0 to a negative value determined by SmartDashboard value
    	//addSequential(new AutoAlign());		//to lift
    	
    	//Auton Step 3    						
    	switch(choice) {
    	case FAR_HOPPER:								//TODO make a far hopper sequence
    	case NEAR_HOPPER:
    		if(fieldPos == 1) {
    			addSequential(new DegreeRotate(-60 * reversalFactor));
    			addSequential(new DriveDistance(100));	//TODO Change 10 to actual distance from Smartdash that is different based on FAR/NEAR				
    			addSequential(new DegreeRotate(90 * reversalFactor));
    			addSequential(new DriveDistance(-30));	//TODO Change -10
    		}
    		else if(fieldPos == 3){
    			addSequential(new DegreeRotate(-30 * reversalFactor));	
    			addSequential(new DriveDistance(-5));	
    			//make sure you pick up
    			//turn and shoot
    		}
    		break;
    	case TURN_SHOOT:
    		if(fieldPos == 1) {
    			addSequential(new DegreeRotate(60 * reversalFactor));
    			//Shoot;;;
    		}
    		else if(fieldPos == 2) {
    			if(color.equals(Alliance.Red))
    				addSequential(new DegreeRotate(110));
    			else
    				addSequential(new DegreeRotate(-110));
    				//Shoot
    		}
    		else if(fieldPos == 3) {
    			addSequential(new DegreeRotate(180 * reversalFactor)); //FIXME Not actually 180, needs to be fixed hence the FIXME xD
        		//Shoot (With autoalign lmao ecks dee)
    		}
    		break;
    		
    	}
    }
}
