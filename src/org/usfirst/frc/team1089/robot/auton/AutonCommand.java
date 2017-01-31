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
    public AutonCommand(int startPos, Alliance color/*, AutonEnum choice*/) {
    	
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
    	
    	int truePos = startPos;
		DriverStation.getInstance().getAlliance();
		//Blue is switched; Red is normal
		if (color.equals(Alliance.Blue)) 		 
			truePos = 10 - startPos;			//XXX makes 9 to 1 and 1 to 9, etc
		
    	
    	int reversalFactor = 1;
    	
    	if(startPos >= 7 && startPos <= 9) 	
    		reversalFactor = -1;
    	else if(startPos >= 4 && startPos <= 6)
    		reversalFactor = 0;
    	
    	
    	//Auton Step 1
    	addSequential(new DriveDistance(90));	//TODO Change 0 to a value determined by SmartDashboard value
    	addSequential(new DegreeRotate(30 * reversalFactor));	
    											//TODO Can only be 30 or -30; change to var based on Alliance color
    	//addSequential(new AutoAlign());		//TODO Code AutoAlign 
    	addSequential(new DriveDistance(60));	//TODO Change 0 to a value determined by SmartDashboard value
    	//addSequential(new DropGear());		//TODO Code DropGear 
    	
    	//Auton Step 2
    	addSequential(new DriveDistance(-40));	//TODO Change -0 to a negative value determined by SmartDashboard value
    	//addSequential(new AutoAlign());		//to lift
    	
    	//Auton Step 3    	
    	
    }
}
