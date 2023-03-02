// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

import frc.robot.subsystems.Intake;
//import frc.robot.commands.ArmControlExtend;
//import frc.robot.commands.IntakeConeIn;
//import frc.robot.commands.DriveTrainBackwards;
//import frc.robot.commands.DriveTrainBalance;

/**
 *
 */
public class AutonomousPlaceConeBalance extends SequentialCommandGroup {

    public AutonomousPlaceConeBalance(Intake theIntake, Arm theArm, DriveTrain theDriveTrain){

// Pick up cone

addCommands(new IntakeConeIn(theIntake).withTimeout(1));

// Extend arm 

addCommands(new IntakeConeHoldStart(theIntake).withTimeout(3));

addCommands(new ArmControlExtend(theArm).withTimeout(2));

// Drop cone

addCommands(new IntakeConeOut(theIntake).withTimeout(2.5));

// Drive backwards

addCommands(new DriveTrainBackwards(theDriveTrain).withTimeout(2.5));

// Balance

addCommands(new DriveTrainBalance(theDriveTrain).withTimeout(6));

    //TODO create a startShooterLow command (so it will keep running, all during auton) and 
        //TODO create a StartIntake command
       // addCommands(new GyroReset(theDriveTrain));
        // Turn on Intake
        // go forward 50"
        // pick up the ball
       // ParallelRaceGroup driveForwardWithIntakeShooter = new ParallelRaceGroup(        
           // new DriveTrainMoveStraight(theDriveTrain, -75 /*Distance*/, 7 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 0 /*Angle to drive straight on*/),
           // new IntakeMove(theIntake),
            //new ArmMoveLow(theArm)
      //  );  
       // addCommands(driveForwardWithIntakeShooter);

        // turn around to face hub
        //addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180/*TurnToAngle*/));
        //ParallelRaceGroup turnWithFlywheel = new ParallelRaceGroup(
           // new DriveTrainTurnSpinToAngle(theDriveTrain, 180),
            //new ArmMoveLow(theArm)
        //);
        //addCommands(turnWithFlywheel);

        // go forward 50"
        // turn on shooter 
        // wait for shooter to charge up then move ball up on the conveytor
        //ParallelRaceGroup driveForwardWithShooter70 = new ParallelRaceGroup(        
            //new ShooterMoveLow(shooter),
            //new IntakeMove(theIntake),
            //new DriveTrainMoveStraight(theDriveTrain, -75 /*Distance*/, 7 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 180 /*Angle to drive straight on*/)
            //);
        //addCommands(driveForwardWithShooter70);

        //shoot the balls 
        //CommandGroupBase spinAndShootAndintake = SequentialCommandGroup.parallel(
           // new ArmMoveLow(shooter),
            //new IntakeMove(theIntake),
         

        //Turn clockwise (180 + 50) degrees
        //addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180+240/*TurnToAngle*/));

        //Drive 120 inches with intake
        //ParallelRaceGroup driveForwardWithIntake120 = new ParallelRaceGroup(
           // new DriveTrainMoveStraight(theDriveTrain, -120 /*Distance*/, 7 /*maxSpeed ft/sec*/, 15 /*inch to get to maxSpeed*/, 180+240 /*Angle to drive straight on*/),
           // new IntakeMove(theIntake)
        //);
        //addCommands(driveForwardWithIntake120);
    
        //Turn around clockwise (230 + 150) degree
        //addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, (180+240 + 155)));
    
        //Drive 50 inches with shooter for long shot
        //ParallelRaceGroup driveForwardWithShooter50 = new ParallelRaceGroup(        
      
           // new IntakeMove(theIntake),
            //new DriveTrainMoveStraight(theDriveTrain, -60 /*Distance*/, 7 /*maxSpeed ft/sec*/, 15 /*inch to get to maxSpeed*/, (180+240 + 155) /*Angle to drive straight on*/)
        //);
        //addCommands(driveForwardWithShooter50);
    
        //Engage conveyor and keep shooter at consistent speed
        //ParallelRaceGroup spinAndShootLong = new ParallelRaceGroup(
           // new ShooterMoveMed(shooter),
           // new ConveyorMove(theConveyor)
        //);
        //addCommands(spinAndShootLong);

    }



}
