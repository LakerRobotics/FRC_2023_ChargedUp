// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;

import javax.swing.tree.ExpandVetoException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    //public final Climber m_climber = new Climber();
    public final Arm m_arm = new Arm();
    public final Intake m_intake = new Intake();
    public final DriveTrain m_driveTrain = new DriveTrain();

// Joysticks
private final XboxController driverController = new XboxController(0);
private final XboxController operatorController = new XboxController(1);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems

    //m_chooser.addOption("CurvedPath", loadPathplannerTrajectoryToRamseteCommand(
 //     "C:\\Users\\Laker-Programming\\FRC2023ChargedUp3\\src\\main\\deploy\\pathplanner\\generatedJSON\\Curved Path.wpilib.json",
      //"pathplanner/generatedJSON/Curved Path.wpilib.json",
       //true));
    //m_chooser.addOption("GoStraight", loadPathplannerTrajectoryToRamseteCommand(
//      "C:\\Users\\Laker-Programming\\FRC2023ChargedUp3\\src\\main\\deploy\\pathplanner\\generatedJSON\\Go Straight.wpilib.json",
      //"pathplanner/generatedJSON/GoStraight.wpilib.json",
       //true));

       //m_chooser.addOption("Left Side Go Straight and Get Away ", loadPathplannerTrajectoryToRamseteCommand(
        //"pathplanner/generatedJSON/Left Side Go Forward and Get Away.wpilib.json",
         //true));
      
      //m_chooser.addOption("Left Side Cone in Slot", loadPathplannerTrajectoryToRamseteCommand(
        //"pathplanner/generatedJSON/Test Left Cone.wpilib.json",
       //true));

      //m_chooser.addOption("8", loadPathplannerTrajectoryToRamseteCommand(
       // "pathplanner/generatedJSON/New Path.wpilib.json",
     // true));

     // m_chooser.addOption("Circle", loadPathplannerTrajectoryToRamseteCommand(
       // "pathplanner/generatedJSON/Circle.wpilib.json",
     // true));

      //m_chooser.addOption("The John Path", loadPathplannerTrajectoryToRamseteCommand(
       // "pathplanner/generatedJSON/the john Path.wpilib.json",
      //true));

      //works
      //m_chooser.addOption("Foward and Backward", loadPathplannerTrajectoryToRamseteCommand(
        //"pathplanner/generatedJSON/Fowrad and Backward.wpilib.json",
      //true));

      //m_chooser.addOption("Test Right Turn", loadPathplannerTrajectoryToRamseteCommand(
      //"pathplanner/generatedJSON/Test Right Turn.wpilib.json",
      //true));

      //m_chooser.addOption("Forward test", loadPathplannerTrajectoryToRamseteCommand(
        //"pathplanner/generatedJSON/Forward test.wpilib.json",
      //true));
      
      m_chooser.addOption("Place Cone", new AutonomousPlaceCone(m_intake, m_arm, m_driveTrain)
      );

      m_chooser.addOption("Place Cone and Balance", new AutonomousPlaceConeBalance(m_intake, m_arm, m_driveTrain)
      );

      m_chooser.addOption("Place Drive Over and Balance", new AutonomousPlaceDriveBalance(m_intake, m_arm, m_driveTrain)
      );

      m_chooser.addOption("Place Drive Straight and Balance", new AutonomousPlaceDriveStraightBalance(m_intake, m_arm, m_driveTrain)
      );

      m_chooser.addOption("Place Cube and Balance", new AutonomousPlaceCubeandBalance(m_intake, m_arm, m_driveTrain)
      );


     m_chooser.setDefaultOption("Place Cone and Balance", new AutonomousPlaceConeBalance(m_intake, m_arm, m_driveTrain));
//      "C:\\Users\\Laker-Programming\\FRC2023ChargedUp3\\src\\main\\deploy\\pathplanner\\generatedJSON\\Go Straight.wpilib.json",
       //"pathplanner/generatedJSON/GoStraight.wpilib.json",
//true));

  
    Shuffleboard.getTab("Autonomous").add(m_chooser);


    // SmartDashboard Buttons
    //SmartDashboard.putData("AutonomousCommand", new AutonomousShootandBackupStraight(m_intake,m_shooter,m_conveyor,m_driveTrain));
    SmartDashboard.putData("DriveTrainArcade", new DriveTrainArcade( m_driveTrain ));
    SmartDashboard.putData("IntakeConeIn", new IntakeConeIn( m_intake ));
    SmartDashboard.putData("IntakeStop", new IntakeStop( m_intake ));
    SmartDashboard.putData("IntakeControlPower", new IntakeTeleop( m_intake ));
    SmartDashboard.putData("IntakeControlSpeed", new IntakeConeHoldStart( m_intake ));
    //SmartDashboard.putData("DriveTrainMoveStraight", new DriveTrainMoveStraight(m_driveTrain, 100, 3, 10, 10));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
//    m_climber.setDefaultCommand(new ClimberMove( m_climber ) );
    m_intake.setDefaultCommand(new IntakeTeleop( m_intake ) );
    m_arm.setDefaultCommand(new ArmControlPower( m_arm ) );
    m_driveTrain.setDefaultCommand(new DriveTrainArcade( m_driveTrain ) );
    //m_driveTrain.setDefaultCommand(new DriveTrainFieldOrientated(m_driveTrain ) );


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    //m_chooser.setDefaultOption("$command.getName()", new ${name.replace(' ', '')}( m_${name.substring(0,1).toLowerCase()}${name.substring(1).replace(' ', '')} ));
//m_chooser.setDefaultOption("AutnomouseShootAndBackup",         new AutonomousShootandBackup(         m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutnomouseShootAndBackup",         new AutonomousShootandBackup(         m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutnomouseShootAndBackupStraight", new AutonomousShootandBackupStraight( m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutnomouseStraight",               new AutonomousWithDriveStraight(      m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutnomouseTurnArround",            new AutonomousTurnArround(            m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "Autonomous2Ball_AlignToSecondBall",new Autonomous2Ball_AlignToSecondBall(m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutonomousTestForwardBack",        new AutonomousTestForwardBack(        m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutonomousGetBall_andShoot",       new AutonomousGetBall_andShoot(       m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "Autonomous3BallRight",             new Autonomous3BallRight(             m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "Autonomous3BallRightFaster",       new Autonomous3BallRightFaster(       m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption(       "AutonomousTestCircle",             new AutonomousTestCircle(             m_intake, m_shooter, m_conveyor, m_driveTrain));

                                                                             // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
  
    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry){
    Trajectory trajectory;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory" + filename,exception.getStackTrace());
      System.out.println("Unable to read from file" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand =
     new RamseteCommand(trajectory, m_driveTrain::getPose, 
                        new RamseteController(DriveTrainConstants.kRamsetB, DriveTrainConstants.kRamseteZeta), 
                        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter,DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
                        DriveTrainConstants.kDriveKinematics,
                        m_driveTrain::getWheelSpeeds, 
                        new PIDController(DriveTrainConstants.kpDriveVel, 0, 0), 
                        new PIDController(DriveTrainConstants.kpDriveVel, 0, 0), 
                        m_driveTrain::tankDriveVolts, m_driveTrain
                       );

    if(resetOdometry){
      return new SequentialCommandGroup(new InstantCommand(()->m_driveTrain.resetOdometry(trajectory.getInitialPose())),ramseteCommand);
    }else{
      return ramseteCommand;
    }
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons

// INTAKE 
//final JoystickButton intakeMovePower = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);        
//intakeMovePower.whileHeld(new IntakeMove( m_intake ) ,true);
    SmartDashboard.putData("IntakeMovePower",new IntakeConeIn( m_intake ) );

// CONVEYOR
//   Operator Controller
//final JoystickButton conveyorMovePower = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);        
                    //conveyorMovePower.whileHeld(new ConveyorMove( m_conveyor ) ,true);
//final JoystickButton conveyorMovePowerBackwards = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
                     //conveyorMovePowerBackwards.whileHeld(new ConveyorMoveBackwards( m_conveyor ) ,true);
//   Driver Controller

final JoystickButton driveTrainLockButton = new JoystickButton(driverController, XboxController.Button.kX.value);        
                     driveTrainLockButton.whileTrue(new DriveTrainLock(m_driveTrain));

//final JoystickButton conveyorMovePowerDriverBackwards = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
                     //conveyorMovePowerDriverBackwards.whileHeld(new ConveyorMoveBackwards( m_conveyor ) ,true);
    SmartDashboard.putData("ArmMovePower",new ArmControlExtend( m_arm ) );


//   Operator Controlller
final JoystickButton armHighButton = new JoystickButton(operatorController, XboxController.Button.kY.value);        
                     armHighButton.whileTrue(new ArmControlHighPosition( m_arm ));
final JoystickButton armMidButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
                     armMidButton.whileTrue(new ArmControlMidPosition( m_arm ));
final JoystickButton armStowButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
                     armStowButton.whileTrue(new ArmControlStowedPosition( m_arm ));

//   Driver Controller
//        same as Operator
//final JoystickButton shooterMovePowerShortDriver = new JoystickButton(driverController, XboxController.Button.kA.value);        
                     //shooterMovePowerShortDriver.whileHeld(new ShooterMoveLow( m_shooter ) ,true);
//        access without interupting driving, with free trigger Finger
//GrantRequest final JoystickButton shooterMovePowerShortDriverEasyAccess = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);        
//GrantRequest                     shooterMovePowerShortDriverEasyAccess.whileHeld(new ShooterMoveLow( m_shooter ) ,true);
//  Smartdashboard

// SHOOTER fieldPermiter Shoot
//   Operator Controlller
//final JoystickButton shooterMovePowerLong = new JoystickButton(operatorController, XboxController.Button.kB.value);        
                     //shooterMovePowerLong.whileHeld(new ShooterMoveHigh( m_shooter ) ,true);
//   Driver Controller
//        same as Operator
//final JoystickButton shooterMovePowerLongDriver = new JoystickButton(driverController, XboxController.Button.kB.value);        
                     //shooterMovePowerLongDriver.whileHeld(new ShooterMoveHigh( m_shooter ) ,true);
                     
//final JoystickButton driveTrainForward = new JoystickButton(driverController, XboxController.Button.kY.value);        
                     //driveTrainForward.whileHeld(new DriveTrainMoveForward( m_driveTrain ) ,true);
                     
//        access without interupting driving, with free trigger Finger
//GrantRequest final JoystickAxisAsButton shooterMovePowerLongDriverEasyAccess = new JoystickAxisAsButton(driverController, XboxController.Button.kRightStick.value);        
//GrantRequest                           shooterMovePowerLongDriverEasyAccess.whileHeld(new ShooterMoveHigh( m_shooter ) ,true);




    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public XboxController getDriverController() {
      return driverController;
    }

public XboxController getOperatorController() {
      return operatorController;
    }

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

}

