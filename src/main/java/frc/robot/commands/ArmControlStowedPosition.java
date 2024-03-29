// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Arm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class ArmControlStowedPosition extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final Arm m_Arm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public ArmControlStowedPosition(Arm subsystem) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_Arm = subsystem;
        addRequirements(m_Arm);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //m_shooter.move(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double retractedPosition=0.4;
        m_Arm.setPosition(retractedPosition);
        if(java.lang.Math.abs((m_Arm.getPosition()-retractedPosition)/retractedPosition) < 0.1){
            // Rumble power is 100% subtract 10% for every 1% off of the target position, so will be at max rumbel when at position
            double rumblePower = 1-10*java.lang.Math.abs((m_Arm.getPosition()-retractedPosition)/retractedPosition);
            RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble,rumblePower);
            RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kLeftRumble,rumblePower);
        }
        else{
            RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble, 0.0);
            RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kLeftRumble, 0.0);
        }
        SmartDashboard.putNumber("Arm Current Position", m_Arm.getPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       // m_Arm.move(0);
        RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble, 0.0);
        RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kLeftRumble, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
