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
public class ArmControlMidPosition extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final Arm m_Arm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public ArmControlMidPosition(Arm subsystem) {


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
        double highPosition=0.65;
        m_Arm.setPosition(highPosition);
        if(java.lang.Math.abs((m_Arm.getPosition()-highPosition)/highPosition) < 0.1){
            // Rumble power is 66% subtract 6.6% for every 1% off of the target position, so will be at less then max rumbel when at position
            double rumblePower = 66-6.6*java.lang.Math.abs((m_Arm.getPosition()-highPosition)/highPosition);
            RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble,rumblePower);
            RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kRightRumble,rumblePower);
        }
        else{
            RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.0);
            RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kRightRumble, 0.0);
        }
        SmartDashboard.putNumber("Arm Current Position", m_Arm.getPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       // m_Arm.move(0);
        RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.0);
        RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kRightRumble, 0.0);
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
