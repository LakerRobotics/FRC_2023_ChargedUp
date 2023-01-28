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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Intake;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class IntakeControlPower extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final Intake m_intake;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public IntakeControlPower(Intake subsystem) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_intake = subsystem;
        addRequirements(m_intake);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // set Power to the maximum specified by the operator controller or the driver controller
        double intakePower = 0;
        double operatorSpecifiedIntakePower = RobotContainer.getInstance().getOperatorController().getLeftX();
        double driverSpecifiedIntakePower = RobotContainer.getInstance().getDriverController().getLeftTriggerAxis();
        double driverSpecifiedIntakeReversePower = RobotContainer.getInstance().getDriverController().getRightTriggerAxis();
        
        if( (operatorSpecifiedIntakePower==0)
         && (driverSpecifiedIntakePower == 0)
         && (driverSpecifiedIntakeReversePower==0)){
            intakePower=0;
        }
        else{
            if (java.lang.Math.abs(operatorSpecifiedIntakePower)>(java.lang.Math.abs(driverSpecifiedIntakePower))){
                intakePower = operatorSpecifiedIntakePower;
            }
            else{
                if(driverSpecifiedIntakePower>0){
                    intakePower = driverSpecifiedIntakePower;
                }else{
                    if(driverSpecifiedIntakeReversePower>0){
                        intakePower = - driverSpecifiedIntakeReversePower;
                    }
                }

            }
        }
        //temp TODO fix code above
        //intakePower=driverSpecifiedIntakePower;
        
        m_intake.movePower(intakePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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
