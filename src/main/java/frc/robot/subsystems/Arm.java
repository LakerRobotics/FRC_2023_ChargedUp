// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
//import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANSparkMax arm;
    private AbsoluteEncoder encoder;
    private SparkMaxPIDController pidController;
    private final double kP = 0.01;
    private final double kI = 0.0001;
    private final double kD = 0.0;
    private final double kFF = 0.0;
    private final double kMaxOutput = 1.0;
    private final double kMinOutput = -1.0;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public Arm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        arm = new CANSparkMax(15, MotorType.kBrushless);
        encoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = arm.getPIDController();

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    public void movePower(double power) {
        arm.set(power);
    }

    public void setPosition(double targetPosition) {
        pidController.setReference(targetPosition, ControlType.kPosition);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the SmartDashboard with the current arm position
        SmartDashboard.putNumber("ArmPosition", getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }
}