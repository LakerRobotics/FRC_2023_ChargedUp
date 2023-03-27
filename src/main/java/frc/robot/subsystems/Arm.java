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
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS\
    private double m_targetPosition;
    private CANSparkMax arm;
    private AbsoluteEncoder absEncoder;
    private SparkMaxPIDController pidController;
    private final double kP = -0.5;
    private final double kI = 0;
    private final double kD = 0.0;
    private final double kFF = 0.0;
    private final double kMaxOutput =  Constants.ArmConstants.ARM_OUTPUT_POWER_AUTON*0.5;
    private final double kMinOutput =  -Constants.ArmConstants.ARM_OUTPUT_POWER_AUTON*0.5;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public Arm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        arm = new CANSparkMax(15, MotorType.kBrushless);
        arm.setInverted(true);
        absEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = arm.getPIDController();
        pidController.setFeedbackDevice(absEncoder);

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
        m_targetPosition = targetPosition;
        pidController.setReference(targetPosition, ControlType.kPosition);
        SmartDashboard.putNumber("ArmTarget Position",m_targetPosition);
    }

    
    

    public double getPosition() {
        return absEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the SmartDashboard with the current arm position
        SmartDashboard.putNumber("ArmPosition", getPosition());
        SmartDashboard.putNumber("ArmTarget Power",arm.getAppliedOutput());
        SmartDashboard.putNumber("ArmTarget Position",m_targetPosition);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }
}