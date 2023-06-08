package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class SwerveModule {
  
    private final CANSparkMax driveMotor;
    private final VictorSP turningMoVictorSP;

    private final RelativeEncoder driveEncoder;
    private final Encoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule (int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId)
    
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    
    
    }



public double getDrivePosition() {
    return driveEncoder.getPosition();
}     

public double getTurningPosition() {
    return turningEncoder.getPosition();
}

public double getDriveVelocity() {
    return turningEncoder.getVelocity();
}
   
public double getTurningVelocity() {
    return turningEncoder.getVelocity();
}

public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

}

public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
}

}


