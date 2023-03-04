package frc.robot.subsystems.utilities;

import java.util.function.DoubleSupplier;

import com.analog.adis16470.frc.ADIS16470_IMU;


public class TiltAngleAsDouble implements DoubleSupplier{
    
    ADIS16470_IMU m_IMU;

    public TiltAngleAsDouble(ADIS16470_IMU theIMU){
        m_IMU = theIMU;
    }
    

    @Override
    public double getAsDouble() {
        return m_IMU.getXComplementaryAngle();
    }
}
