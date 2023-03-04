package frc.robot.subsystems.utilities;

//import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * 
 * @author richard.topolewski
 *
 */
public class AdjustSpeedBasedOnTiltHelper extends AdjustSpeedAsTravelHelper{
    private double m_conversionTiltToSpeed;
    private double m_consideredBalancedIfWithinThisManyDegrees; 
	   	
	/**
     * This helper class just takes a tilt, and will provide a speed (either up or down)  to get to that end distance and be traveling at the end speed
     
     * @param converstionTiltToSpeed this will be multiplied by tilt to give the target speed
     * @param source           The source information about, i.e. the encoder we are going to be monitoring
     * @param output           The output, i.e. the motor power we are going to be controlling to achieve the start-end speed profile desired
     */
    public AdjustSpeedBasedOnTiltHelper(double conversionTiltToSpeed, 
                                    double consideredBalancedIfWithinThisManyDegrees, 
    		                       DoubleSupplier/*PIDSource*/ source//, DoubleConsumer/*PIDOutput*/ output
                                   ){
         m_conversionTiltToSpeed = conversionTiltToSpeed;
         m_consideredBalancedIfWithinThisManyDegrees = consideredBalancedIfWithinThisManyDegrees; 
    }


	/**
     * Given the currentPosition (i.e. distance) this will return the current target speed 
     * @param currentMeasuredDistance // the current position, in distance units for example inches or Degrees-of-rotation
     * @return
     */
    public double getTargetSpeed(double currentMeasuredTilt){
    	
        if (currentMeasuredTilt < m_consideredBalancedIfWithinThisManyDegrees) currentMeasuredTilt = 0;       

    	double targetSpeed = currentMeasuredTilt*m_conversionTiltToSpeed;
        
    	return targetSpeed;
    }
    

 

}
