package frc.robot.subsystems.utilities;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionControlPIDController extends PIDController {
	AdjustSpeedAsTravelHelper m_motionControlHelper; 
	/**
	 * 
	 * @return
	 * @override
	 * @throws Exception
	 */
/*	public double getRate() throws Exception
	{
// This looks like a recusive loop which is bad so 		SmartDashboard.putNumber("Motion Control Rate", this.getRate());
		SmartDashboard.putNumber("MotionControlPIDController Rate", m_motionControlHelper.getM_source().getAsDouble());
		return m_motionControlHelper.getM_source().getAsDouble();
	}
*/	

	public MotionControlPIDController(double Kp, double Ki, double Kd, AdjustSpeedAsTravelHelper motionControl) 
	{
		//super(Kp, Ki, Kd, motionControl.getM_source(), motionControl.getM_output());
		super(Kp, Ki, Kd);
		//TODO code needs to call calculate(motionControl.getM_source(), motionControl.getM_output());
		m_motionControlHelper = motionControl;
		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
	}
	
}
