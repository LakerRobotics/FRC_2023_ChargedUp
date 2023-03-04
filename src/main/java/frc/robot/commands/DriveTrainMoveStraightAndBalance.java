package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelHelper;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import frc.robot.subsystems.utilities.AdjustSpeedBasedOnTiltHelper;
import frc.robot.subsystems.utilities.EncoderAvgLeftRight;
import frc.robot.subsystems.utilities.MotionControlPIDController;
import frc.robot.subsystems.utilities.PIDOutputStraightMotion;
import frc.robot.subsystems.utilities.TiltAngleAsDouble;

/**
 *
 */
public class DriveTrainMoveStraightAndBalance extends CommandBase {
    private final DriveTrain m_DriveTrain;

    RelativeEncoder m_leftEncoder; 
    RelativeEncoder m_rightEncoder; 
    Gyro m_rotationSource;

    protected EncoderAvgLeftRight m_LineSource;
    protected TiltAngleAsDouble m_tiltSource;
	protected Gyro m_TurnSource;
    private double m_maxspeed;
    private double m_targetAngle;
    
	private double m_StraightTolerance;
	private AdjustSpeedAsTravelHelper m_AdustsSpeedAsTilttHelper;
    private PIDOutputStraightMotion    m_StraightRotationPIDOutput;
	private MotionControlPIDController m_StraightDistancePIDController;

    private SimpleMotorFeedforward m_simpleMotorFeedForward;

    private boolean isStraightMovingForward = true;
    public final double StraightKp = 0.006;// 0.020;
    public final double StraightKi = 0.008;//0.001;
    public final double StraightKd = 0.0;
//    private final double StraightMaxPower = 1;
    private double convertTiltToSpeed = (1/38) // 38 is the tilt of the ramp to the charging station
                                            *3;//, 3 is Meters/sec so the max speed when at max tilt

/** 
    * @param theDriveTrain the drivetrain subsystem
    * @param distanceSource the wheel encoders
    * @param rotationSource the Gryro
    * @param distance  to travel in inches
    * @param maxSpeed  in ft/sec
    * @param ramp      in inches
    * @param targetAngle assume it want Degrees
    ------------------------------------------------*/
   public DriveTrainMoveStraightAndBalance(DriveTrain theDriveTrain, double distance, double maxspeed,  double targetAngle){

        m_DriveTrain = theDriveTrain;
        addRequirements(m_DriveTrain);
        m_leftEncoder    = theDriveTrain.getEncoderLeft();
        m_rightEncoder   = theDriveTrain.getEncoderRight();
        m_rotationSource = theDriveTrain.getGyro();
     
        m_LineSource = new EncoderAvgLeftRight(m_leftEncoder, m_rightEncoder);
        m_tiltSource = new TiltAngleAsDouble(theDriveTrain.getIMU());
        m_TurnSource = m_rotationSource;
        m_maxspeed = maxspeed;
        m_targetAngle = targetAngle;

        m_StraightTolerance = 2;
        
    }
     


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
	{
			
//		if (!(Math.abs(m_LineSource.getDistance()) > Math.abs(m_DistanceToExceed)))
//		{
        // TURN POWER to drive straight, Setup PID to constantly adjust the turn to follow the gyro
		m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);

        // FORWARD POWER, will have two parts, a guess of the motor power needed plus PID control to try and get to actaul speed requesting
        // so 20% min power to move (deadzone)
        // assuming max speed robot is 13 ft/sec which 156 in/sec need to get to 100% that calculcat e0.0059rr for the second number, but it was too low
        m_simpleMotorFeedForward = new SimpleMotorFeedforward(0.20, 0.008);//0.005944);

        //Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
        m_AdustsSpeedAsTilttHelper = new AdjustSpeedBasedOnTiltHelper(
                                                       convertTiltToSpeed,
                                                       3, //considered balance if within 3 degrees and set speed to 0
                                                        m_tiltSource/*m_StraightRotationPIDOutput*/);// Not needed to be passed in, this is done here
                                                        
		m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTilttHelper);
		m_StraightDistancePIDController.setTolerance(m_StraightTolerance);// there is also a setTolerance that takes position and velocity acceptable tolerance
//		}
    }
}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


//        double distanceSoFar = m_LineSource.getDistance();
//        double targetSpeed   = m_AdustsSpeedAsTravelStraightHelper.getTargetSpeed(distanceSoFar);
        double tiltAngle   = m_DriveTrain.getIMU().
                             getXComplementaryAngle(); //this seems to be what we need but, this will be impacted by acceloration of the robot so we should move slow
                                                      // relate posts: https://www.chiefdelphi.com/t/analog-devices-adis-16470-eng-basic-java-gyro-code-help/422553/17
                                                      // potential replacement code for th IMU, but does not implement gyro so suld need to be adapted to simply plug in as gyro
                                                      //   https://github.com/Greater-Rochester-Robotics/SwerveBase2023/blob/main/src/main/java/frc/robot/subsystems/ADIS16470_IMU.java
                                                      //   described: https://www.chiefdelphi.com/t/reading-the-pitch-rate-of-the-16470-imu/425010/6
                                     
        double targetSpeed = m_AdustsSpeedAsTilttHelper.getTargetSpeed(tiltAngle);
 
        double currentSpeed  = m_LineSource.getRate();// gets current speed of the robot

        double firstGuessAtMotorPower = m_simpleMotorFeedForward.calculate(targetSpeed);
        double pidTuneForwardPower = m_StraightDistancePIDController.calculate(currentSpeed, targetSpeed);
        double forwardPower = firstGuessAtMotorPower + pidTuneForwardPower;
    
        double angleRightNow = m_TurnSource.getAngle();
        double targetAngle = m_targetAngle;
        double turnPower = m_StraightRotationPIDOutput.calculate(angleRightNow,targetAngle);

        m_DriveTrain.arcadeDrive(forwardPower, turnPower);

        SmartDashboard.putNumber("DriveStraight CurrentTiltAngle", targetSpeed );
        SmartDashboard.putNumber("DriveStraight targetSpeed", targetSpeed);
        SmartDashboard.putNumber("DriveStraight currentSpeed", currentSpeed);
        SmartDashboard.putNumber("DriveStraight forwardPower", forwardPower);
        SmartDashboard.putNumber("DriveStraight angleRightNow", angleRightNow);
        SmartDashboard.putNumber("DriveStraight turnPower", turnPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//never finished keep balancing till time runs out
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
