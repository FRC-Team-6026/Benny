package frc.robot;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is just to separate the Drivetrain (motors that move the wheels) code from the
 * rest of the robot code. It is good practice to break code into small chunks that do one
 * thing and do that one thing well. If we need to change how the robot drives we should
 * only need to change this class.
 */
public class Lift{
    /**
     * These are private variables used by this class. The final means that they are initialized
     * either directly here where they are defined, or in the constructor. The preceding underscore
     * is just a convention I am used to from C# that communicates that the variable is a class variable
     * and not a local variable or a parameter.
     */

     //There is 1 motor driving the drive train, and they are combined into a SpeedControllerGroup
     //The controller is initialized with new WPI_TalonSRX(#) where the number is the CAN ID. The CAN
     //Id's can be viewed and set from the Pheonix Tuner software.
     private final WPI_TalonSRX _liftMotor = new WPI_TalonSRX(7);
     private final int _pidIndex = 0;
     private final int _slotIndex = 0;
     private final int _timeoutMs = 30;
     private final Gains _liftGains = new Gains(0.2,0,0,0.2,0,1.0);
     private final double _bottomPosition = 0;
     private final double _hatchPosition = 1000;
     private final double _topPosition = 10000;
     
     /**
     * This is the contructor to create a drivetrain object.
     */
    public Lift(){}

    public void initialize(){
        _liftMotor.configFactoryDefault();
        _liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											_pidIndex, 
                                            _timeoutMs);
                      
        /**
		 * Configure Talon SRX Output and Sensor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_liftMotor.setSensorPhase(false);
        _liftMotor.setInverted(true);

        /* Set Neutral Mode */
		_liftMotor.setNeutralMode(NeutralMode.Brake);
    
        /* Set relevant frame periods to be at least as fast as periodic rate */
		_liftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, _timeoutMs);
        _liftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, _timeoutMs);
    
        /* Set the peak and nominal outputs */
		_liftMotor.configNominalOutputForward(0, _timeoutMs);
		_liftMotor.configNominalOutputReverse(0, _timeoutMs);
		_liftMotor.configPeakOutputForward(1, _timeoutMs);
        _liftMotor.configPeakOutputReverse(-1, _timeoutMs);
    
        /* Set Motion Magic gains in slot0 - see documentation */
		_liftMotor.selectProfileSlot(_slotIndex, _pidIndex);
		_liftMotor.config_kF(_slotIndex, _liftGains.F, _timeoutMs);
		_liftMotor.config_kP(_slotIndex, _liftGains.P, _timeoutMs);
		_liftMotor.config_kI(_slotIndex, _liftGains.I, _timeoutMs);
        _liftMotor.config_kD(_slotIndex, _liftGains.D, _timeoutMs);
        _liftMotor.config_IntegralZone(_slotIndex, _liftGains.IZone, _timeoutMs);
        _liftMotor.configClosedLoopPeakOutput(_slotIndex, _liftGains.PeakOutput, _timeoutMs);
    
        /* Set acceleration and vcruise velocity - see documentation */
		_liftMotor.configMotionCruiseVelocity(10000, _timeoutMs);
        _liftMotor.configMotionAcceleration(3000, _timeoutMs);
    
        zeroSensor();
    }

    public void liftManualControl(double output){
        _liftMotor.set(ControlMode.PercentOutput, output);
    }

    public void goToBottomPosition(){
        _liftMotor.set(ControlMode.MotionMagic, _bottomPosition);
    }

    public void goToHatchPosition(){
        _liftMotor.set(ControlMode.MotionMagic, _hatchPosition);
    }

    public void goToTopPosition(){
        _liftMotor.set(ControlMode.MotionMagic, _topPosition);
    }

    public void smartDashboardDisplay(){
        var liftPosition = _liftMotor.getSensorCollection().getQuadraturePosition();
        SmartDashboard.putNumber("Lift encoder", liftPosition);
    }

    public void zeroSensor(){
        /* Zero the sensor */
        _liftMotor.getSensorCollection().setQuadraturePosition(0,0);
    }
}