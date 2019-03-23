package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is just to separate the stilts (motors that move the lifting stilts) code from the
 * rest of the robot code. It is good practice to break code into small chunks that do one
 * thing and do that one thing well. If we need to change how the robot rises and lowers we should
 * only need to change this class.
 */
public class Stilts{
    /**
     * These are private variables used by this class. The final means that they are initialized
     * either directly here where they are defined, or in the constructor. The preceding underscore
     * is just a convention I am used to from C# that communicates that the variable is a class variable
     * and not a local variable or a parameter.
     */

     //There are 2 motors driving the stilts, and they are combined into one SpeedControllerGroup
     //The controllers are initialized with new WPI_TalonSRX(#) where the number is the CAN ID. The CAN
     //Id's can be viewed and set from the Pheonix Tuner software.
    private final WPI_TalonSRX _frontLegs = new WPI_TalonSRX(6);
    private final WPI_TalonSRX _rearLegs = new WPI_TalonSRX(5);
	private final WPI_TalonSRX _driveWheelRight = new WPI_TalonSRX(8);
	private final WPI_TalonSRX _driveWheelLeft = new WPI_TalonSRX(9);

    private final int _positionSlot = 0;
    private final int _pidPosition = 0;
    private final int _timeoutMs = 30;
    private final Gains _positionGains = new Gains(0.2,0,0,0.2,100,0.5);
    public final static double _neutralDeadband = 0.001;
	private final double _topPosition = 10000; //Need to set based on testing
	private double _rearTarget = 0;
	private double _frontTarget = 0;

    /**
     * This is the contructor to create a stilts object.
     */
    public Stilts(){
    }

    public void initialize(){
		SmartDashboard.putNumber("Top position for stilts", _topPosition);
        /* Factory Default all hardware to prevent unexpected behaviour */
		_rearLegs.configFactoryDefault();
		_frontLegs.configFactoryDefault();
		_driveWheelRight.configFactoryDefault();
		_driveWheelLeft.configFactoryDefault();

		_driveWheelRight.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
		_driveWheelLeft.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
		
		/* Set Neutral Mode */
		_frontLegs.setNeutralMode(NeutralMode.Brake);
		_rearLegs.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		_frontLegs.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative,	// Local Feedback Source
													_pidPosition,					// PID Slot for Source [0, 1]
													_timeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rearLegs.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,		// Local feedback source
												_pidPosition,	// PID Slot for source
												_timeoutMs);	// Configuration Timeout
		
		/* Configure output and sensor direction */
		_frontLegs.setInverted(true);
		_frontLegs.setSensorPhase(false);
		_rearLegs.setInverted(true);
		_rearLegs.setSensorPhase(true);

		/* Configure neutral deadband */
		_rearLegs.configNeutralDeadband(_neutralDeadband, _timeoutMs);
		_frontLegs.configNeutralDeadband(_neutralDeadband, _timeoutMs);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_rearLegs.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, _timeoutMs);
        _rearLegs.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, _timeoutMs);
		
		/* Motion Magic Configurations */
		_rearLegs.configMotionAcceleration(3000, _timeoutMs);
		_rearLegs.configMotionCruiseVelocity(10000, _timeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_frontLegs.configPeakOutputForward(+1.0, _timeoutMs);
		_frontLegs.configPeakOutputReverse(-1.0, _timeoutMs);
		_rearLegs.configPeakOutputForward(+1.0, _timeoutMs);
		_rearLegs.configPeakOutputReverse(-1.0, _timeoutMs);

		/* FPID Gains for distance servo */
		_rearLegs.config_kP(_positionSlot, _positionGains.P, _timeoutMs);
		_rearLegs.config_kI(_positionSlot, _positionGains.I, _timeoutMs);
		_rearLegs.config_kD(_positionSlot, _positionGains.D, _timeoutMs);
		_rearLegs.config_kF(_positionSlot, _positionGains.F, _timeoutMs);
		_rearLegs.config_IntegralZone(_positionSlot, _positionGains.IZone, _timeoutMs);
		_rearLegs.configClosedLoopPeakOutput(_positionSlot, _positionGains.PeakOutput, _timeoutMs);
		_rearLegs.configAllowableClosedloopError(_positionSlot, 0, _timeoutMs);

		/* FPID Gains for distance servo */
		_frontLegs.config_kP(_positionSlot, _positionGains.P, _timeoutMs);
		_frontLegs.config_kI(_positionSlot, _positionGains.I, _timeoutMs);
		_frontLegs.config_kD(_positionSlot, _positionGains.D, _timeoutMs);
		_frontLegs.config_kF(_positionSlot, _positionGains.F, _timeoutMs);
		_frontLegs.config_IntegralZone(_positionSlot, _positionGains.IZone, _timeoutMs);
		_frontLegs.configClosedLoopPeakOutput(_positionSlot, _positionGains.PeakOutput, _timeoutMs);
		_frontLegs.configAllowableClosedloopError(_positionSlot, 0, _timeoutMs);

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rearLegs.configClosedLoopPeriod(0, closedLoopTimeMs, _timeoutMs);
		_rearLegs.configClosedLoopPeriod(1, closedLoopTimeMs, _timeoutMs);

		zeroSensors();

		/* Determine which slot affects which PID */
		_rearLegs.selectProfileSlot(_positionSlot, _pidPosition);
		_frontLegs.selectProfileSlot(_positionSlot, _pidPosition);
	}
	
	public void smartDashboardDisplay(){
		var rearPosition = _rearLegs.getSensorCollection().getQuadraturePosition();
		var frontPosition = _frontLegs.getSensorCollection().getQuadraturePosition();
		SmartDashboard.putNumber("rear encoder", rearPosition);
		SmartDashboard.putNumber("front encoder", frontPosition);
		SmartDashboard.putNumber("rear target", _rearTarget);
		SmartDashboard.putNumber("front target", _frontTarget);
	}

    public void driveRearLegsEncoder(double output){
		_rearTarget = output * 20;
		_rearLegs.set(ControlMode.MotionMagic, _rearTarget);
	}

	public void driveFrontLegsEncoder(double output){
		_frontTarget = output * 20;
		_frontLegs.set(ControlMode.MotionMagic, _frontTarget);
	}

	public void driveRearLegs(double output){
		_rearLegs.set(ControlMode.PercentOutput, output);
	}

	public void driveFrontLegs(double output){
		_frontLegs.set(ControlMode.PercentOutput, output);
	}

    public void driveWheel(double output){
		_driveWheelRight.set(ControlMode.PercentOutput, output);
		_driveWheelLeft.set(ControlMode.PercentOutput, output);
    }

    /** Zero quadrature encoders on Talon */
	public void zeroSensors() {
		_frontLegs.getSensorCollection().setQuadraturePosition(0, _timeoutMs);
		_rearLegs.getSensorCollection().setQuadraturePosition(0, _timeoutMs);
		System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
	}
}