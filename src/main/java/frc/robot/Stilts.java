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
    private final int _differenceSlot = 1;
    private final int _pidPosition = 0;
    private final int _pidDifference = 1;
    private final int _remote0 = 0;
    private final int _timeoutMs = 30;
    private final Gains _positionGains = new Gains(0.2,0,0,0.2,100,0.5);
    private final Gains _differenceGains = new Gains(1.0,0,2.0,0,200,1.0);
    public final static double _neutralDeadband = 0.001;
	private final double _topPosition = 10000; //Need to set based on testing

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
		_frontLegs.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													_pidPosition,					// PID Slot for Source [0, 1]
													_timeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rearLegs.configRemoteFeedbackFilter(_frontLegs.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												_remote0,							// Source number [0, 1]
												_timeoutMs);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		_rearLegs.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, _timeoutMs);				// Feedback Device of Remote Talon
		_rearLegs.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, _timeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		_rearLegs.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, _timeoutMs);
		_rearLegs.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, _timeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		_rearLegs.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													_pidPosition,
													_timeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		_rearLegs.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														_pidPosition,		// PID Slot of Source 
														_timeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		_rearLegs.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													_pidDifference, 
													_timeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		_rearLegs.configSelectedFeedbackCoefficient(	1,
														_pidDifference, 
														_timeoutMs);
		/* Configure output and sensor direction */
		_frontLegs.setInverted(false);
		_frontLegs.setSensorPhase(true);
		_rearLegs.setInverted(true);
		_rearLegs.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		_rearLegs.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, _timeoutMs);
		_rearLegs.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, _timeoutMs);
		_rearLegs.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, _timeoutMs);
		_rearLegs.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, _timeoutMs);
		_frontLegs.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, _timeoutMs);

		/* Configure neutral deadband */
		_rearLegs.configNeutralDeadband(_neutralDeadband, _timeoutMs);
		_frontLegs.configNeutralDeadband(_neutralDeadband, _timeoutMs);
		
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

		/* FPID Gains for turn servo */
		_rearLegs.config_kP(_differenceSlot, _differenceGains.P, _timeoutMs);
		_rearLegs.config_kI(_differenceSlot, _differenceGains.I, _timeoutMs);
		_rearLegs.config_kD(_differenceSlot, _differenceGains.D, _timeoutMs);
		_rearLegs.config_kF(_differenceSlot, _differenceGains.F, _timeoutMs);
		_rearLegs.config_IntegralZone(_differenceSlot, (int)_differenceGains.IZone, _timeoutMs);
		_rearLegs.configClosedLoopPeakOutput(_differenceSlot, _differenceGains.PeakOutput, _timeoutMs);
		_rearLegs.configAllowableClosedloopError(_differenceSlot, 0, _timeoutMs);

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

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_rearLegs.configAuxPIDPolarity(false, _timeoutMs);

		/* Initialize */
		_rearLegs.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
		zeroSensors();

		/* Determine which slot affects which PID */
		_rearLegs.selectProfileSlot(_positionSlot, _pidPosition);
		_rearLegs.selectProfileSlot(_differenceSlot, _pidDifference);
	}
	
	public void smartDashboardDisplay(){
		var rearPosition = _rearLegs.getSensorCollection().getQuadraturePosition();
		var frontPosition = _frontLegs.getSensorCollection().getQuadraturePosition();
		SmartDashboard.putNumber("rear encoder", rearPosition);
		SmartDashboard.putNumber("front encoder", frontPosition);
	}

    public void driveRearLegs(double output){
		_rearLegs.set(ControlMode.PercentOutput, output);
	}

	public void driveFrontLegs(double output){
		_frontLegs.set(ControlMode.PercentOutput, output);
	}
	
	public void moveToTopPosition(){
		/* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
		_rearLegs.set(ControlMode.MotionMagic, _topPosition, DemandType.AuxPID, 0);
		_frontLegs.follow(_rearLegs, FollowerType.AuxOutput1);
	}

	public void liftRearLegsFrontToTop(){
		_rearLegs.set(ControlMode.PercentOutput, -.25);
		_frontLegs.set(ControlMode.MotionMagic, _topPosition);
	}

	public void liftBothLegsToZero(){
		_rearLegs.set(ControlMode.MotionMagic, 0, DemandType.AuxPID, 0);
		_frontLegs.follow(_rearLegs, FollowerType.AuxOutput1);
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