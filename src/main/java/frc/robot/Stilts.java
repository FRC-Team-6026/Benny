package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
    private final WPI_TalonSRX _driveWheel = new WPI_TalonSRX(8);
    private final ADIS16448_IMU _imu;
    private StiltState _frontStiltState = StiltState.FullStop;
    private StiltState _rearStiltState = StiltState.FullStop;
    private double _commandPitchAngle = 0;

    /**
     * This is the contructor to create a stilts object.
     */
    public Stilts(ADIS16448_IMU imu){
        _imu = imu;
    }

    public void initialize(){
        _frontLegs.configFactoryDefault();
        _rearLegs.configFactoryDefault();
        _driveWheel.configFactoryDefault();

        _frontLegs.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _rearLegs.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _driveWheel.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);

        _rearLegs.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        _rearLegs.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    public void periodic(){
        var output = 0.0;
        var frontOutput = 0.0;
        var rearOutput = 0.0;
        var pitch = _imu.getPitch();
        var pitchDiff = _commandPitchAngle - pitch;
        var pitchComponent = Math.min(1, pitchDiff / 3.0) * 0.04;
        switch(_frontStiltState){
            case Rising:
            output = -.2;
            frontOutput = output - pitchComponent;
            _frontLegs.set(ControlMode.PercentOutput, frontOutput);
            break;
            case Lowering:
            output = .1;
            frontOutput = output - pitchComponent;
            _frontLegs.set(ControlMode.PercentOutput, frontOutput);
            break;
            case Hover:
            var zAccel = _imu.getAccelZ();
            var zComponent = (1 + (1/zAccel))* 0.1;
            output = -0.06;
            frontOutput = output - pitchComponent - zComponent;
            frontOutput = Math.min(frontOutput, .05);
            _frontLegs.set(ControlMode.PercentOutput, frontOutput);
            break;
            case ForceLower:
            _frontLegs.set(ControlMode.PercentOutput, .2);
            break;
            case FullStop:
            _frontLegs.set(ControlMode.PercentOutput, 0);
            break;
        }
        switch(_rearStiltState){
            case Rising:
            output = -.2;
            rearOutput = output + pitchComponent + -.1;
            _rearLegs.set(ControlMode.PercentOutput, rearOutput);
            var rearCurrent = _rearLegs.getOutputCurrent();
            //if the rear legs have hit the limit switch
            if (rearCurrent < 0.01){
                _rearStiltState = StiltState.Hover;
                _frontStiltState = StiltState.Hover;
            }
            break;
            case Lowering:
            output = .1;
            rearOutput = output + pitchComponent;
            _rearLegs.set(ControlMode.PercentOutput, rearOutput);
            break;
            case Hover:
            var zAccel = _imu.getAccelZ();
            var zComponent = (1 + (1/zAccel))* 0.1;
            output = -0.07;
            rearOutput = output + pitchComponent - zComponent;
            rearOutput = Math.min(rearOutput, .05);
            _rearLegs.set(ControlMode.PercentOutput, rearOutput);
            break;
            case ForceLower:
            _rearLegs.set(ControlMode.PercentOutput, 0.1);
            break;
            case FullStop:
            _rearLegs.set(ControlMode.PercentOutput, 0);
            break;
        }
        SmartDashboard.putNumber("Rear current", _rearLegs.getOutputCurrent());
        SmartDashboard.putNumber("Front current", _frontLegs.getOutputCurrent());
    }

    public void raise(){
        _frontStiltState = StiltState.Rising;
        _rearStiltState = StiltState.Rising;
    }

    public void lower(){
        _frontStiltState = StiltState.Lowering;
        _rearStiltState = StiltState.Lowering;
    }

    public void hover(){
        _frontStiltState = StiltState.Hover;
        _rearStiltState = StiltState.Hover;
    }

    public void stop(){
        _commandPitchAngle = 0;
        _frontStiltState = StiltState.FullStop;
    }

    public void setPitchAngle(double angle){
        _commandPitchAngle = angle;
    }

    public void forceLowerRearLegs(){
        _rearStiltState = StiltState.ForceLower;
        _frontStiltState = StiltState.Hover;
    }

    public void forceLowerFrontLegs(){
        _frontStiltState = StiltState.ForceLower;
    }

    public void driveWheel(double output){
        if (Math.abs(output) < 0.015) output = 0;
        output = output * output * output;
        _driveWheel.set(ControlMode.PercentOutput, output);
    }

    private enum StiltState{
        FullStop,
        Rising,
        Lowering,
        ForceLower,
        Hover
    }
}