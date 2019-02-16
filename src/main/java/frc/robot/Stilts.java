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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

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
    private final ADIS16448_IMU _imu;
    private StiltState _stiltState = StiltState.Stopped;
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

        _frontLegs.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _rearLegs.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
    }

    public void periodic(){
        var output = 0.0;
        var frontOutput = 0.0;
        var rearOutput = 0.0;
        var pitchDiff = _commandPitchAngle - _imu.getPitch();
        var pitchComponent = Math.min(1, pitchDiff / 5.0) * 0.1;
        switch(_stiltState){
            case Rising:
            output = -.2;
            frontOutput = output - pitchComponent;
            rearOutput = output + pitchComponent + -.1;
            _frontLegs.set(ControlMode.PercentOutput, frontOutput);
            _rearLegs.set(ControlMode.PercentOutput, rearOutput);
            break;
            case Lowering:
            output = .1;
            frontOutput = output - pitchComponent;
            rearOutput = output + pitchComponent;
            _frontLegs.set(ControlMode.PercentOutput, frontOutput);
            _rearLegs.set(ControlMode.PercentOutput, rearOutput);
            break;
            case Stopped:
            _frontLegs.set(ControlMode.PercentOutput, 0);
            _rearLegs.set(ControlMode.PercentOutput, 0);
            break;
        }
    }

    public void raise(){
        _stiltState = StiltState.Rising;
    }

    public void lower(){
        _stiltState = StiltState.Lowering;
    }

    public void stop(){
        _stiltState = StiltState.Stopped;
    }

    public void setPitchAngle(double angle){
        _commandPitchAngle = angle;
    }

    public void raiseFront(){
        _frontLegs.set(ControlMode.PercentOutput, 0.2);
    }

    public void lowerFront(){
        _frontLegs.set(ControlMode.PercentOutput, -0.2);
    }

    public void raiseRear(){
        _rearLegs.set(ControlMode.PercentOutput, 0.2);
    }

    public void lowerRear(){
        _rearLegs.set(ControlMode.PercentOutput, -0.2);
    }

    private enum StiltState{
        Stopped,
        Rising,
        Lowering
    }
}