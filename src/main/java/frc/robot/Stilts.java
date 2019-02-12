package frc.robot;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
    private final WPI_TalonSRX _frontLegs = new WPI_TalonSRX(5);
    private final WPI_TalonSRX _rearLegs = new WPI_TalonSRX(6);
    private final SpeedControllerGroup _allLegs = new SpeedControllerGroup(_frontLegs, _rearLegs);

    /**
     * This is the contructor to create a stilts object.
     */
    public Stilts(){}

    public void initialize(){
        _frontLegs.configFactoryDefault();
        _rearLegs.configFactoryDefault();

        _frontLegs.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _rearLegs.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
    }

    public void raise(){
        _allLegs.set(0.2);
    }

    public void lower(){
        _allLegs.set(-0.2);
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

    public void stop(){
        _allLegs.set(0);
    }
}