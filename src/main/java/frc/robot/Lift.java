package frc.robot;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
     
     /**
     * This is the contructor to create a drivetrain object.
     */
    public Lift(){}

    public void initialize(){
        _liftMotor.configFactoryDefault();

        _liftMotor.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);

        //Not sure if needed yet. Un-comment if motors are going the wrong way.
        // _liftMotor. setInverted(true);
    }

    public void liftControl(double input){
        _liftMotor.set(ControlMode.PercentOutput,input);
    }

}