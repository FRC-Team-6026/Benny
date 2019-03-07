package frc.robot;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.analog.adis16448.frc.ADIS16448_IMU;

/**
 * This class is just to separate the Drivetrain (motors that move the wheels) code from the
 * rest of the robot code. It is good practice to break code into small chunks that do one
 * thing and do that one thing well. If we need to change how the robot drives we should
 * only need to change this class.
 */
public class Drivetrain{
    /**
     * These are private variables used by this class. The final means that they are initialized
     * either directly here where they are defined, or in the constructor. The preceding underscore
     * is just a convention I am used to from C# that communicates that the variable is a class variable
     * and not a local variable or a parameter.
     */

     //There are 4 motors driving the drive train, and they are combined into two SpeedControllerGroups
     //The controllers are initialized with new WPI_TalonSRX(#) where the number is the CAN ID. The CAN
     //Id's can be viewed and set from the Pheonix Tuner software.
    private final WPI_TalonSRX _leftFront = new WPI_TalonSRX(3);
    private final WPI_TalonSRX _leftRear = new WPI_TalonSRX(4);
    private final SpeedControllerGroup _left = new SpeedControllerGroup(_leftFront, _leftRear);
    private final WPI_TalonSRX _rightFront = new WPI_TalonSRX(1);
    private final WPI_TalonSRX _rightRear = new WPI_TalonSRX(2);
    private final SpeedControllerGroup _right = new SpeedControllerGroup(_rightFront, _rightRear);
    private final ADIS16448_IMU _imu;

    //The differential drive is a class from WPI and is exactly that. A way to drive a robot with a
    //motor on each side. It takes in the two speed controller groups created above.
    private final DifferentialDrive _drive = new DifferentialDrive(_left, _right);

    /**
     * This is the contructor to create a drivetrain object.
     */
    public Drivetrain(ADIS16448_IMU imu){
        _imu = imu;
    }

    public void initialize(){
        _leftFront.configFactoryDefault();
        _leftRear.configFactoryDefault();
        _rightFront.configFactoryDefault();
        _rightRear.configFactoryDefault();

        _leftFront.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _leftRear.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _rightFront.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
        _rightRear.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);

        _leftFront.setInverted(false);
        _leftRear.setInverted(false);
        _rightFront.setInverted(false);
        _rightRear.setInverted(false);

        _commandedHeading = _imu.getAngleZ();
    }

    /**
     * We are exposing just one public (other people outside of this class can use it) method
     * and this method just drives the robot using the arcade drive method (single stick control)
     * The other option would be tank drive (one stick per side so two stick control)
     */
    public void arcadeDrive(double speed, double rotation){
        _drive.arcadeDrive(speed, rotation*0.5);
    }
}