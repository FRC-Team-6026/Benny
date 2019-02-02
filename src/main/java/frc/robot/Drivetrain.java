package frc.robot;

/**
 * These are the namespaces that we are using in this class. For example the class type
 * and contructors used for the talon controllers WPI_TalonSRX are in the namespace
 * com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 */
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    private final WPI_TalonSRX _leftFront = new WPI_TalonSRX(0);
    private final WPI_TalonSRX _leftRear = new WPI_TalonSRX(1);
    private final SpeedControllerGroup _left = new SpeedControllerGroup(_leftFront, _leftRear);
    private final WPI_TalonSRX _rightFront = new WPI_TalonSRX(2);
    private final WPI_TalonSRX _rightRear = new WPI_TalonSRX(3);
    private final SpeedControllerGroup _right = new SpeedControllerGroup(_rightFront, _rightRear);

    //The differential drive is a class from WPI and is exactly that. A way to drive a robot with a
    //motor on each side. It takes in the two speed controller groups created above.
    private final DifferentialDrive _drive = new DifferentialDrive(_left, _right);

    //The controller will be initialized in the contstructor since we want to take in a controller
    //so only one controller object is used for all the Robot code.
    private final XboxController _controller;

    /**
     * This is the contructor to create a drive train object.
     * We have the XBox controller as a parameter because the controller is
     * going to be used in other parts of the robot code, so we only want
     * one controller being "passed around"
     */
    public Drivetrain(XboxController controller){
        //We don't want null things being passed in, so we are just verifying it is not null
        //and if it's not we are throwing an exception. If no one catches the exception, this
        //will crash the program, which is sometimes what we want.
        if (controller == null){
            throw new IllegalArgumentException("controller is null");
        }

        _controller = controller;
    }

    /**
     * We are exposing just one public (other people outside of this class can use it) method
     * and this method just drives the robot using the arcade drive method (single stick control)
     * The other option would be tank drive (one stick per side so two stick control)
     */
    public void arcadeDrive(){
        /**
         * for arcade drive to work it needs a Y stick position -1 to 1 and an X stick position -1 to 1
         * Here we are using the XBox controller Y position of the left stick and the X position of the
         * left stick.
         */
        _drive.arcadeDrive(_controller.getY(Hand.kLeft), _controller.getX(Hand.kLeft));
    }
}