/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String _autoSelected;
  private final SendableChooser<String> _chooser = new SendableChooser<>();

  /**
   * Initializing a xbox controller to use for the robot and setting up our
   * subsystems
   */
  private final XboxController _controller = new XboxController(0);
  private final ADIS16448_IMU _imu = new ADIS16448_IMU();
  private final Drivetrain _drivetrain = new Drivetrain();
  private final Stilts _stilts = new Stilts(_imu);
  private final Lift _Lift = new Lift();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    _chooser.setDefaultOption("Default Auto", kDefaultAuto);
    _chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", _chooser);
    _drivetrain.initialize();
    _imu.calibrate();
    _stilts.initialize();
    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (_controller.getStickButtonPressed(Hand.kRight)){
      _imu.calibrate();
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    _autoSelected = _chooser.getSelected();
    // _autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + _autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    /**
     * When TeleOP mode is enabled this block will be called in a loop
     */

    /**
     * Here we are just calling our arcade drive that is setup in our
     * drivetrain (see Drivetrain.java) This will drive the robot with
     * The left stick of the XBox controller using the Y axis to move
     * forward and back and the X axis to turn left and right.
     * We are inverting the x to get the correct turn direction.
     */
    _drivetrain.arcadeDrive(_controller.getY(Hand.kLeft), -_controller.getX(Hand.kLeft));
    _Lift.liftControl(_controller.getY(Hand.kRight));

    if (_controller.getBumperPressed(Hand.kRight)){
      _stilts.raise();
    } else if (_controller.getBumperPressed(Hand.kLeft)) {
      _stilts.lower();
    } else if (_controller.getYButtonPressed()){
      _stilts.hover();
    } else if (_controller.getXButtonPressed()){
      _stilts.stop();
    }

    _stilts.periodic();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
