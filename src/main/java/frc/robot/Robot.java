/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
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
   * custom drivetrain (see Drivetrain.java). All the drivetrain needs is an
   * XBox controller.
   */
  private final XboxController _controller = new XboxController(0);
  private final Drivetrain _drivetrain = new Drivetrain(_controller);
  private final Stilts _stilts = new Stilts();

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
     */
    _drivetrain.arcadeDrive();

    var rightBumperDown = _controller.getBumper(Hand.kRight);
    var leftBumperDown = _controller.getBumper(Hand.kLeft);
    var rightY = _controller.getY(Hand.kRight);
    var rightStickDown = rightY < -0.2;
    var rightStickUp = rightY > 0.2;

    if (rightStickUp){
      if (rightBumperDown && leftBumperDown){
        _stilts.raise();
      } else if (rightBumperDown) {
        _stilts.raiseFront();
      } else if (leftBumperDown) {
        _stilts.raiseRear();
      } else {
        _stilts.stop();
      }
    } else if (rightStickDown) {
      if (rightBumperDown && leftBumperDown){
        _stilts.lower();
      } else if (rightBumperDown) {
        _stilts.lowerFront();
      } else if (leftBumperDown) {
        _stilts.lowerRear();
      } else {
        _stilts.stop();
      }
    } else {
      _stilts.stop();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
