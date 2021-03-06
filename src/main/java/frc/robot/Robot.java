/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * Initializing a xbox controller to use for the robot and setting up our
   * subsystems
   */
  private final XboxController _driverControl = new XboxController(0);
  private final XboxController _operatorControl = new XboxController(1);
  private final ADIS16448_IMU _imu = new ADIS16448_IMU();
  private final Drivetrain _drivetrain = new Drivetrain();
  private final Stilts _stilts = new Stilts();
  private final Lift _lift = new Lift();
  private final HatchGrabber _hatchGrabber = new HatchGrabber(_operatorControl);
  private boolean _driveCameraSelected = true;
  private UsbCamera _driveCamera;
  private UsbCamera _targetCamera;
  private VideoSink _cameraServer;
  private boolean _isSlowSpeedMode = false;
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    _imu.calibrate();
    _drivetrain.initialize();
    _stilts.initialize();
    _lift.initialize();
    _driveCamera = CameraServer.getInstance().startAutomaticCapture("driver", 0);
    _targetCamera = CameraServer.getInstance().startAutomaticCapture("target", 1);
    _cameraServer = CameraServer.getInstance().getServer();
    _driveCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    _targetCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    _cameraServer.setSource(_driveCamera);
    _hatchGrabber.initialize();
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
    SmartDashboard.putBoolean("Slow speed mode", _isSlowSpeedMode);
    SmartDashboard.putNumber("Heading", _imu.getAngleZ());
    _stilts.smartDashboardDisplay();
    _lift.smartDashboardDisplay();
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
    teleopInit(); //not using autonomous mode, just teleop
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic(); //not using autonomous mode, just teleop
  }

  @Override
  public void teleopInit() {
    _hatchGrabber.initializeGrip();
    _isSlowSpeedMode = true;
    clearButtons(_operatorControl);
    clearButtons(_driverControl);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    driverPeriodic();
    operatorPeriodic();
    //testPeriodic();
  }

  private void driverPeriodic(){
    var speed = _isSlowSpeedMode ? -_driverControl.getY(Hand.kLeft) * 0.6 : -_driverControl.getY(Hand.kLeft);
    var rotation = _isSlowSpeedMode ? _driverControl.getX(Hand.kRight) * 0.8 : _driverControl.getX(Hand.kRight);
    _drivetrain.arcadeDrive(speed, rotation);

    if (_driverControl.getTriggerAxis(Hand.kLeft) > 0.9) {
      _isSlowSpeedMode = true;
      _stilts.driveFrontLegsEncoder(-15000);
    } else {
      _stilts.driveFrontLegsEncoder(100);
    }

    if (_driverControl.getTriggerAxis(Hand.kRight) > 0.9) {
      _isSlowSpeedMode = true;
      _stilts.driveRearLegsEncoder(17000);
    } else {
      _stilts.driveRearLegsEncoder(-100);
    }

    var leftBumper = _driverControl.getBumper(Hand.kLeft);
    var rightBumper = _driverControl.getBumper(Hand.kRight);
    var leftBumperPressed = _driverControl.getBumperPressed(Hand.kLeft);
    var rightBumperPressed = _driverControl.getBumperPressed(Hand.kRight);
    if ((leftBumper && rightBumperPressed) || (rightBumper && leftBumperPressed)){
      _isSlowSpeedMode = !_isSlowSpeedMode;
    }

    if(_driverControl.getStartButtonPressed()){
      _driveCameraSelected = !_driveCameraSelected;
      if (_driveCameraSelected){
        _cameraServer.setSource(_driveCamera);
      } else{
        _cameraServer.setSource(_targetCamera);
      }
    }
  }

  private void operatorPeriodic(){
    //Lift auto control
    //if (_operatorControl.getY(Hand.kLeft) < -0.5){//up on left stick
    //  _lift.goToTopPosition();
    //} else if (_operatorControl.getY(Hand.kLeft) > 0.5){//down on left stick
    //  _lift.goToHatchPosition();
    //}
    _lift.liftManualControl(deadband(-_operatorControl.getY(Hand.kLeft))*0.5);

    if (_operatorControl.getStickButtonPressed(Hand.kLeft)){
      _lift.zeroSensor();
    }

    _hatchGrabber.periodic();
  }

  private double deadband(double input){
    if (Math.abs(input) < 0.05){
      return 0;
    } else {
      return input;
    }
  }

  private void clearButtons(XboxController controller) {
    controller.getAButtonPressed();
    controller.getAButtonReleased();
    controller.getBButtonPressed();
    controller.getBButtonReleased();
    controller.getBackButtonPressed();
    controller.getBackButtonReleased();
    controller.getBumperPressed(Hand.kLeft);
    controller.getBumperReleased(Hand.kLeft);
    controller.getBumperPressed(Hand.kRight);
    controller.getBumperReleased(Hand.kRight);
    controller.getStartButtonPressed();
    controller.getStartButtonReleased();
    controller.getStickButtonPressed(Hand.kLeft);
    controller.getStickButtonReleased(Hand.kRight);
    controller.getStickButtonPressed(Hand.kRight);
    controller.getStickButtonReleased(Hand.kRight);
    controller.getXButtonPressed();
    controller.getXButtonReleased();
    controller.getYButtonPressed();
    controller.getYButtonReleased();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    _drivetrain.arcadeDrive(-_driverControl.getY(Hand.kLeft), _driverControl.getX(Hand.kRight));

    var liftOutput = deadband(-_operatorControl.getY(Hand.kLeft));
    _lift.liftManualControl(liftOutput);

    _hatchGrabber.periodic();

    if(_driverControl.getStartButtonPressed()){
      _driveCameraSelected = !_driveCameraSelected;
      if (_driveCameraSelected){
        _cameraServer.setSource(_driveCamera);
      } else{
        _cameraServer.setSource(_targetCamera);
      }
    }
  }
}
