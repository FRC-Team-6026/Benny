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
  private final Drivetrain _drivetrain = new Drivetrain(_imu);
  private final Stilts _stilts = new Stilts();
  private final Lift _lift = new Lift();
  private final Compressor _compressor = new Compressor(10);
  private final DoubleSolenoid _gripSolenoid = new DoubleSolenoid(10,0,1);
  private final DoubleSolenoid _ballHolder = new DoubleSolenoid(10,2,3);
  private boolean _driveCameraSelected = true;
  private UsbCamera _driveCamera;
  private UsbCamera _targetCamera;
  private VideoSink _cameraServer;
  private boolean _isStiltMode = false;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    _imu.calibrate();
    _drivetrain.initialize();
    _stilts.initialize();
    _driveCamera = CameraServer.getInstance().startAutomaticCapture("driver", 0);
    _targetCamera = CameraServer.getInstance().startAutomaticCapture("target", 1);
    _cameraServer = CameraServer.getInstance().getServer();
    _driveCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    _targetCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    _cameraServer.setSource(_driveCamera);
    _compressor.setClosedLoopControl(true);
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
    SmartDashboard.putBoolean("Stilt mode", _isStiltMode);
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    driverPeriodic();
    operatorPeriodic();
  }

  private void driverPeriodic(){
    _drivetrain.arcadeDrive(-_driverControl.getY(Hand.kLeft), _driverControl.getX(Hand.kRight));

    if (_driverControl.getStickButton(Hand.kLeft) && _driverControl.getStickButtonPressed(Hand.kRight)){
      _isStiltMode = !_isStiltMode;
    }

    if(_isStiltMode){
      if (_driverControl.getBumperPressed(Hand.kLeft)){
        _stilts.liftBothLegsToZero();
      } else if (_driverControl.getBumperPressed(Hand.kRight)){
        _stilts.moveToTopPosition();
      } else if (_driverControl.getAButtonPressed()){
        _stilts.liftRearLegsStopFront();
      }
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
    //Lift control
    if (_operatorControl.getBButtonPressed()){
      _lift.goToHatchPosition();
    } else if (_operatorControl.getYButtonPressed()){
      _lift.goToTopPosition();
    } else if (_operatorControl.getAButtonPressed()){
      _lift.goToBottomPosition();
    }

    grabberControl();
  }

  private void grabberControl()
  {
    //Grabber control
    if (_operatorControl.getBumperPressed(Hand.kRight)){
      _gripSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (_operatorControl.getBumperPressed(Hand.kLeft)){
      _gripSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (_driverControl.getStickButton(Hand.kLeft) && _driverControl.getStickButtonPressed(Hand.kRight)){
      _isStiltMode = !_isStiltMode;
    }

    if(_isStiltMode){
      _stilts.driveRearLegs(-_driverControl.getY(Hand.kLeft));
      _stilts.driveFrontLegs(-_driverControl.getY(Hand.kRight));
    } else {
      _drivetrain.arcadeDrive(-_driverControl.getY(Hand.kLeft), _driverControl.getX(Hand.kRight));
    }

    _lift.liftManualControl(-_operatorControl.getY(Hand.kLeft));

    grabberControl();
  }
}
