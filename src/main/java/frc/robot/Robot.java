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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  /**
   * Initializing a xbox controller to use for the robot and setting up our
   * subsystems
   */
  private final XboxController _controller = new XboxController(0);
  private final ADIS16448_IMU _imu = new ADIS16448_IMU();
  private final Drivetrain _drivetrain = new Drivetrain(_imu);
  private final Stilts _stilts = new Stilts();
  private final Lift _lift = new Lift();
  private final Compressor _compressor = new Compressor(10);
  private final DoubleSolenoid _gripSolenoid = new DoubleSolenoid(10,0,1);
  private final DoubleSolenoid _ballHolder = new DoubleSolenoid(10,2,3);
  private UsbCamera _driveCamera;
  private UsbCamera _targetCamera;
  private SelectedCamera _selectedCamera;
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
    selectCamera(SelectedCamera.Driver);
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
    _drivetrain.arcadeDrive(_controller.getY(Hand.kLeft), -_controller.getX(Hand.kRight));

    if (_controller.getStickButton(Hand.kLeft) && _controller.getStickButtonPressed(Hand.kRight)){
      _isStiltMode = !_isStiltMode;
    }

    if(_isStiltMode){
      if (_controller.getBumperPressed(Hand.kLeft)){
        _stilts.liftBothLegsToZero();
      } else if (_controller.getBumperPressed(Hand.kRight)){
        _stilts.moveToTopPosition();
      } else if (_controller.getAButtonPressed()){
        _stilts.liftRearLegsStopFront();
      }
    } else {
      _lift.liftControl(_controller.getTriggerAxis(Hand.kRight) - _controller.getTriggerAxis(Hand.kLeft));

      if(_controller.getAButtonPressed()){
        _gripSolenoid.set(DoubleSolenoid.Value.kForward);
      }
      if(_controller.getYButtonPressed()){
        _gripSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
      if(_controller.getXButtonPressed()){
        _ballHolder.set(DoubleSolenoid.Value.kForward);
      }
      if(_controller.getBButtonPressed()){
        _ballHolder.set(DoubleSolenoid.Value.kReverse);
      }
    }

    if(_controller.getStickButtonPressed(Hand.kRight)){
      toggleCamera();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (_controller.getStickButton(Hand.kLeft) && _controller.getStickButtonPressed(Hand.kRight)){
      _isStiltMode = !_isStiltMode;
    }

    if(_isStiltMode){
      _stilts.driveRearLegs(_controller.getY(Hand.kLeft));
      _stilts.driveFrontLegs(_controller.getY(Hand.kRight));
    }
  }

  private void selectCamera(SelectedCamera camera){
    _selectedCamera = camera;
    if (_selectedCamera == SelectedCamera.Driver){
      SmartDashboard.putString("Camera Selection", _driveCamera.getName());
    } else if (_selectedCamera == SelectedCamera.Target){
      SmartDashboard.putString("Camera Selection", _targetCamera.getName());
    }
  }

  private void toggleCamera(){
    if (_selectedCamera == SelectedCamera.Driver){
      _selectedCamera = SelectedCamera.Target;
      SmartDashboard.putString("Camera Selection", _targetCamera.getName());
    } else if (_selectedCamera == SelectedCamera.Target){
      _selectedCamera = SelectedCamera.Driver;
      SmartDashboard.putString("Camera Selection", _driveCamera.getName());
    }
  }

  private enum SelectedCamera{
    Driver,
    Target
  }
}
