package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Pneumatics{
    private final Compressor _compressor = new Compressor(10);
    private final DoubleSolenoid _gripSolenoid = new DoubleSolenoid(10,2,3);
    private final DoubleSolenoid _ballHolder = new DoubleSolenoid(10,4,5);
    private final DoubleSolenoid _gripExtension = new DoubleSolenoid(10,0,1);
    private final int _cycleDelay = 20;
    private final XboxController _operatorControl;
    private int _solenoidCycleCount = 0;
    private boolean _retrievingHatch = false;
    private boolean _placingHatch = false;
    private boolean _extendingToGetHatch = false;
    private boolean _needToCloseGrabber = false;

    public Pneumatics(XboxController operatorControl) {
        _operatorControl = operatorControl;
    }

    public void initialize(){
        _compressor.setClosedLoopControl(true);
    }

    public void initializeGrip(){
        closeGrip();
        retractGrip();
    }

    public void periodic()
    {
        _solenoidCycleCount++;
    
        //manual controls
        if (_operatorControl.getYButtonPressed()){
          _placingHatch = false;
          _retrievingHatch = false;
          _needToCloseGrabber = false;
          _extendingToGetHatch = false;
          openGrip();
        }
    
        if (_operatorControl.getXButtonPressed()){
          _placingHatch = false;
          _retrievingHatch = false;
          _needToCloseGrabber = false;
          _extendingToGetHatch = false;
          closeGrip();
        }
    
        //placing hatch
        if (_operatorControl.getBButtonPressed()){
          extendGrip();
        }
    
        if (_operatorControl.getBButtonReleased()){
          _placingHatch = true;
          _needToCloseGrabber = false;
          _retrievingHatch = false;
          _extendingToGetHatch = false;
          _solenoidCycleCount = 0;
          openGrip();
        }
    
        if(_placingHatch && _solenoidCycleCount > _cycleDelay){
          _placingHatch = false;
          retractGrip();
          _needToCloseGrabber = true;
          _retrievingHatch = false;
          _extendingToGetHatch = false;
          _solenoidCycleCount = 0;
        }
    
        if (_needToCloseGrabber && _solenoidCycleCount > 12*_cycleDelay){
          _needToCloseGrabber = false;
          closeGrip();
        }
    
        //grabbing hatch
        if (_operatorControl.getAButtonPressed()){
          //open
          _gripSolenoid.set(DoubleSolenoid.Value.kReverse);
          _extendingToGetHatch = true;
          _needToCloseGrabber = false;
          _placingHatch = false;
          _retrievingHatch = false;
          _solenoidCycleCount = 0;
        }
    
        if (_extendingToGetHatch && _solenoidCycleCount > _cycleDelay){
          _extendingToGetHatch = false;
          extendGrip();
        }
    
        if (_operatorControl.getAButtonReleased()){
          _retrievingHatch = true;
          _needToCloseGrabber = false;
          _placingHatch = false;
          _extendingToGetHatch = false;
          _solenoidCycleCount = 0;
          closeGrip();
        }
    
        if (_retrievingHatch && _solenoidCycleCount > _cycleDelay){
          _retrievingHatch = false;
          retractGrip();
        }
    
        //ball holder control
        if (_operatorControl.getBumperPressed(Hand.kRight)){
          _ballHolder.set(DoubleSolenoid.Value.kForward);
        } else if (_operatorControl.getBumperPressed(Hand.kLeft)){
          _ballHolder.set(DoubleSolenoid.Value.kReverse);
        }
    }

    private void openGrip(){
        _gripSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    private void closeGrip(){
        _gripSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    private void extendGrip(){
        _gripExtension.set(DoubleSolenoid.Value.kReverse);
    }

    private void retractGrip(){
        _gripExtension.set(DoubleSolenoid.Value.kForward);
    }
}