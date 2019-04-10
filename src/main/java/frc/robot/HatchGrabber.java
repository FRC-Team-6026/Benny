package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;

public class HatchGrabber{
    private final Compressor _compressor = new Compressor(10);
    private final DoubleSolenoid _gripSolenoid = new DoubleSolenoid(10,2,3);
    private final DoubleSolenoid _gripExtension = new DoubleSolenoid(10,0,1);
    private final int _cycleDelay = 20;
    private final XboxController _operatorControl;
    private int _solenoidCycleCount = 0;
    private GrabberState _grabberState = GrabberState.None;

    public HatchGrabber(XboxController operatorControl) {
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
          _grabberState = GrabberState.None;
          openGrip();
        }
    
        if (_operatorControl.getXButtonPressed()){
          _grabberState = GrabberState.None;
          closeGrip();
        }
    
        //placing hatch
        if (_operatorControl.getBButtonPressed()){
          extendGrip();
        }
    
        if (_operatorControl.getBButtonReleased()){
          _grabberState = GrabberState.PlacingHatch;
          _solenoidCycleCount = 0;
          openGrip();
        }
    
        if(_grabberState == GrabberState.PlacingHatch && _solenoidCycleCount > _cycleDelay){
          retractGrip();
          _grabberState = GrabberState.NeedToCloseGrabber;
          _solenoidCycleCount = 0;
        }
    
        if (_grabberState == GrabberState.NeedToCloseGrabber && _solenoidCycleCount > 12*_cycleDelay){
          _grabberState = GrabberState.None;
          closeGrip();
        }
    
        //grabbing hatch
        if (_operatorControl.getAButtonPressed()){
          openGrip();
          _grabberState = GrabberState.ExtendingToGetHatch;
          _solenoidCycleCount = 0;
        }
    
        if (_grabberState == GrabberState.ExtendingToGetHatch && _solenoidCycleCount > _cycleDelay){
          _grabberState = GrabberState.None;
          extendGrip();
        }
    
        if (_operatorControl.getAButtonReleased()){
          _grabberState = GrabberState.RetrievingHatch;
          _solenoidCycleCount = 0;
          closeGrip();
        }
    
        if (_grabberState == GrabberState.RetrievingHatch && _solenoidCycleCount > _cycleDelay){
          _grabberState = GrabberState.None;
          retractGrip();
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

    private enum GrabberState {
      None,
      RetrievingHatch,
      PlacingHatch,
      ExtendingToGetHatch,
      NeedToCloseGrabber
    }
}