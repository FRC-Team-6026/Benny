package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drivetrain{
    WPI_TalonSRX leftFront = new WPI_TalonSRX(0);
    WPI_TalonSRX leftRear = new WPI_TalonSRX(1);
    SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftRear);
    WPI_TalonSRX rightFront = new WPI_TalonSRX(2);
    WPI_TalonSRX rightRear = new WPI_TalonSRX(3);
    SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightRear);
    DifferentialDrive drive = new DifferentialDrive(left, right);
    XboxController controller;

    public Drivetrain(XboxController controller){
        if (controller == null){
            throw new IllegalArgumentException("controller is null");
        }
    }

    public void arcadeDrive(){
        drive.arcadeDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kLeft));
    }
}