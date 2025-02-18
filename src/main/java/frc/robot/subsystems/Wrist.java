package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final SparkBase wristMotor; 
    AbsoluteEncoder wristEncoder; 
    SparkClosedLoopController wristController; 
    double holdAngle; 

    public enum wristPositions { 
        //wrist positions net scoring, algae acquire, 
        STOW, //Will hit bar on elevator if moved fully back. 
        NET_SCORE, 
        REEF_ACQUIRE,
        GROUND_INTAKE
    }
    
    
    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless); 
          wristMotor.configure(WristConstants.MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );   
          wristEncoder = wristMotor.getAbsoluteEncoder(); 
          wristController = wristMotor.getClosedLoopController(); 
          holdAngle = ElevatorConstants.STOW_ANGLE; 
        
    }

    public double getPosition() {
        return wristEncoder.getPosition(); 
    }

     public void setAngle(double angle) {
          wristController.setReference(angle, ControlType.kPosition);
     }  

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Pos", wristEncoder.getPosition());
    }
}
