package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final SparkBase wristMotor; 
    AbsoluteEncoder wristEncoder; 
    SparkClosedLoopController wristController; 
    double holdAngle;
    
    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless); 
          wristMotor.configure(WristConstants.MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );   
          wristEncoder = wristMotor.getAbsoluteEncoder(); 
          wristController = wristMotor.getClosedLoopController(); 
          holdAngle = WristConstants.WRIST_STOW; 
        
    }

    public double getPosition() {
        return wristEncoder.getPosition(); 
    }

     

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hold Angle", holdAngle);
        SmartDashboard.putNumber("Wrist Encoder Angle", getAngle());
        SmartDashboard.putNumber("Wrist Current", wristMotor.getOutputCurrent()); 
    }

    /**
     * This gives us the angle of the motor
     * @return The current angle of the wrist in degrees
     */
    public double getAngle()
    {
         return wristEncoder.getPosition();
    }

    /**
    * This gives us the hold angle of the PID controller
    * @return The current hold angle of the PID controller in degrees
    */
    public double getHoldAngle()
    {
        return holdAngle;
    }

    public void setWristMotor(double speed)
    {
        SmartDashboard.putNumber("Wrist Motor Speed", speed);
        wristMotor.set(speed);
    }

    public void setWristForwardSpeed(DoubleSupplier speedSupplier)
    {
        SmartDashboard.putNumber("Wrist Motor Speed", speedSupplier.getAsDouble());
        wristMotor.set(speedSupplier.getAsDouble()*0.2);
    }

    public void setWristReverseSpeed(DoubleSupplier speedSupplier)
    {
        SmartDashboard.putNumber("Wrist Motor Speed", speedSupplier.getAsDouble());
        wristMotor.set(speedSupplier.getAsDouble()*-0.2);
    }

    public void setAngle(double angle)
    {
         if (getAngle() > Constants.WristConstants.GROUND_INTAKE + 5.0 || getAngle() >= 0 && getAngle() < Constants.WristConstants.WRIST_INTAKE / 2.0) {
          setWristReverseSpeed(()-> 0.5);
     //     } else if (getAngle() < Constants.WristConstants.WRIST_INTAKE && getAngle() > Constants.WristConstants.WRIST_INTAKE / 2.0) {
     //      setWristForwardSpeed(()-> 1.0);
         } else {
          holdAngle = angle;
          wristController.setReference(holdAngle, ControlType.kPosition);
     }
    }  

    public boolean atSetpoint(){
     double tolerance = 6;
          return Math.abs(getAngle() - holdAngle) < tolerance; 
    }

    public Command StowPositionCommand(){
         return new InstantCommand(()->setAngle(WristConstants.WRIST_STOW)); 
    }

    public Command IntakePositionCommand(){
         return new InstantCommand(()->setAngle(WristConstants.WRIST_INTAKE)); 
    }

    public Command ProcessorPositionCommand(){ 
          return new InstantCommand(()-> setAngle(WristConstants.WRIST_PROCESSOR)); 
    }

    public Command HoverPositionCommandL1(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_L1)); 
     }

    public Command HoverPositionCommandL2(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_L2)); 
     }
    public Command HoverPositionCommandL3(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_L3)); 
     }

     public Command HoverPositionCommandL4(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_L4)); 
     }
     public Command LowAlgaePositionCommand(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_ALGAE_LOW)); 
     }
     public Command HighAlgaePositionCommand(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_ALGAE_HIGH)); 
     }

    public Command manualWristForwardMovement(DoubleSupplier speedSupplier){
        return new RunCommand(()-> setWristForwardSpeed(speedSupplier), this); 
    }

    public Command manualWristReverseMovement(DoubleSupplier speedSupplier){
     return new RunCommand(()-> setWristReverseSpeed(speedSupplier), this); 
 }
}
