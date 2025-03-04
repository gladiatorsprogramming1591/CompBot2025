package frc.robot.subsystems;

import java.util.EnumMap;

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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final SparkBase wristMotor; 
    AbsoluteEncoder wristEncoder; 
    SparkClosedLoopController wristController; 
    double holdAngle;
    private WristPosition desiredPosition; 

    EnumMap<WristPosition, Double> wristMap = new EnumMap<>(WristPosition.class);

    public enum WristPosition{
     STOW, 
     L1,
     CORAL_INTAKE,
     REEF_ACQUIRE, 
     GROUND_INTAKE, 
     PROCESSOR, 
     HOVER,
     HOVER_L4, 
     ALGAE_LOW,
     ALGAE_HIGH, 
     DUNK, 
     REEF_ALGAE_LOW, 
     REEF_ALGAE_HIGH
    }

    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless); 
          wristMotor.configure(WristConstants.MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );   
          wristEncoder = wristMotor.getAbsoluteEncoder(); 
          wristController = wristMotor.getClosedLoopController(); 
          holdAngle = ElevatorConstants.kSTOW; 
          
     
        wristMap.put(WristPosition.STOW, WristConstants.WRIST_STOW);
        wristMap.put(WristPosition.L1, WristConstants.WRIST_L1);
        wristMap.put(WristPosition.CORAL_INTAKE, WristConstants.WRIST_STOW);
        wristMap.put(WristPosition.REEF_ACQUIRE, WristConstants.REEF_ACQUIRE_ANGLE);
        wristMap.put(WristPosition.GROUND_INTAKE, WristConstants.GROUND_INTAKE);
        wristMap.put(WristPosition.PROCESSOR, WristConstants.WRIST_PROCESSOR);
        wristMap.put(WristPosition.HOVER, WristConstants.WRIST_HOVER);
        wristMap.put(WristPosition.HOVER_L4, WristConstants.WRIST_HOVER_L4);
        wristMap.put(WristPosition.ALGAE_HIGH, WristConstants.WRIST_ALGAE_HIGH);
        wristMap.put(WristPosition.ALGAE_LOW, WristConstants.WRIST_ALGAE_LOW);
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

    public void setAngle(double angle)
    {
         holdAngle = angle;
         wristController.setReference(holdAngle, ControlType.kPosition);
    }  

    public void setHoldAngle()
    {
         wristController.setReference(holdAngle, ControlType.kPosition);
    }

    public boolean atSetpoint(){
     double tolerance = 5;
          return Math.abs(getAngle() - holdAngle) < tolerance; 
    }
    
    public void setPosition(WristPosition position) {
        desiredPosition = position;
        switch (position) {
             case HOVER -> setAngle(WristConstants.WRIST_HOVER);
             case GROUND_INTAKE -> setAngle(WristConstants.GROUND_INTAKE);
             case REEF_ALGAE_LOW -> setAngle(WristConstants.WRIST_ALGAE_LOW);
             case REEF_ALGAE_HIGH -> setAngle(WristConstants.WRIST_ALGAE_HIGH);
             case DUNK -> setAngle(WristConstants.WRIST_DUNK_CORAL);
             case PROCESSOR -> setAngle(WristConstants.WRIST_PROCESSOR);
             case L1 -> setAngle(WristConstants.WRIST_L1);
             default -> setAngle(WristConstants.WRIST_STOW);
        }
    }
    
    public Command AquirePositionCommand()
    {
        return new InstantCommand(()->setAngle(WristConstants.REEF_ACQUIRE_ANGLE));
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

    public Command HoverPositionCommand(){
          return new InstantCommand(()->setAngle(WristConstants.WRIST_HOVER)); 
     }
    public Command HoldPositionCommand()
    {
         return new RunCommand(()->setHoldAngle(),this);
    }

    public Command manualWristMovement(double speed){
        return new RunCommand(()-> setWristMotor(speed)); 
    }
}
