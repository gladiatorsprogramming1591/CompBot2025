package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

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
