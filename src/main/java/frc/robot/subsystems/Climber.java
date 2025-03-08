package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final SparkBase climbRollerMotor; 
    private final SparkBase winchMotor; 
    AbsoluteEncoder climberEncoder; 
    
    public Climber() {
        climbRollerMotor = new SparkMax(frc.robot.Constants.ClimberConstants.CLIMB_ROLLER_CAN_ID, MotorType.kBrushless); 
          climbRollerMotor.configure(ClimberConstants.CLIMB_ROLLER_MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );      
            
          winchMotor = new SparkMax(frc.robot.Constants.ClimberConstants.WINCH_CAN_ID, MotorType.kBrushless); 
          winchMotor.configure(ClimberConstants.WINCH_MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );
          climberEncoder = winchMotor.getAbsoluteEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder Angle", getAngle());
        SmartDashboard.putNumber("Winch Current", winchMotor.getOutputCurrent()); 
        SmartDashboard.putNumber("Climb Roller Current", climbRollerMotor.getOutputCurrent()); 
    }

    /**
     * This gives us the angle of the motor
     * @return The current angle of the wrist in degrees
     */
    public double getAngle()
    {
         return climberEncoder.getPosition();
    }

    public void setclimbRollerMotor(double speed)
    {
        SmartDashboard.putNumber("Climb Roller Motor Speed", speed);
        climbRollerMotor.set(speed*0.2);
    }

    public void setWinchSpeed(double speed)
    {
        SmartDashboard.putNumber("Winch Motor Speed", speed);
        winchMotor.set(speed*0.2);
    }

    public Command manualClimbMovement(DoubleSupplier rollerSupplier, DoubleSupplier winchSupplier){
        return new ParallelCommandGroup(
          new RunCommand(()-> setWinchSpeed(winchSupplier.getAsDouble())),
          new RunCommand(()-> setclimbRollerMotor(rollerSupplier.getAsDouble()))
          ); 
    }
}
