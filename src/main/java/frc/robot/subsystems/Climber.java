package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

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
    private CommandSwerveDrivetrain drivetrain;
    AbsoluteEncoder climberEncoder; 
    boolean winchAtPosition = false;
    boolean robotAtDesiredPitch = false;
    
    public Climber(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        climbRollerMotor = new SparkFlex(frc.robot.Constants.ClimberConstants.CLIMB_ROLLER_CAN_ID, MotorType.kBrushless); 
          climbRollerMotor.configure(ClimberConstants.CLIMB_ROLLER_MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );      
            
          winchMotor = new SparkFlex(frc.robot.Constants.ClimberConstants.WINCH_CAN_ID, MotorType.kBrushless); 
          winchMotor.configure(ClimberConstants.WINCH_MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );
          climberEncoder = winchMotor.getAbsoluteEncoder();
    }

    @Override
    public void periodic() {
        double TARGET_ANGLE_MIN = 130.0;
        double TARGET_ANGLE_MAX = 126.5;
        double angle = getAngle();
        if (angle > TARGET_ANGLE_MIN && angle < TARGET_ANGLE_MAX) {
            winchAtPosition = true;
        } else {
            winchAtPosition = false;
        }

        double TARGET_PITCH_MAX = -3.0;
        double TARGET_PITCH_MIN = -5.0;
        double pitch = drivetrain.getPitch();
        if (pitch > TARGET_PITCH_MIN && pitch < TARGET_PITCH_MAX) {
            robotAtDesiredPitch = true;
        } else {
            robotAtDesiredPitch = false;
        }

        SmartDashboard.putBoolean("Winch At Pos", winchAtPosition);
        SmartDashboard.putBoolean("At Desired Pitch", robotAtDesiredPitch);
        SmartDashboard.putNumber("Climb Encoder Angle", getAngle());
        SmartDashboard.putNumber("Climb Pitch", pitch);
        SmartDashboard.putNumber("Winch Current", winchMotor.getOutputCurrent()); 
        SmartDashboard.putNumber("Climb Roller Current", climbRollerMotor.getOutputCurrent()); 
        SmartDashboard.putBoolean("Start Winch Angle", isAtStartAngle()); 
    }

    /**
     * This gives us the angle of the motor
     * @return The current angle of the wrist in degrees
     */
    public double getAngle()
    {
         return climberEncoder.getPosition();
    }

    public boolean isAtStartAngle(){
        return (Math.abs(getAngle() - 127) < 1);
            
    }

    public void setclimbRollerMotor(double speed)
    {
        SmartDashboard.putNumber("Climb Roller Motor Speed", speed);
        climbRollerMotor.set(speed*-1.0);
    }

    public void setWinchSpeed(double speed)
    {
        SmartDashboard.putNumber("Winch Motor Speed", speed);
        winchMotor.set(speed*1.0);
    }

    class DefaultCommand extends ParallelCommandGroup {
        public DefaultCommand(Climber climber, DoubleSupplier rollerSupplier, DoubleSupplier winchSupplier) {
            addRequirements(climber);
            addCommands(
                new RunCommand(()-> climber.setWinchSpeed(winchSupplier.getAsDouble()))
                    .until(()-> getAngle() > 105.0)
                    .andThen(()->climber.setWinchSpeed(0)),
                new RunCommand(()-> climber.setclimbRollerMotor(rollerSupplier.getAsDouble()))
            );
        }
    }

    public Command manualClimbMovement(DoubleSupplier rollerSupplier, DoubleSupplier winchSupplier){
        return new DefaultCommand(this, rollerSupplier, winchSupplier);
    }
}
