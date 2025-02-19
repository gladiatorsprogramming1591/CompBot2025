package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    SparkFlex intakeMotor;
    DigitalInput placementCoralBeamBreak;
    DigitalInput intakeBeamBreak;
    SparkLimitSwitch coralRearBeam;
    SparkLimitSwitch coralFrontBeam;

   
    public EndEffector(){
         intakeMotor = new SparkFlex(EndEffectorConstants.EE_MOTOR_ID, MotorType.kBrushless);
            intakeMotor.configure(
                EndEffectorConstants.MOTOR_CONFIG, 
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            );

            coralFrontBeam = intakeMotor.getForwardLimitSwitch();
            coralRearBeam = intakeMotor.getReverseLimitSwitch();
    }

    //Methods for Coral
    public void setCoralSpeed(double speed) {
        intakeMotor.set(speed);
    }
    
    public void intakeCoral() {
        intakeMotor.set(EndEffectorConstants.CORAL_INTAKE_SPEED); 
    }

    public void ejectCoral() {
        intakeMotor.set(EndEffectorConstants.CORAL_EJECT_SPEED);  
    }

    public boolean hasCoral() { 
        if(placementCoralBeamBreak.get() == true || intakeBeamBreak.get() == true){ 
            return true; 
        }
        else {
            return false; 
        }
    }

    public boolean isCoralFrontBeamBroken()
    {
        return coralFrontBeam.isPressed();
    }

    public boolean isCoralRearBeamBroken()
    {
        return coralRearBeam.isPressed();
    }

    // Methods for Algae
    public boolean hasAlgae() {
        return intakeMotor.getOutputCurrent() > EndEffectorConstants.HAS_ALGAE_CURRENT;
    }

    public void intakeAlgae() {
        intakeMotor.set(EndEffectorConstants.ALGAE_INTAKE_SPEED);
    }

    public void ejectAlgae() {
        intakeMotor.set(EndEffectorConstants.ALGAE_EJECT_SPEED); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Algae?", hasAlgae()); 

    }
}
