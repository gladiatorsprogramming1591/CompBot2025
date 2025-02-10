package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    SparkFlex intakeMotor;
    public EndEffector(){
         
    }

    //Methods for Coral
    public void setCoralSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void ejectCoral() {
        intakeMotor.set(EndEffectorConstants.CORAL_EJECT_SPEED);  
        // intakeMotor.set(-speed); 
    }

    // Methods for Algae
    public boolean hasAlgae() {
        return intakeMotor.getOutputCurrent() > EndEffectorConstants.HAS_ALGAE_CURRENT;
    }

    public void ejectAlgae() {
        intakeMotor.set(EndEffectorConstants.ALGAE_EJECT_SPEED); 
    }

    @Override
    public void periodic() {
    //TODO: Get values we want for the dashboard
    }
}
