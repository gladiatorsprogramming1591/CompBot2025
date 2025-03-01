package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    SparkFlex intakeMotor;
    DigitalInput placementCoralBeamBreak;
    SparkLimitSwitch coralRearBeam;
    SparkLimitSwitch coralFrontBeam;

   
    public EndEffector(){
         intakeMotor = new SparkFlex(EndEffectorConstants.EE_MOTOR_ID, MotorType.kBrushless);
            intakeMotor.configure(
                EndEffectorConstants.MOTOR_CONFIG, 
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            );

            coralFrontBeam = intakeMotor.getReverseLimitSwitch();
            coralRearBeam = intakeMotor.getForwardLimitSwitch();
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


    public boolean isCoralFrontBeamBroken() {
        return coralFrontBeam.isPressed();
    }

    public boolean isCoralRearBeamBroken(){
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

    /**
     *spins intake backwards to arm the coral
     */
    public void armCoral() {
        intakeMotor.set(EndEffectorConstants.ARM_CORAL_SPEED);
    }
    
    public boolean coralArmed() {
        return isCoralRearBeamBroken() && isCoralFrontBeamBroken();
    }

    public boolean hasCoral() {
        return ((!isCoralRearBeamBroken()) && isCoralFrontBeamBroken());
    }

    public void algaeCheckRoutine() {
        if(intakeMotor.getOutputCurrent() < EndEffectorConstants.HAS_ALGAE_CURRENT){intakeMotor.stopMotor();}//Multiple loop check routine
    }

    public Command intakeAlgaeCommand() {
        return new RunCommand(() -> setCoralSpeed(EndEffectorConstants.ALGAE_INTAKE_SPEED));
    }

    /**
     * Runs the intake until the robot has coral, slowing down as coral progresses through the system
     * @return the command
     */
     public Command intakeCoralCommand() {
        return new SequentialCommandGroup(new RunCommand(() -> setCoralSpeed(0.1))
            .until(this::hasCoral),
            new RunCommand(() -> setCoralSpeed(-0.1))
            .until(this::coralArmed),
            new RunCommand(() ->setCoralSpeed(0)));
    }

    public Command ejectCoralCommand() {
        return new SequentialCommandGroup(new RunCommand(() -> ejectCoral())
            .until(()-> (!isCoralRearBeamBroken()) && !isCoralFrontBeamBroken()),
            new InstantCommand(() ->setCoralSpeed(0)));
    }

    public Command ejectAlgaeCommand(){
        return new InstantCommand(()-> ejectAlgae()); 
    }
    





    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Algae?", hasAlgae()); 
        SmartDashboard.putNumber("ee Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Rear Beam Broken?", isCoralRearBeamBroken()); 
        SmartDashboard.putBoolean("Front Beam Broken?", isCoralFrontBeamBroken()); 
    }
}
