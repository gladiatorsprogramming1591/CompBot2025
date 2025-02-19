package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector;

public class EjectCoral extends Command {
    private EndEffector m_endEffector;
    private DigitalInput intakeBeamBreak; 
    private DigitalInput placementBeamBreak; 
    public EjectCoral(EndEffector endEffector) {
        m_endEffector = endEffector; 
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        m_endEffector.setCoralSpeed(EndEffectorConstants.CORAL_EJECT_SPEED);
    }

    @Override 
    public void end(boolean isInterrupted) {
        m_endEffector.setCoralSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return intakeBeamBreak.get() && placementBeamBreak.get();
    }
}
