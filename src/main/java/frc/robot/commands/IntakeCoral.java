package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector;

public class IntakeCoral extends Command {
    private EndEffector m_endEffector;


    public IntakeCoral(EndEffector endEffector) {
        m_endEffector = endEffector;  
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        m_endEffector.setCoralSpeed(EndEffectorConstants.CORAL_INTAKE_SPEED);
    }

    @Override 
    public void end(boolean isInterrupted) {
        m_endEffector.setCoralSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return (m_endEffector.isCoralFrontBeamBroken() && !m_endEffector.isCoralRearBeamBroken());
    }
}
