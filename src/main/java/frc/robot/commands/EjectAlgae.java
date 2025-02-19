package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector;

public class EjectAlgae extends Command {
    private EndEffector m_endEffector;
    public EjectAlgae(EndEffector endEffector) {
        m_endEffector = endEffector; 
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        m_endEffector.setCoralSpeed(EndEffectorConstants.ALGAE_EJECT_SPEED);
    }

    @Override 
    public void end(boolean isInterrupted) {
        m_endEffector.hasAlgae();
    }

    @Override
    public boolean isFinished(){
        return false; //TODO: read beam break sensor
    }
}
