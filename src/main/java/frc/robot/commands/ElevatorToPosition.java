package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToPosition extends Command {
    private ElevatorSubsystem m_elevator; 
    private ElevatorSubsystem.elevatorPositions m_targetPos; 
    private double m_elevatorSetpoint; 
    private boolean m_keepRunning; 
    private boolean m_customSetpoint = false; 
    private int m_currentLimit = ElevatorConstants.CURRENT_LIMIT; 

    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.elevatorPositions pos){
        m_elevator = elevator;
        m_targetPos = pos;
        m_keepRunning = false;
        addRequirements(m_elevator);
    }

    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.elevatorPositions pos, boolean keepRunning){
        m_elevator = elevator;
        m_targetPos = pos;
        m_keepRunning = keepRunning;
        addRequirements(m_elevator);
    }

    public ElevatorToPosition(ElevatorSubsystem elevator, double elevatorSetpoint){
        m_elevator = elevator;
        m_elevatorSetpoint = elevatorSetpoint;
        m_customSetpoint = true;
        addRequirements(m_elevator);
    }

    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.elevatorPositions pos, int currentLimit) {
        m_elevator = elevator;
        m_targetPos = pos;
        m_keepRunning = false;
        addRequirements(m_elevator);
        m_currentLimit = currentLimit;
    }

    @Override
    public void execute(){
        m_elevator.ElevatorToPositionInternalEnc(m_targetPos);
    }

    @Override
    public boolean isFinished(){
        // This command just sets the setpoint and is done, so return true.
        return true;
    }

    @Override
    public void end(boolean isInterrupted){
        // Do nothing since we want the motors to continue running to/at the setpoint.
    }
}
