package frc.robot.commands;

import org.w3c.dom.ElementTraversal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPositions;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public class ScoreCoral extends SequentialCommandGroup {

    public ScoreCoral(EndEffector endEffector,
                        ElevatorSubsystem elevatorSubsystem,
                        Wrist wrist
                        ) {
        addRequirements(endEffector, elevatorSubsystem, wrist);
        addCommands(
            new ElevatorToPosition(elevatorSubsystem, elevatorPositions.STOW),
            new IntakeCoral(endEffector),
            new ElevatorToPosition(elevatorSubsystem, elevatorPositions.L4),
            new InstantCommand(() ->wrist.setAngle(30))
            );
                        }
    
}
