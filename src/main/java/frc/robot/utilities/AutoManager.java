package frc.robot.utilities;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoManager {
    private SendableChooser<Command> autos;
    private CommandSwerveDrivetrain drivetrain;
    private String leftSide3PieceName = "StartLineJCoral";
    private String rightSide3PieceName = "Right StartLineJCoral";

    private PathConstraints constraints = new PathConstraints(
            7, 3, 1, 1
    );

    public AutoManager(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        autos = new SendableChooser<>();
        autos.addOption("Left L4 3 Piece", wrapAutoWithPose(new PathPlannerAuto(leftSide3PieceName)));
        autos.addOption("New Right L4 3 Piece", wrapAutoWithPose(new PathPlannerAuto(rightSide3PieceName, true)));
        autos.addOption("CenterAuto", wrapAutoWithPose(new PathPlannerAuto("CenterAuto", false)));
    }

    private Command wrapAutoWithPose(PathPlannerAuto autoCommand) {
        return new InstantCommand(() -> {
            new PathPlannerAuto(autoCommand, drivetrain.getState().Pose).schedule();
        });
    }

    public SendableChooser<Command> getChooser() {
        return autos;
    }
}
