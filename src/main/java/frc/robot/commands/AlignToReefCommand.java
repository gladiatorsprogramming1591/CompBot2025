package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToReefCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    // Supplier of the current teleop drive request (FieldCentric)
    private final Supplier<SwerveRequest.FieldCentric> teleopSupplier;
    // The robot-centric request used for auto alignment (translation scaling, etc.)
    private final SwerveRequest.RobotCentric reefAlign;
    // Flag: if true, use drivetrain pose for selecting left/right; if false, use camera TX values.
    private final boolean usePoseSelection;
    // A “default” desired heading offset used when using camera selection.
    private final double defaultHeadingOffset; 
    boolean autoScheduled = false;

    // PID for heading correction
    private final PIDController headingController = new PIDController(0.1, 0, 0);
    // Scaling factors (adjust as needed)
    private final double translationalScalingX = 0.3;
    private final double translationalScalingY = 0.38;
    private final double rotationalScaling = 0.5;

    public AlignToReefCommand(CommandSwerveDrivetrain drivetrain,
                              Supplier<SwerveRequest.FieldCentric> teleopSupplier,
                              SwerveRequest.RobotCentric reefAlign,
                              boolean usePoseSelection,
                              double defaultHeadingOffset) {
        this.drivetrain = drivetrain;
        this.teleopSupplier = teleopSupplier;
        this.reefAlign = reefAlign;
        this.usePoseSelection = usePoseSelection;
        this.defaultHeadingOffset = defaultHeadingOffset;
        // Set a tolerance (in degrees) for heading alignment
        headingController.setTolerance(2);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        SwerveRequest.FieldCentric teleopDrive = teleopSupplier.get();
        // Check if driver translation input exceeds the deadband.
        boolean isTeleopActive = (Math.abs(teleopDrive.VelocityX) > teleopDrive.Deadband ||
                                  Math.abs(teleopDrive.VelocityY) > teleopDrive.Deadband);
        // Get the current robot pose.
        Pose2d currentPose = drivetrain.getState().Pose;
        double desiredHeading = 0;
        double commandedX = 0;
        double commandedY = 0;
        double rotationCmd = 0;
        if (isTeleopActive) {
            // While driving, use teleop translation.
            commandedX = teleopDrive.VelocityX;
            commandedY = teleopDrive.VelocityY;
            // "Look at the reef": use the nearest reef face as reference.
            Pose2d nearestFace = FieldConstants.getNearestReefFace(currentPose);
            // Option 1: Use the rotation defined for the reef face.
            desiredHeading = nearestFace.getRotation().getDegrees();
            // Alternatively, you could compute the direction vector:
            // desiredHeading = nearestFace.getTranslation().minus(currentPose.getTranslation()).getAngle().getDegrees();
            double currentYaw = currentPose.getRotation().getDegrees();
            rotationCmd = headingController.calculate(currentYaw, desiredHeading);
            // Command teleop translation plus the rotation correction.
            drivetrain.setControl(
                reefAlign.withVelocityX(commandedX * translationalScalingX)
                        .withVelocityY(commandedY * translationalScalingY)
                        .withRotationalRate(rotationCmd * rotationalScaling)
            );
        } else {
            // When driver stops driving, take over and schedule an AutoScore command—only once.
            if (!autoScheduled) {
                // if (usePoseSelection) {
                    // Use drivetrain pose: get the left and right branches of the nearest reef face.
                    Pose2d leftBranch = FieldConstants.getNearestReefBranch(currentPose, ReefSide.LEFT);
                    Pose2d rightBranch = FieldConstants.getNearestReefBranch(currentPose, ReefSide.RIGHT);
                    double distLeft = currentPose.getTranslation().getDistance(leftBranch.getTranslation());
                    double distRight = currentPose.getTranslation().getDistance(rightBranch.getTranslation());
                    // Use the branch’s rotation as the desired heading.
                    if (distLeft < distRight) {
                        new AutoScoreLeftCommand(drivetrain, reefAlign).withTimeout(3).schedule();
                    } else {
                        new AutoScoreRightCommand(drivetrain, reefAlign).withTimeout(3).schedule();
                    }
                // } else {
                //     // Use limelight TX values: compare left and right TX errors.
                //     boolean leftHasTarget = LimelightHelpers.getTV("limelight-left");
                //     boolean rightHasTarget = LimelightHelpers.getTV("limelight-right");
                //     double txLeft = leftHasTarget ? Math.abs(LimelightHelpers.getTX("limelight-left")) : Double.MAX_VALUE;
                //     double txRight = rightHasTarget ? Math.abs(LimelightHelpers.getTX("limelight-right")) : Double.MAX_VALUE;
                //     double currentYaw = currentPose.getRotation().getDegrees();
                //     if (txLeft < txRight) {
                //         new AutoScoreLeftCommand(drivetrain, reefAlign).withTimeout(3).schedule();
                //     } else {
                //         new AutoScoreRightCommand(drivetrain, reefAlign).withTimeout(3).schedule();
                //     }
                // }
                autoScheduled = true;
            }
            // Command zero translation while waiting for the auto scoring command to take over.
            drivetrain.setControl(
                reefAlign.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
            );
        }
    }
    @Override
    public boolean isFinished() {
        // Once an auto scoring command is scheduled (i.e. driver has stopped driving), finish.
        return autoScheduled;
    }
}
