package frc.robot.commands;

// import frc.robot.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoReefPoseCommand extends Command {
    private CommandSwerveDrivetrain drivetrain; 
    private SwerveRequest.RobotCentric reefAlign;   
    private Supplier<ReefSide> position;
    // private RobotState state;

    private PIDController distanceController = new PIDController(1,0, 0.0);
    private PIDController strafeController = new PIDController(10, 0, 0.0);
    private PIDController angleController = new PIDController(0.5, 0, 0.02);

    private DoubleSupplier controllerX;
    private DoubleSupplier controllerY;
    private DoubleSupplier controllerT;

    public AutoReefPoseCommand(CommandSwerveDrivetrain drivetrain,
                                    SwerveRequest.RobotCentric reefAlign,
                                    DoubleSupplier controllerX,
                                    DoubleSupplier controllerY,
                                    DoubleSupplier controllerT,
                                    Supplier<ReefSide> position) {
                                        // Supplier<ReefSide> position,
                                        // RobotState state) {
        this.drivetrain = drivetrain;
        this.reefAlign = reefAlign;
        this.position = position;
        // this.state = state;
        distanceController.setTolerance(0.0);
        strafeController.setTolerance(0.0);
        angleController.setTolerance(0.0);
        angleController.enableContinuousInput(-180,180);
        addRequirements(drivetrain);
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerT = controllerT;
    }

    public void initialize() {
    }
    
    @Override
    public void execute() {
        double distanceVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d reefPose = FieldConstants.getNearestReefBranch(currentPose, position.get());
        // Need to uncomment below when we add RobotState or want to test ALGAE
        // Pose2d reefPose = FieldConstants.getNearestReefBranch(currentPose, state.getReefPos(position.get()));
        reefPose = reefPose.rotateAround(reefPose.getTranslation(), Rotation2d.k180deg);
        Pose2d goal = FieldConstants.toRobotRelative(currentPose, reefPose);
        
        distanceController.setSetpoint(0.15); 
        strafeController.setSetpoint(0);
        angleController.setSetpoint(1);   
        
        strafeVal = distanceController.calculate(goal.getX()); 
        distanceVal = strafeController.calculate(goal.getY());
        rotationVal = angleController.calculate(goal.getRotation().getDegrees());

        if (strafeController.atSetpoint())
            strafeVal = 0;
        if (distanceController.atSetpoint()) 
            distanceVal = 0;
        if (angleController.atSetpoint())
            rotationVal = 0;

        /* Drive */
        double velocityX = (controllerX.getAsDouble()-strafeVal) * 0.5;
        double velocityY = (controllerY.getAsDouble()-distanceVal) * 0.5;
        double rotationalRate = (controllerT.getAsDouble()-rotationVal) * 0.25;
        drivetrain.setControl(
            reefAlign.withVelocityX(velocityX) // Drive forward with negative Y (forward) strafeController
                .withVelocityY(velocityY) // Drive left with negative X (left)
                .withRotationalRate(rotationalRate) // Drive counterclockwise with negative X (left)
        );

        SmartDashboard.putNumber("strafeVal", strafeVal);
        SmartDashboard.putNumber("distanceVal", distanceVal);
        SmartDashboard.putNumber("rotationVal", rotationVal);
        SmartDashboard.putNumber("Auto align velocity X", velocityX);
        SmartDashboard.putNumber("Auto align velocity Y", velocityY);
        SmartDashboard.putNumber("Auto align rot rate", rotationalRate);
        
    }
      // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        // If all 3 PIDs are at their target, we're done
        return distanceController.atSetpoint() 
            && strafeController.atSetpoint() 
            && angleController.atSetpoint();
    }

    // Called once after isFinished returns true
    protected void end() {
        // RobotContainer.candleSubsystem.setAnimate("Rainbow");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end(); 
    }

}