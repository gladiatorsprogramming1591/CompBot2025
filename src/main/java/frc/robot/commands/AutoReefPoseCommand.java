package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
// import frc.robot.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
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

    private PIDController distanceController = new PIDController(2,0, 0.0);
    private PIDController strafeController = new PIDController(10, 0, 0.0);
    private PIDController angleController = new PIDController(0.5, 0, 0.02);

    private DoubleSupplier controllerX;
    private DoubleSupplier controllerY;
    private DoubleSupplier controllerT;
    private DoubleSupplier elevatorHeight;
    
        public AutoReefPoseCommand(CommandSwerveDrivetrain drivetrain,
                                        SwerveRequest.RobotCentric reefAlign,
                                        DoubleSupplier controllerX,
                                        DoubleSupplier controllerY,
                                        DoubleSupplier controllerT,
                                        Supplier<ReefSide> position,
                                        DoubleSupplier elevatorHeight) {
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
            this.elevatorHeight = elevatorHeight;
        }
    
        public void initialize() {
            // if(position.get() == ReefSide.RIGHT) {
            //     CommandSwerveDrivetrain.onlyUseRightCamera();
            // } else{
            //     CommandSwerveDrivetrain.onlyUseLeftCamera();
            // }
        }
    
        @Override
        public void execute() {
            double distanceVal = 0;
            double strafeVal = 0;
            double rotationVal = 0;
            Pose2d currentPose = drivetrain.getState().Pose;
            if(position == null)
            {
                position = () -> FieldConstants.getNearestReefSide(drivetrain.getState().Pose);
            }
            
            // boolean isCoral = state.getCurrentMode() == GamePiece.CORAL;
            Pose2d reefPose = FieldConstants.getNearestReefBranch(currentPose, position.get());
            // Need to uncomment below when we add RobotState or want to test ALGAE
            // Pose2d reefPose = FieldConstants.getNearestReefBranch(currentPose, state.getReefPos(position.get()));
            reefPose = reefPose.rotateAround(reefPose.getTranslation(), Rotation2d.k180deg);
            Pose2d goal = FieldConstants.toRobotRelative(currentPose, reefPose);
            
            distanceController.setSetpoint(0.095); 
            strafeController.setSetpoint(position.get() == ReefSide.RIGHT ? 0: 0);
            angleController.setSetpoint(position.get() == ReefSide.RIGHT ? -0.5 : -0.5);   
            
            double goalX = goal.getX();
            double goalY = goal.getY();
            double goalRotation = goal.getRotation().getDegrees();
    
            strafeVal = distanceController.calculate(goalX); 
            distanceVal = strafeController.calculate(goalY);
            rotationVal = angleController.calculate(goalRotation);
            strafeVal = 1/(Math.exp(Math.abs(distanceVal)*2));  
    
            if (strafeController.atSetpoint())
                strafeVal = 0;
            if (distanceController.atSetpoint())
                distanceVal = 0;
            if (angleController.atSetpoint())
                rotationVal = 0;
    
            /* Drive */
            double deadband = 0.00;
            double velocityX = controllerX.getAsDouble() + MathUtil.applyDeadband(strafeVal, deadband);
            double velocityY = controllerY.getAsDouble() - MathUtil.applyDeadband(distanceVal, deadband);
            double rotationalRate = controllerT.getAsDouble() - MathUtil.applyDeadband(rotationVal, deadband);

            double velocityXSign = velocityX / Math.abs(velocityX);
            double velocityYSign = velocityY / Math.abs(velocityY);
            double rotationalRateSign = rotationalRate / Math.abs(rotationalRate);

            double velocityXLimited = Math.min(Math.abs(velocityX), 0.75) * velocityXSign;
            // Limited to 0.22 max output at elevator height of 26.4 (L4)
            double velocityYLimited = Math.min(Math.abs(velocityY), Math.min(Math.pow(DriveConstants.LIMIT_SCALE_PER_INCH_AUTO_ALIGN, elevatorHeight.getAsDouble()), 1.0)) * velocityYSign;
            double rotationalRateLimited = Math.min(Math.abs(rotationalRate), 0.35) * rotationalRateSign;


        drivetrain.setControl(
            reefAlign.withVelocityX(velocityXLimited) // Drive forward with negative Y (forward) strafeController
                .withVelocityY(velocityYLimited) // Drive left with negative X (left)
                .withRotationalRate(rotationalRateLimited) // Drive counterclockwise with negative X (left)
        );

        SmartDashboard.putNumber("goalX", goalX);
        SmartDashboard.putNumber("goalY", goalY);
        SmartDashboard.putNumber("goalRotation", goalRotation);
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
        // CommandSwerveDrivetrain.useBothCameras();
        // RobotContainer.candleSubsystem.setAnimate("Rainbow");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

}