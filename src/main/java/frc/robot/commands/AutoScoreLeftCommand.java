package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoScoreLeftCommand extends Command {    
    private CommandSwerveDrivetrain drivetrain; 
    private SwerveRequest.RobotCentric reefAlign;   
    AprilTagFieldLayout fieldLayout;

    private double tx;
    private double ta;
    private double tAng;
    private double rotationVal;
    private double strafeVal;
    private double distanceVal;

    private double targetAngle = 1.5;
    private double targetStrafe = 0;
    private double targetArea = 22.5;

    PIDController strafeController = new PIDController(0.15, 0, 0.02);
    PIDController distanceController = new PIDController(0.15, 0, 0.0);
    PIDController angleController = new PIDController(0.1, 0, 0);

    public AutoScoreLeftCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric reefAlign) {
        this.drivetrain = drivetrain;
        this.reefAlign = reefAlign;
        strafeController.setTolerance(0.25);
        distanceController.setTolerance(1);
        angleController.setTolerance(.15);
        addRequirements(drivetrain);
    }

    public void initialize() {
        strafeController.setSetpoint(targetStrafe); // to do local pid controller
        distanceController.setSetpoint(targetArea);
        angleController.setSetpoint(targetAngle);
    }
    
    @Override
    public void execute() {
        // Check if the tag is visible
        PhotonTrackedTarget target = drivetrain.getLastTarget();    
        if(target != null) {
            tx = MathUtil.clamp(target.getYaw(), -10, 10);
            ta = target.getArea();
            tAng = target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees();
            // When the tag is visible, use tx and ta from the limelight
        
            // Display diagnostic values
            SmartDashboard.putNumber("tx", tx);
            SmartDashboard.putNumber("ta", ta);
            SmartDashboard.putNumber("tAng", tAng);

            // Uses PID to point at target
            rotationVal = -angleController.calculate(tAng, targetAngle);
            strafeVal = strafeController.calculate(tx, targetStrafe);
            distanceVal = distanceController.calculate(ta, targetArea);

            // Zero out values if at setpoint
            if (distanceController.atSetpoint()) distanceVal = 0;
            if (strafeController.atSetpoint()) strafeVal = 0;
            if (angleController.atSetpoint()) rotationVal = 0;

            SmartDashboard.putNumber("Y Vel", strafeVal * 0.38);

            // Command the drivetrain using the calculated values
            drivetrain.setControl(
                reefAlign.withVelocityX(distanceVal * 0.3)
                        .withVelocityY(strafeVal * 0.38)
                        .withRotationalRate(-rotationVal * 0.5)
            );
        } 
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