// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 1 1/2 of a rotation per second max angular velocity
    // TODO: Move this into constants
    // public static final double INITIAL_DEADBAND = 0.10; // 10% Deadband
    public static final double DEADBAND = 0.10; // 10% Deadband
    public static final double MIN_DEADBAND = 0.05; // 5% Deadband to perpendicular axis while robot is at max speed. (Scaled in between)

    /* Setting up bindings for necessary control of the swerve drive platform */
    // TODO: Issue: Deadband is not applied while bot is in motion (e.g. strafing while driving).
    // - Suspicion that output does not scale from 0 to max after the deadband (joystick is touchy). 
    // TODO: Idea?: Try changing OpenLoopVoltage to Velocity if we switch to FOC
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * INITIAL_DEADBAND).withRotationalDeadband(MaxAngularRate * INITIAL_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController joystick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser(); // A default auto can be passed in as parameter.
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-applyDynamicDeadband(joystick.getLeftY(), joystick.getLeftX(), DEADBAND, MIN_DEADBAND) * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-applyDynamicDeadband(joystick.getLeftX(), joystick.getLeftY(), DEADBAND, MIN_DEADBAND) * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-MathUtil.applyDeadband(joystick.getRightX(), DEADBAND) * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        // joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.configNeutralMode(NeutralModeValue.Coast)));
        // joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.configNeutralMode(NeutralModeValue.Brake)));
        // joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.setCoastMode()));
        // joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.setBrakeMode()));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.leftTrigger().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightTrigger().onTrue(Commands.runOnce(SignalLogger::stop));

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double applyDynamicDeadband(double axis, double perpendicularAxis, double Deadband, double minDeadband)
    {
        return MathUtil.applyDeadband(axis, MathUtil.clamp(Math.abs(1 - perpendicularAxis) * Deadband, minDeadband, Deadband));
    }

    // public double applyDualDeadband(double axis, double perpendicularAxis, double staticDeadband, double kineticDeadband)
    // {
    //     MathUtil.applyDeadband(axis, MathUtil.clamp(Math.abs(1 - perpendicularAxis) * staticDeadband, kineticDeadband, staticDeadband));
    //     return MathUtil.applyDeadband(axis, MathUtil.clamp(Math.abs(1 - perpendicularAxis) * staticDeadband, kineticDeadband, staticDeadband));
    // }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
