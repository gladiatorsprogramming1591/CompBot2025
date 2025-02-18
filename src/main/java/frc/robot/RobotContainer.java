// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPositions;

public class RobotContainer {
    //Subsystems 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final EndEffector endEffector = new EndEffector();
	private final Wrist wrist = new Wrist();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 1 1/2 of a rotation per second max angular velocity
    // TODO: Move this into constants
    public static final double DEADBAND = 0.10; // 10% Deadband
    public static final double MIN_DEADBAND = 0.05; // 5% Deadband to perpendicular axis while robot is in motion

    /* Setting up bindings for necessary control of the swerve drive platform */
    // TODO: Issue: Deadband is not applied while bot is in motion (e.g. strafing while driving).
    // - Suspicion that output does not scale from 0 to max after the deadband (joystick is touchy). 
    // TODO: Idea?: Try changing OpenLoopVoltage to Velocity if we switch to FOC
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController joystick = new CommandXboxController(0);
        
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
                        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // elevator.setDefaultCommand(new RunCommand(() -> elevator.setMotorSpeed(joystick.getRightY()),elevator));

        //Driver Controls
        joystick.a().onTrue(new IntakeCoral(endEffector));
        joystick.b().onTrue(new IntakeAlgae(endEffector)); 

        joystick.rightTrigger().whileTrue(new InstantCommand(()-> endEffector.ejectAlgae()))
            .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0)));
        joystick.leftTrigger().whileTrue(new InstantCommand(()-> endEffector.ejectCoral()))
            .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0)));

        joystick.rightStick().onTrue(new InstantCommand(()-> wrist.setAngle(30))  // TODO: Button conflict
            .andThen(()-> endEffector.ejectAlgae()));
                    
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.povDown().onTrue(new ElevatorToPosition(elevator, elevatorPositions.L1)); 
        joystick.povLeft().onTrue(new ElevatorToPosition(elevator, elevatorPositions.L2)); 
        joystick.povUp().onTrue(new ElevatorToPosition(elevator, elevatorPositions.L3)); 
        joystick.povRight().onTrue(new ElevatorToPosition(elevator, elevatorPositions.L4)); 

        joystick.x().onTrue(new ElevatorToPosition(elevator, elevatorPositions.STOW)); 
        joystick.y().onTrue(new InstantCommand(() -> elevator.zeroElevator()));


        // Tunning commands
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //         point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // joystick.leftTrigger().onTrue(Commands.runOnce(SignalLogger::start));
        // joystick.rightTrigger().onTrue(Commands.runOnce(SignalLogger::stop));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
