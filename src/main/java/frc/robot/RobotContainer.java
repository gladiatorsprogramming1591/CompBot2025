// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.kSTOW;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.robotInitConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPositions;

public class RobotContainer {
    //Subsystems 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final EndEffector endEffector = robotInitConstants.isCompBot ? new EndEffector() : null;
	private final Wrist wrist = robotInitConstants.isCompBot ? new Wrist() : null;
	public final ElevatorSubsystem elevator = robotInitConstants.isCompBot ? new ElevatorSubsystem() : null;


    private double MaxSpeed = robotInitConstants.isCompBot ? PoseidonTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : ChazTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
        
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
                        drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );
        
        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // Subsystems
        // All Poseidon-specific commands MUST be within if-statement, or a NullPointerException will be thrown.
        if (robotInitConstants.isCompBot) {    
            // =====================================  Driver Controls  =====================================
            // Elevator
            driverController.start().onTrue(complexElevatorStowCommand(elevatorPositions.STOW));
            
            // End Effector
            // driverController.a().whileTrue(endEffector.intakeCoralCommand())
            //     .onFalse(new InstantCommand(()-> endEffector.setCoralSpeed(0)));
            driverController.a().whileTrue(complexIntakeCommand(elevatorPositions.STOW)) 
                .onFalse(new RunCommand(()-> endEffector.setCoralSpeed(0))); 
            driverController.b().whileTrue(new RunCommand(()-> endEffector.setCoralSpeed(EndEffectorConstants.ALGAE_INTAKE_SPEED)))
                .onFalse(new RunCommand(()-> endEffector.algaeCheckRoutine()));
                
            driverController.y().whileTrue(new InstantCommand(()-> endEffector.ejectAlgae()))
                .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0)));
            driverController.x().whileTrue(new InstantCommand(()-> endEffector.ejectCoral()))
                .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0)));
                
            // Wrist
            driverController.rightTrigger().whileTrue(new RunCommand(()-> wrist.setWristMotor(driverController.getRightTriggerAxis()*0.20), wrist))
                .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
            driverController.leftTrigger().whileTrue(new RunCommand(()-> wrist.setWristMotor(-driverController.getLeftTriggerAxis()*0.20), wrist))
                .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));;
                
            // ===================================== Operator Controls =====================================
            // Elevator
            operatorController.povDown().onTrue(complexElevatorScoreCommand(elevatorPositions.L1)); 
            operatorController.povLeft().onTrue(complexElevatorScoreCommand(elevatorPositions.L2)); 
            operatorController.povRight().onTrue(complexElevatorScoreCommand(elevatorPositions.L3)); 
            operatorController.povUp().onTrue(complexElevatorScoreCommand(elevatorPositions.L4));
            // operatorController.back().onTrue(new InstantCommand(() -> elevator.zeroElevator()));

            // Wrist
            operatorController.a().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.WRIST_STOW)));
            operatorController.b().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.WRIST_PROCESSOR)));
            operatorController.x().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.GROUND_INTAKE)));
            operatorController.y().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.REEF_ACQUIRE_ANGLE)));

            // Default Commands
            // wrist.setDefaultCommand(new RunCommand(()-> wrist.setWristMotor(operatorController.getRightY()*0.20), wrist));
            // elevator.setDefaultCommand(new RunCommand(() -> elevator.setMotorSpeed(operatorController.getRightY()*0.50), elevator));
        }

        // Drivetrain tunning commands
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

    public Command complexElevatorScoreCommand(elevatorPositions position) {
        return new SequentialCommandGroup(
        wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen((new ElevatorToPosition(elevator,position)))
        .andThen(new WaitUntilCommand(elevator::atSetpoint))
        .andThen(new InstantCommand(()-> wrist.setAngle(WristConstants.WRIST_HOVER))));
    }

    public Command complexElevatorStowCommand(elevatorPositions position) {
        return new SequentialCommandGroup(
        wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen((new ElevatorToPosition(elevator,position))));
    }

    public Command complexProcessorCommand(elevatorPositions position) {
        return new SequentialCommandGroup(
        wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen(new ElevatorToPosition(elevator, position))
        .andThen(new WaitUntilCommand(elevator::atSetpoint))
        .andThen(new InstantCommand(()-> wrist.setAngle(WristConstants.WRIST_PROCESSOR)))); 
    }

    public Command complexIntakeCommand(elevatorPositions position) {
         return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen(new ElevatorToPosition(elevator, position))
        .andThen(new WaitUntilCommand(elevator::atSetpoint))
        .andThen(new InstantCommand(()-> wrist.setAngle(WristConstants.WRIST_INTAKE)))
        .andThen(endEffector.intakeCoralCommand()); 
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
