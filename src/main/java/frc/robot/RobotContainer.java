// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.kSTOW;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.robotInitConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPositions;

public class RobotContainer {
    // TODO: Move this into constants
    public static final double CTRE_DEADBAND = 0.05;
    public static final double STATIC_DEADBAND = 0.10; // 10% Deadband before robot moves
    public static final double KINETIC_DEADBAND = 0.02; // 2% Deadband to perpendicular axis while robot is in motion
    public static double MAX_SPEED_PERCENT = 1.00;
    public static double MAX_ANGULAR_RATE_PERCENT = 1.00;

    //Subsystems 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final EndEffector endEffector = robotInitConstants.isCompBot ? new EndEffector() : null;
	private final Wrist wrist = robotInitConstants.isCompBot ? new Wrist() : null;
	public final ElevatorSubsystem elevator = robotInitConstants.isCompBot ? new ElevatorSubsystem() : null;


    private double MaxSpeed = robotInitConstants.isCompBot ? PoseidonTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : ChazTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 1 1/2 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // TODO: Issue: Deadband is not applied while bot is in motion (e.g. strafing while driving).
    // - Suspicion that output does not scale from 0 to max after the deadband (joystick is touchy). 
    // TODO: Idea?: Try changing OpenLoopVoltage to Velocity if we switch to FOC
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * CTRE_DEADBAND).withRotationalDeadband(MaxAngularRate * CTRE_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
        
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        registerNamedCommands();
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
                        // drive.withVelocityX(-MathUtil.applyDeadband(driverController.getLeftY(), 0, MAX_SPEED_PERCENT) * MaxSpeed) // Drive forward with negative Y (forward)
                        //             .withVelocityY(-MathUtil.applyDeadband(driverController.getLeftX(), 0, MAX_SPEED_PERCENT) * MaxSpeed) // Drive left with negative X (left)
                        //             .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), 0, MAX_ANGULAR_RATE_PERCENT) * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        drive.withVelocityX(-drivetrain.apply2dDynamicDeadband(driverController.getLeftY(), driverController.getLeftX(), STATIC_DEADBAND, KINETIC_DEADBAND, false) * MaxSpeed * MAX_SPEED_PERCENT) // Drive forward with negative Y (forward)
                                    .withVelocityY(-drivetrain.apply2dDynamicDeadband(driverController.getLeftX(), driverController.getLeftY(), STATIC_DEADBAND, KINETIC_DEADBAND, false) * MaxSpeed * MAX_SPEED_PERCENT) // Drive left with negative X (left)
                                    .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), STATIC_DEADBAND) * MaxAngularRate * MAX_ANGULAR_RATE_PERCENT) // Drive counterclockwise with negative X (left)
                )
        );
        
        // reset the field-centric heading on left bumper press
        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverController.start().onTrue(new InstantCommand(()-> slowMode(true)))
            .onFalse(new InstantCommand(()-> slowMode(false)));
        
        
        // Subsystems
        // All Poseidon-specific commands MUST be within if-statement, or a NullPointerException will be thrown.
        if (robotInitConstants.isCompBot) {    
            // =====================================  Driver Controls  =====================================
            // Elevator

            
            // End Effector
            // driverController.a().whileTrue(endEffector.intakeCoralCommand())
            //     .onFalse(new InstantCommand(()-> endEffector.setCoralSpeed(0)));
            driverController.a().whileTrue(complexIntakeCommand())
                .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0)));
            driverController.rightBumper().whileTrue(endEffector.intakeAlgaeCommand())
                .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
            
                
            driverController.leftBumper().whileTrue(endEffector.ejectAlgaeCommand())
                .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0)));
            driverController.x().onTrue(endEffector.ejectCoralCommand());                
            // Wrist
            driverController.rightTrigger().whileTrue(wrist.manualWristMovement(-driverController.getRightTriggerAxis()*-0.20))
                .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
            driverController.leftTrigger().whileTrue(wrist.manualWristMovement(-driverController.getLeftTriggerAxis()*0.20))
                .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
                
            // ===================================== Operator Controls =====================================
            // Elevator
            operatorController.povDown().onTrue(complexElevatorScoreCommand(elevatorPositions.L1)); 
            operatorController.povLeft().onTrue(complexElevatorScoreCommand(elevatorPositions.L2)); 
            operatorController.povRight().onTrue(complexElevatorScoreCommand(elevatorPositions.L4)); 
            operatorController.povUp().onTrue(complexElevatorScoreCommand(elevatorPositions.L3));
            operatorController.leftBumper().onTrue(complexElevatorStowCommand(elevatorPositions.STOW));
            operatorController.back().onTrue(new InstantCommand(() -> elevator.zeroElevatorCommand()));

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

    public void slowMode(boolean isSlow){
        if(isSlow){
            MAX_ANGULAR_RATE_PERCENT = 0.1; 
            MAX_SPEED_PERCENT = 0.2; 
        }
        else {
           MAX_ANGULAR_RATE_PERCENT = 1.0; 
           MAX_SPEED_PERCENT = 1.0;  
        }
    }

    public Command complexElevatorScoreCommand(elevatorPositions position) {
        System.out.println("Running complex score command"); 
        return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen((new ElevatorToPosition(elevator,position)))
        .andThen(new WaitUntilCommand(elevator::atSetpoint))
        .andThen(wrist.HoverPositionCommand());
    }

    public Command complexElevatorStowCommand(elevatorPositions position) {
        return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen((new ElevatorToPosition(elevator,position)));
    }

    public Command complexProcessorCommand(elevatorPositions position) {
        return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen(new ElevatorToPosition(elevator, position))
        .andThen(new WaitUntilCommand(elevator::atSetpoint))
        .andThen(wrist.ProcessorPositionCommand()); 
    }

    public Command complexIntakeCommand() { 
         return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
        .andThen(new InstantCommand(()-> System.out.println("Running complex intake command")))
        .andThen(new ElevatorToPosition(elevator, elevatorPositions.STOW))
        .andThen(new InstantCommand(()-> System.out.println("Running elevator stow command")))
        .andThen(new WaitUntilCommand(elevator::atSetpoint))
        .andThen(new InstantCommand(()-> System.out.println("Running wrist intake command")))
        .andThen(wrist.IntakePositionCommand())
        .andThen(new InstantCommand(()-> System.out.println("Running intake coral command")))
        .andThen(endEffector.intakeCoralCommand())
        .andThen(new InstantCommand(()-> System.out.println("Running stow command")))
        .andThen(wrist.StowPositionCommand())
        .andThen(new InstantCommand(()-> System.out.println("Running elevator L2 command")))
        .andThen(new ElevatorToPosition(elevator, elevatorPositions.L2))
        .andThen(new WaitUntilCommand(elevator::atSetpoint)); 
    }
    
    // 2.3976 is the y postion of StartLineToF
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }
    
    public void registerNamedCommands(){
        NamedCommands.registerCommand("IntakeCoral", complexIntakeCommand());
        NamedCommands.registerCommand("ComplexScoreCommand", complexElevatorScoreCommand(elevatorPositions.L4).andThen(()-> System.out.println("Complex Score Command")));
        NamedCommands.registerCommand("ScoreCoral", endEffector.ejectCoralCommand()); 
        NamedCommands.registerCommand("ComplexStow", complexElevatorStowCommand(elevatorPositions.STOW));
    }
}
