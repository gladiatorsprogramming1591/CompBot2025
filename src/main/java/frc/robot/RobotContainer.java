// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.robotInitConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.AutoReefPoseCommand;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FlapServo;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DynamicRateLimiter;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPositions;

public class RobotContainer {

    //Subsystems 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final EndEffector endEffector = robotInitConstants.isCompBot ? new EndEffector() : null;
	private final Wrist wrist = robotInitConstants.isCompBot ? new Wrist() : null;
	public final ElevatorSubsystem elevator = robotInitConstants.isCompBot ? new ElevatorSubsystem() : null;
    private final Climber climber = robotInitConstants.isCompBot ? new Climber(drivetrain) : null;
    private final FlapServo flapServo = robotInitConstants.isCompBot ? new FlapServo() : null;
    
    public PathPlannerPath startLineFCoralStartPath;
    private boolean prepL4Finished = false;
    
    private double MaxSpeed = robotInitConstants.isCompBot ? PoseidonTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
    : ChazTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 1 1/2 of a rotation per second max angular velocity
    private double maxSpeedPercent = 0.65;
    private double maxAngularRatePercent = 0.40;
    public static double kineticDeadband = KINETIC_DEADBAND;
    
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    // TODO: Issue: Deadband is not applied while bot is in motion (e.g. strafing while driving).
    // - Suspicion that output does not scale from 0 to max after the deadband (joystick is touchy). 
    // TODO: Idea?: Try changing OpenLoopVoltage to Velocity if we switch to FOC
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final DynamicRateLimiter xLimiter = new DynamicRateLimiter(1);
	private final DynamicRateLimiter yLimiter = new DynamicRateLimiter(1);
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
        
    private final SendableChooser<Command> autoChooser;
    private boolean aligning = false;
    private final PIDController headingController = new PIDController(0.1, 0, 0);
    private final SwerveRequest.RobotCentric reefAlign = new SwerveRequest.RobotCentric()
        // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
            public RobotContainer() {
                DataLogManager.start();
                registerNamedCommands();
                autoChooser = AutoBuilder.buildAutoChooser(); // A default auto can be passed in as parameter.
                SmartDashboard.putData("Auto Mode", autoChooser);
                SmartDashboard.putData(drivetrain.getCurrentCommand());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime()); 
                SmartDashboard.putNumber("", DriverStation.getStickButtons(1));
                
                if (robotInitConstants.isCompBot) configureBindingsComp(); else configureBindingsChassis();
                
            }
            
            private void configureBindingsComp() {
                // =====================================  Driver Controls  =====================================
                // Drivetrain
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                        // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(() ->
                                drive.withVelocityX(xLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftY(), driverController.getLeftX(), STATIC_DEADBAND, KINETIC_DEADBAND, true) * MaxSpeed * maxSpeedPercent,
                                                    INITIAL_LIMIT * Math.pow(LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), TIME_TO_STOP)) // Drive forward with negative Y (forward)
                                            .withVelocityY(yLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftX(), driverController.getLeftY(), STATIC_DEADBAND, KINETIC_DEADBAND, true) * MaxSpeed * maxSpeedPercent,
                                                    INITIAL_LIMIT * Math.pow(LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), TIME_TO_STOP)) // Drive left with negative X (left)
                                            .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), ROTATION_DEADBAND, maxAngularRatePercent) * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        )
                );
        
                // reset the field-centric heading on back button press
                driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                driverController.start().onTrue(new InstantCommand(()-> slowMode(true)))
                    .onFalse(new InstantCommand(()-> slowMode(false)));
                // driverController.rightStick().toggleOnTrue(new InstantCommand(() -> cardinalDriveMode(true))
                //     .handleInterrupt(() -> cardinalDriveMode(false)));
                
                // driverController.rightTrigger()
                //     .whileTrue(new RunCommand(()->AutoScoreAlign()))
                // 	.onFalse(new InstantCommand(()->{aligning=false;})); // this needs to be cleaned up ASAP
        
                // End Effector
                driverController.leftTrigger().whileTrue(complexIntakeCoral())
                    .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0),endEffector));
                driverController.rightBumper().whileTrue(endEffector.ejectAlgaeCommand())
                    .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
                
                driverController.leftBumper().whileTrue(endEffector.intakeAlgaeCommand())
                    .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0),endEffector));
                    driverController.rightTrigger().onTrue(ejectCoralAndStow())  
                    .onFalse(new InstantCommand(() -> endEffector.setCoralSpeed(0),endEffector));
        
                driverController.x().whileTrue(new AutoReefPoseCommand(drivetrain, reefAlign, this::driveX, this::driveY, this::driveT, ()->ReefSide.LEFT));
                driverController.y().whileTrue(new AutoReefPoseCommand(drivetrain, reefAlign, this::driveX, this::driveY, this::driveT, ()->ReefSide.RIGHT));
                driverController.a().whileTrue(new AutoReefPoseCommand(drivetrain, reefAlign, this::driveX, this::driveY, this::driveT, ()->FieldConstants.getNearestReefSide(drivetrain.getState().Pose)));
        
                // Wrist
                // driverController.rightTrigger().whileTrue(wrist.manualWristForwardMovement(driverController::getRightTriggerAxis))
                //     .onFalse(new InstantCommand(()-> wrist.setWristMotor(0),wrist));
                // driverController.leftTrigger().whileTrue(wrist.manualWristReverseMovement(driverController::getLeftTriggerAxis))
                //     .onFalse(new InstantCommand(()-> wrist.setWristMotor(0),wrist));
                    
                // ===================================== Operator Controls =====================================
                // Elevator
                operatorController.povDown().onTrue(prepElevatorScore(elevatorPositions.L1)); 
                operatorController.povLeft().onTrue(prepElevatorScoreL2(elevatorPositions.L2)); 
                operatorController.povRight().onTrue(prepElevatorScoreL4(elevatorPositions.L4)); 
                operatorController.povUp().onTrue(prepElevatorScore(elevatorPositions.L3));
                operatorController.leftBumper().onTrue(complexElevatorStowCommand(elevatorPositions.STOW));
                operatorController.back().onTrue(new InstantCommand(() -> elevator.zeroElevatorCommand()));
                operatorController.b().onTrue(complexProcessorCommand(elevatorPositions.STOW));
        
                // Wrist
                // operatorController.a().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.WRIST_STOW)));
                operatorController.x().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.GROUND_INTAKE)));
                operatorController.y().onTrue(new InstantCommand(()-> wrist.setAngle(WristConstants.REEF_ACQUIRE_ANGLE)));
        
                operatorController.rightTrigger().onTrue(complexHighAlgaeIntakeCommand(elevatorPositions.ALGAE_HIGH));
                operatorController.leftTrigger().onTrue(complexLowAlgaeIntakeCommand(elevatorPositions.ALGAE_LOW));
                // operatorController.rightTrigger().onTrue(flapServo.setFlapAngleCommand(()-> operatorController.getRightTriggerAxis()));
                operatorController.a().onTrue(flapServo.setFlapUpCommand());
                operatorController.rightBumper().onTrue(flapServo.setFlapDownCommand());
        
                // operatorController.rightTrigger().whileTrue(wrist.manualWristMovement(operatorController.getRightTriggerAxis()*0.20))
                //     .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
                // operatorController.leftTrigger().whileTrue(wrist.manualWristMovement(-operatorController.getLeftTriggerAxis()*0.20))
                //     .onFalse(new InstantCommand(()-> wrist.setWristMotor(0)));
        
                // Default Commands
                // wrist.setDefaultCommand(wrist.manualWristMovement(operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis() * 0.20));
                // operatorController.rightStick().toggleOnTrue(new RunCommand(() -> elevator.setMotorSpeed(operatorController.getRightY() * 0.50), elevator)
                //     .handleInterrupt(() -> elevator.setMotorSpeed(0)));
            
                climber.setDefaultCommand(climber.manualClimbMovement(()-> MathUtil.applyDeadband(operatorController.getRightY(), STATIC_DEADBAND), ()-> MathUtil.applyDeadband(operatorController.getLeftY(), STATIC_DEADBAND)));

                // Add a button to SmartDashboard that will create and follow an on-the-fly path
                // This example will simply move the robot 2m in the +X field direction
                SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
                Pose2d currentPose = drivetrain.getState().Pose;
                
                // The rotation component in these poses represents the direction of travel
                Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
                Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
                PathPlannerPath path = new PathPlannerPath(
                    waypoints, 
                    new PathConstraints(
                    4.0, 4.0, 
                    Units.degreesToRadians(360), Units.degreesToRadians(540)
                    ),
                    null, // Ideal starting state can be null for on-the-fly paths
                    new GoalEndState(0.0, currentPose.getRotation())
                );

                // Prevent this path from being flipped on the red alliance, since the given positions are already correct
                path.preventFlipping = true;

                AutoBuilder.followPath(path).schedule();
                }));

                drivetrain.registerTelemetry(logger::telemeterize);
            }
        
            private void configureBindingsChassis() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                        // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(() ->
                                drive.withVelocityX(xLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftY(), driverController.getLeftX(), STATIC_DEADBAND, KINETIC_DEADBAND, true) * MaxSpeed * maxSpeedPercent,
                                                    INITIAL_LIMIT, TIME_TO_STOP)) // Drive forward with negative Y (forward)
                                            .withVelocityY(yLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftX(), driverController.getLeftY(), STATIC_DEADBAND, KINETIC_DEADBAND, true) * MaxSpeed * maxSpeedPercent,
                                                    INITIAL_LIMIT, TIME_TO_STOP)) // Drive left with negative X (left)
                                            .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), ROTATION_DEADBAND, maxAngularRatePercent) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
                );
                
                // reset the field-centric heading on left bumper press
                driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                driverController.start().onTrue(new InstantCommand(()-> slowMode(true)))
                    .onFalse(new InstantCommand(()-> slowMode(false)));
        
                driverController.x().whileTrue(new AutoReefPoseCommand(drivetrain, reefAlign, this::driveXChassis, this::driveYChassis, this::driveT, ()->ReefSide.LEFT));
                driverController.y().whileTrue(new AutoReefPoseCommand(drivetrain, reefAlign, this::driveXChassis, this::driveYChassis, this::driveT, ()->ReefSide.RIGHT));
                driverController.a().whileTrue(new AutoReefPoseCommand(drivetrain, reefAlign, this::driveXChassis, this::driveYChassis, this::driveT, ()->FieldConstants.getNearestReefSide(drivetrain.getState().Pose)));    
        
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
                    maxAngularRatePercent = 0.1; 
                    maxSpeedPercent = 0.2; 
                }
                else {
                   maxAngularRatePercent = 1.0; 
                   maxSpeedPercent = 1.0;  
                }
            }
        
            public void cardinalDriveMode(boolean isCardinalDrive){
                if(isCardinalDrive){
                    kineticDeadband = 0.75; 
                }
                else {
                   kineticDeadband = KINETIC_DEADBAND;  
                }
            }
        
            public void prepL4Finished(boolean isFinished) {
                if (isFinished)
                prepL4Finished = true;
                else
                prepL4Finished = false;
            }


        public Command prepElevatorScore(elevatorPositions position) {
            System.out.println("Running complex score command"); 
            return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen((new ElevatorToPosition(elevator,position)))
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.HoverPositionCommandL2())
            .andThen(new RunCommand(() -> endEffector.setCoralSpeed(-0.15),endEffector).withTimeout(0.10))
            .andThen(new InstantCommand(() -> endEffector.setCoralSpeed(0.0),endEffector));
        }
        public Command prepElevatorScoreL2(elevatorPositions position) {
            System.out.println("Running complex score command"); 
            return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen((new ElevatorToPosition(elevator,position)))
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.HoverPositionCommand())
            .andThen(new RunCommand(() -> endEffector.setCoralSpeed(-0.15),endEffector).withTimeout(0.10))
            .andThen(new InstantCommand(() -> endEffector.setCoralSpeed(0.0),endEffector));
        }
    
        public Command prepElevatorScoreL4(elevatorPositions position) {
            System.out.println("Running complex score command"); 
            return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen((new ElevatorToPosition(elevator,position)))
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.L4HoverPositionCommand())
            .andThen(new RunCommand(() -> endEffector.setCoralSpeed(-0.15),endEffector).withTimeout(0.25)
                .alongWith(new InstantCommand(() -> prepL4Finished(true))))
            .andThen(new InstantCommand(() -> endEffector.setCoralSpeed(0.0),endEffector))
            .andThen(new InstantCommand(() -> prepL4Finished(true)));
        }

        public Command confirmPrepElevatorScoreL4(elevatorPositions position) {
            return new ConditionalCommand(
            new InstantCommand(()-> prepL4Finished = false),
            new ElevatorToPosition(elevator,position).onlyIf(()-> prepL4Finished)
                .andThen(new WaitUntilCommand(elevator::atSetpoint))
                .andThen(wrist.L4HoverPositionCommand())
                .andThen(new RunCommand(() -> endEffector.setCoralSpeed(-0.15),endEffector).withTimeout(0.25))
                .andThen(new InstantCommand(() -> endEffector.setCoralSpeed(0.0),endEffector))
                .andThen(new InstantCommand(() -> prepL4Finished(false))),
            () -> prepL4Finished);
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
    
        public Command complexLowAlgaeIntakeCommand(elevatorPositions position){
            return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen((new ElevatorToPosition(elevator,position)))
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.LowAlgaePositionCommand())
            .andThen(new WaitUntilCommand(wrist::atSetpoint));
        }
        public Command complexHighAlgaeIntakeCommand(elevatorPositions position){
            return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen((new ElevatorToPosition(elevator,position)))
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.HighAlgaePositionCommand())
            .andThen(new WaitUntilCommand(wrist::atSetpoint));
        }

        public Command complexIntakeCoral() { 
             return wrist.StowPositionCommand().andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen(new ElevatorToPosition(elevator, elevatorPositions.STOW))
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.IntakePositionCommand())
            .andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen(endEffector.intakeCoralCommand())
            .andThen(wrist.StowPositionCommand());
            // .andThen(new ElevatorToPosition(elevator, elevatorPositions.L2))
            // .andThen(new WaitUntilCommand(elevator::atSetpoint)); 
        }

        public Command autoComplexIntakeCommand() { // TODO: LP: check if elevator last pos was stow
             return new ElevatorToPosition(elevator, elevatorPositions.STOW)
            .andThen(new WaitUntilCommand(elevator::atSetpoint))
            .andThen(wrist.IntakePositionCommand())
            .andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen(endEffector.autoIntakeCoralCommand());
        }

        public Command ejectCoralAndStow() {
            return endEffector.ejectCoralCommand()
            .andThen(wrist.StowPositionCommand())
            .andThen(new WaitUntilCommand(wrist::atSetpoint))
            .andThen(new ElevatorToPosition(elevator, elevatorPositions.STOW));
        }

        public Command complexIntakeAlgae() {
            return endEffector.intakeAlgaeCommand().until(()-> endEffector.hasAlgae())
            .andThen(endEffector.holdAlgaeCommand());//TODO: Verify if this can be interupted
        }
    
        public void AutoScoreAlign()
        {
            // if(state.getDesiredElevatorLevel() == Level.L1)
            // {
            //     return;
            // }
            SwerveRequest.FieldCentric teleopDrive =                         
                drive.withVelocityX(xLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftY(), driverController.getLeftX(), STATIC_DEADBAND, KINETIC_DEADBAND, false) * MaxSpeed * maxSpeedPercent,
                                    INITIAL_LIMIT * Math.pow(LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), TIME_TO_STOP)) // Drive forward with negative Y (forward)
                            .withVelocityY(yLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftX(), driverController.getLeftY(), STATIC_DEADBAND, KINETIC_DEADBAND, false) * MaxSpeed * maxSpeedPercent,
                                    INITIAL_LIMIT * Math.pow(LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), TIME_TO_STOP)) // Drive left with negative X (left)
                            .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), ROTATION_DEADBAND, maxAngularRatePercent) * MaxAngularRate); // Drive counterclockwise with negative X (left)

            boolean isTeleopActive = (Math.abs(teleopDrive.VelocityX) > STATIC_DEADBAND ||
                                      Math.abs(teleopDrive.VelocityY) > STATIC_DEADBAND);
            // Get the current robot pose.
            Pose2d currentPose = drivetrain.getState().Pose;
            double desiredHeading = 0;
            double commandedX = 0;
            double commandedY = 0;
            double rotationCmd = 0;
            Pose2d nearestFace = FieldConstants.getNearestReefFace(currentPose);
            double distanceToReef = currentPose.getTranslation().getDistance(nearestFace.getTranslation());
            SmartDashboard.putNumber("Distance to Reef", distanceToReef);
            if (isTeleopActive) {
                // While driving, use teleop translation.
                commandedX = teleopDrive.VelocityX;
                commandedY = teleopDrive.VelocityY;
                // "Look at the reef": use the nearest reef face as reference.
                // Option 1: Use the rotation defined for the reef face.
                // Alternatively, you could compute the direction vector:
                // desiredHeading = nearestFace.getTranslation().minus(currentPose.getTranslation()).getAngle().getDegrees();
                double currentYaw = currentPose.getRotation().getDegrees();
                if(distanceToReef < 2)
                {
                desiredHeading = nearestFace.getRotation().plus(Rotation2d.k180deg).getDegrees();
                }
                else
                {
                    desiredHeading = currentYaw;
                }
                rotationCmd = headingController.calculate(currentYaw, desiredHeading);
                // Command teleop translation plus the rotation correction.
                drivetrain.setControl(
                    teleopDrive.withVelocityX(commandedX)
                            .withVelocityY(commandedY)
                            .withRotationalRate(rotationCmd * 0.5)
                );
            }
            else {
                if(!aligning && distanceToReef < 1) {
                    new AlignToReefCommand(drivetrain, () ->
                            drive.withVelocityX(xLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftY(), driverController.getLeftX(), STATIC_DEADBAND, KINETIC_DEADBAND, false) * MaxSpeed * maxSpeedPercent,
                                                INITIAL_LIMIT * Math.pow(LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), TIME_TO_STOP)) // Drive forward with negative Y (forward)
                                        .withVelocityY(yLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftX(), driverController.getLeftY(), STATIC_DEADBAND, KINETIC_DEADBAND, false) * MaxSpeed * maxSpeedPercent,
                                                INITIAL_LIMIT * Math.pow(LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), TIME_TO_STOP)) // Drive left with negative X (left)
                                        .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), ROTATION_DEADBAND, maxAngularRatePercent) * MaxAngularRate), // Drive counterclockwise with negative X (left)
                            reefAlign, true, 0).schedule();
                    aligning = true;
                }
	    }
    }

    	/**
	 * Limits the acceleration of the drivetrain so the robot can't tip over
	 * @return the X joystick value, limited
	 */
	public double driveX() {
		return xLimiter.calculate(-driverController.getLeftY()*MaxSpeed, DriveConstants.INITIAL_LIMIT*Math.pow(DriveConstants.LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), DriveConstants.TIME_TO_STOP);
	}
	public double driveXChassis() {
		return xLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftY(), driverController.getLeftX(), STATIC_DEADBAND, KINETIC_DEADBAND, true) * MaxSpeed * maxSpeedPercent,
        INITIAL_LIMIT, TIME_TO_STOP);
	}

	/**
	 * Limits the acceleration of the drivetrain so the robot can't tip over
	 * @return the Y joystick value, limited
	 */
	public double driveY() {
		return yLimiter.calculate(-driverController.getLeftX()*MaxSpeed, DriveConstants.INITIAL_LIMIT*Math.pow(DriveConstants.LIMIT_SCALE_PER_INCH, elevator.getPositionInches()), DriveConstants.TIME_TO_STOP);
	}
	public double driveYChassis() {
		return yLimiter.calculate(-drivetrain.apply2dDynamicDeadband(driverController.getLeftX(), driverController.getLeftY(), STATIC_DEADBAND, KINETIC_DEADBAND, true) * MaxSpeed * maxSpeedPercent,
        INITIAL_LIMIT, TIME_TO_STOP);
	}

	public double driveT() {
		return MathUtil.applyDeadband(-driverController.getRightX(), STATIC_DEADBAND, maxAngularRatePercent) * MaxAngularRate;
	}
    
    // 2.3976 is the y postion of StartLineToF
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }
    
    public void registerNamedCommands(){
        if (robotInitConstants.isCompBot) {
            NamedCommands.registerCommand("Intake Coral", complexIntakeCoral());
            NamedCommands.registerCommand("Intake Algae", complexIntakeAlgae());
            NamedCommands.registerCommand("Prep L4", prepElevatorScoreL4(elevatorPositions.L4).handleInterrupt((() -> endEffector.setCoralSpeed(0))).andThen(()-> System.out.println("Prep L4")));
            NamedCommands.registerCommand("Confirm Prep L4", confirmPrepElevatorScoreL4(elevatorPositions.L4).handleInterrupt((() -> endEffector.setCoralSpeed(0))).andThen(()-> System.out.println("Confirm Prep L4")));
            NamedCommands.registerCommand("Prep Auto L4", prepElevatorScoreL4(elevatorPositions.AUTO_L4).handleInterrupt((() -> endEffector.setCoralSpeed(0))).andThen(()-> System.out.println("Prep Auto L4")));
            NamedCommands.registerCommand("Confirm Prep Auto L4", confirmPrepElevatorScoreL4(elevatorPositions.AUTO_L4).handleInterrupt((() -> endEffector.setCoralSpeed(0))).andThen(()-> System.out.println("Confirm Prep Auto L4")));
            NamedCommands.registerCommand("Prep L3", prepElevatorScore(elevatorPositions.L3).andThen(()-> System.out.println("Prep L3")));
            NamedCommands.registerCommand("Prep L2", prepElevatorScore(elevatorPositions.L2).andThen(()-> System.out.println("Prep L2")));
            NamedCommands.registerCommand("Prep Algae High", complexHighAlgaeIntakeCommand(elevatorPositions.ALGAE_HIGH));
            NamedCommands.registerCommand("Prep Algae Low", complexHighAlgaeIntakeCommand(elevatorPositions.ALGAE_LOW));
            NamedCommands.registerCommand("Score Coral", endEffector.ejectCoralCommand()); 
            NamedCommands.registerCommand("Score Algae", endEffector.ejectAlgaeCommand()); //Untested
            NamedCommands.registerCommand("Stow", complexElevatorStowCommand(elevatorPositions.STOW));
            NamedCommands.registerCommand("Prep Processor", complexProcessorCommand(elevatorPositions.STOW));
            NamedCommands.registerCommand("Home Coral", endEffector.homingSequenceCommand()); 
            NamedCommands.registerCommand("Auto Intake Coral", autoComplexIntakeCommand());
            NamedCommands.registerCommand("Funnel Beam Break", new WaitUntilCommand(() -> endEffector.isCoralInFunnel()));
            NamedCommands.registerCommand("Stop intake", new InstantCommand(() -> endEffector.setCoralSpeed(0)));
            NamedCommands.registerCommand("startLineFCoral Start Path", Commands.runOnce(() -> AutoBuilder.followPath(startLineFCoralStartPath).schedule(), drivetrain));
        }
    }
}
