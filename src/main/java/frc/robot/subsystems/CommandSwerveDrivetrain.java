package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.robotInitConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utilities.DynamicRateLimiter;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    PowerDistribution m_pdh = robotInitConstants.isCompBot ? new PowerDistribution(1, ModuleType.kRev) : new PowerDistribution(21, ModuleType.kRev);

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    /* Keep track if we have initialized pose with vision once */
    private boolean m_hasAppliedVisionPose = false;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this
            )
    );
    
    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this
            )
    );
    
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this
            )

    );

    
    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    // PhotonCamera m_frontCamera;
    PhotonCamera m_leftCamera;
    PhotonCamera m_rightCamera;
    PhotonPoseEstimator[] m_photonPoseEstimators;
    AprilTagFieldLayout fieldLayout;
        private PhotonTrackedTarget lastTarget;
    
        // public static final Transform3d kFrontCameraLocation = robotInitConstants.isCompBot ? new Transform3d(
        //         new Translation3d(Units.inchesToMeters(4.5), Units.inchesToMeters(10.9),
        //             Units.inchesToMeters(9.25)),\[]

        //         new Rotation3d(0.0, 0.0, Math.toRadians(-25.0)))
        //         : new Transform3d(
        //             new Translation3d(Units.inchesToMeters(11.007), Units.inchesToMeters(0.1875),
        //             Units.inchesToMeters(5.789)),
        //         new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(0.0)));

        private static final Transform3d kleftCameraLocation = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.8), Units.inchesToMeters(12.45),
                Units.inchesToMeters(7.9)),
            new Rotation3d(0.0, Math.toRadians(-12.0 - 1.0), Math.toRadians(-29.0)));

        private static final Transform3d krightCameraLocation = new Transform3d(
            new Translation3d(Units.inchesToMeters(8.5), Units.inchesToMeters(-10.7),
                Units.inchesToMeters(7.9)),
            new Rotation3d(0.0, Math.toRadians(-12.0 - 1.0), Math.toRadians(30.0)));

    
        public static final double VISION_FIELD_MARGIN = 0.5;
        public static final double VISION_Z_MARGIN = 0.75;
        public static final double VISION_STD_XY_SCALE = 0.02;
        public static final double VISION_STD_ROT_SCALE = 0.035;
        public static final double FIELD_LENGTH = 16.5417;
        public static final double FIELD_WIDTH = 8.0136;
        
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules             Constants for each specific module
         */
        public CommandSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(drivetrainConstants, modules);
            if (Utils.isSimulation()) {
                startSimThread();
            }
            configureAutoBuilder();
            configureVision();
        }
        
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public CommandSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(drivetrainConstants, odometryUpdateFrequency, modules);
            if (Utils.isSimulation()) {
                startSimThread();
            }
            configureAutoBuilder();
            configureVision();
        }
        
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public CommandSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
            if (Utils.isSimulation()) {
                startSimThread();
            }
            configureAutoBuilder();
            configureVision();
        }
        
        private void configureAutoBuilder() {
            try {
                var config = RobotConfig.fromGUISettings();
                AutoBuilder.configure(
                        () -> getState().Pose,   // Supplier of current robot pose
                        this::resetPose,         // Consumer for seeding pose against auto
                        () -> getState().Speeds, // Supplier of current robot speeds
                        // Consumer of ChassisSpeeds and feedforwards to drive the robot
                        (speeds, feedforwards) -> setControl(
                                m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                        ),
                        new PPHolonomicDriveController(
                                // PID constants for translation
                                new PIDConstants(7.5, 0, 0),
                                // PID constants for rotation
                                new PIDConstants(3.5, 0.2, 0)
                        ),
                        config,
                        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                        this // Subsystem for requirements
                );
            } catch (Exception ex) {
                DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
            }
        }

        private void configureVision() {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
            // m_frontCamera = new PhotonCamera("Front");
            m_leftCamera = new PhotonCamera("Left");
            m_rightCamera = new PhotonCamera("Right");
    
            m_photonPoseEstimators = new PhotonPoseEstimator[] {
                new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    kleftCameraLocation
                ),
                // TODO: find out how to implent this to pose estimator
                // ,
                new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    krightCameraLocation)
            };
        }
        
        /**
         * Returns a command that applies the specified control request to this swerve drivetrain.
         *
         * @param request Function returning the request to apply
         * @return Command to run
         */
        public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> this.setControl(requestSupplier.get()));
        }
        
        /**
         * Runs the SysId Quasistatic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Quasistatic test
         * @return Command to run
         */
        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return m_sysIdRoutineToApply.quasistatic(direction);
        }
        
        /**
         * Runs the SysId Dynamic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Dynamic test
         * @return Command to run
         */
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return m_sysIdRoutineToApply.dynamic(direction);
        }
    
        public void updatePoseEstimationWithFilter() {
            Pose2d currentPose = getState().Pose;
            for (PhotonPoseEstimator poseEstimator : m_photonPoseEstimators) {
                // TODO: need to find the camera associated with a pose estimator, hard coded to front
                // var results = m_frontCamera.getAllUnreadResults();
                // if (m_photonPoseEstimators[0].equals(poseEstimator)) {
                //     List<PhotonPipelineResult> results = m_leftCamera.getAllUnreadResults();
                // } else if (m_photonPoseEstimators[1].equals(poseEstimator)) {
                //     List<PhotonPipelineResult> results = m_rightCamera.getAllUnreadResults();
                // } else {
                //     continue;
                // }
                var results = m_photonPoseEstimators[0].equals(poseEstimator) ? m_leftCamera.getAllUnreadResults() : m_rightCamera.getAllUnreadResults();
                PhotonPipelineResult result;
                Optional<EstimatedRobotPose> pose = null;
                if (!results.isEmpty()) {
                    // Camera processed a new frame since last
                    // Get the last one in the list.
                    result = results.get(results.size() - 1);    
                    if(!result.hasTargets()) continue;
                    lastTarget = result.getBestTarget();
                    double latency = result.metadata.getLatencyMillis();  
                    SmartDashboard.putNumber("Front Latency", latency); 
                    double latencyThreshold = 12.0;
                    SmartDashboard.putBoolean("Front Latency OK", latency > latencyThreshold);
                    pose = poseEstimator.update(result);
                }
                if (pose != null && pose.isPresent()) {
                    Pose3d pose3d = pose.get().estimatedPose;
                    Pose2d pose2d = pose3d.toPose2d();
                    if (
                        pose3d.getX() >= -VISION_FIELD_MARGIN &&
                        pose3d.getX() <= FIELD_LENGTH + VISION_FIELD_MARGIN &&
                        pose3d.getY() >= -VISION_FIELD_MARGIN &&
                        pose3d.getY() <= FIELD_WIDTH + VISION_FIELD_MARGIN &&
                        pose3d.getZ() >= -VISION_Z_MARGIN &&
                        pose3d.getZ() <= VISION_Z_MARGIN
                    ) {
                        double sum = 0.0;
                        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                            Optional<Pose3d> tagPose =
                                fieldLayout.getTagPose(target.getFiducialId());
                            if (tagPose.isEmpty()) continue;
                            sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                        }

                        int tagCount = pose.get().targetsUsed.size();
                        double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                        double xyStd = VISION_STD_XY_SCALE * stdScale;
                        double rotStd = VISION_STD_ROT_SCALE * stdScale;
                        //time this as well
                        SmartDashboard.putNumber("Vision x", pose2d.getX());
                        SmartDashboard.putNumber("Vision y", pose2d.getY());
                        SmartDashboard.putNumber("Vision rot", pose2d.getRotation().getDegrees());
                        SmartDashboard.putNumber("Vision ts", pose.get().timestampSeconds);
                        SmartDashboard.putNumber("Robot ts", Utils.getCurrentTimeSeconds());
                        SmartDashboard.putNumber("Vision xyStd", xyStd);
                        SmartDashboard.putNumber("Vision rotStd", rotStd);
                        // addVisionMeasurement(pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
                        if(!m_hasAppliedVisionPose) {
                            resetPose(pose2d);
                            m_hasAppliedVisionPose = true;
                        }

                        if(xyStd < 0.15){
                            addVisionMeasurement(pose2d, Utils.fpgaToCurrentTime(pose.get().timestampSeconds));
                        }

                        continue;
                    }
                }
            }
        }

        public PhotonTrackedTarget getLastTarget() {
            return lastTarget;
        }

    @Override
    public void periodic() {
        updatePoseEstimationWithFilter();
        
        // SmartDashboard.putBoolean("FrontConnected", m_frontCamera.isConnected());
        SmartDashboard.putBoolean("LeftConnected", m_leftCamera.isConnected());
        SmartDashboard.putBoolean("RightConnected", m_rightCamera.isConnected());
        try {
            //TODO: Learn more on why getAllUnreadResults() returns a list of PhotonPipelineResults instead
            // SmartDashboard.putBoolean("Back Latency OK", m_backamera.getLatestResult().getLatencyMillis() > latencyThreshold);
        } catch(Exception e) {

        }
        
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("FL Vel",  getState().ModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Vel",  getState().ModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Vel",  getState().ModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Vel",  getState().ModuleStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("FL Drive OCurrent",  getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("FR Drive OCurrent",  getModule(1).getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("BL Drive OCurrent",  getModule(2).getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("BR Drive OCurrent",  getModule(3).getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Heading", getState().RawHeading.getDegrees());
        SmartDashboard.putNumber("Swerve States", getState().ModuleStates.length);
        // SmartDashboard.putNumber("Vision Current", m_pdh.getCurrent(15));
        SmartDashboard.putBoolean("DIO channel 0", robotInitConstants.dIO_port.get());
    }

    /*
     * Return robot heading in degrees
     */
    public double getHeading() {
        return getState().RawHeading.getDegrees();
    }
    
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            
            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // TODO: Use getConfigurator to set different control loops and edit on smartDashBoard

    // TODO: Low Priority: Troubleshoot why setting idle mode to coast fights with the brake mode default

    // Mimics CTRE's deadband function, but with properly scaled output between 0-1 after deadband.
    public double apply2dDynamicDeadband(double axis, double perpendicularAxis, double deadband)
    {
        return apply2dDynamicDeadband(axis, perpendicularAxis, deadband, 0, false);
    }

    public double apply2dDynamicDeadband(double axis, double perpendicularAxis, double staticDeadband, double kineticDeadband, boolean squaredInputs)
    {
        SmartDashboard.putNumber("axis apply2dDynamicDeadband", axis);

        if (squaredInputs) {
            double axisSign = axis / Math.abs(axis);
            double perpendicularAxisSign = perpendicularAxis / Math.abs(perpendicularAxis);
            axis *= axis * axisSign;
            perpendicularAxis *= perpendicularAxis * perpendicularAxisSign;
            staticDeadband *= staticDeadband;
            kineticDeadband *= kineticDeadband;
        }

        double deadband = Math.abs(perpendicularAxis) > staticDeadband ? kineticDeadband : staticDeadband;
        double result = MathUtil.applyDeadband(axis, deadband);

        SmartDashboard.putNumber("axis squared apply2dDynamicDeadband", axis);
        SmartDashboard.putNumber("deadband apply2dDynamicDeadband", deadband);
        SmartDashboard.putNumber("result apply2dDynamicDeadband", result);
        return result;
    }

    public void teleopDrive(SwerveRequest.FieldCentric fieldCentric, double velocityXAxis, double velocityYAxis, double rotationalRateAxis, double staticDeadband, double kineticDeadband, double rotationDeadband,
                            double maxSpeed, double maxAngularRate, double maxSpeedPercent, double maxAngularRatePercent, boolean squaredInputs,
                            DynamicRateLimiter xLimiter, DynamicRateLimiter yLimiter, double initialLimit, double limitScalePerInch, double elevatorHeight, double timeToStop) {
        applyRequest(() ->
                fieldCentric.withVelocityX(xLimiter.calculate(-apply2dDynamicDeadband(velocityXAxis, velocityYAxis, staticDeadband, kineticDeadband, true) * maxSpeed * maxSpeedPercent,
                                    initialLimit * Math.pow(limitScalePerInch, elevatorHeight), timeToStop)) // Drive forward with negative Y (forward)
                            .withVelocityY(yLimiter.calculate(-apply2dDynamicDeadband(velocityYAxis, velocityXAxis, staticDeadband, kineticDeadband, true) * maxSpeed * maxSpeedPercent,
                                    initialLimit * Math.pow(limitScalePerInch, elevatorHeight), timeToStop)) // Drive left with negative X (left)
                            .withRotationalRate(-MathUtil.applyDeadband(rotationalRateAxis, rotationDeadband, maxAngularRatePercent) * maxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }
}
