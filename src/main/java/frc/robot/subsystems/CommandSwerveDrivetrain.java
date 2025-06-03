package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
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
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.robotInitConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utilities.DynamicRateLimiter;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;
import frc.robot.utilities.RobotPoseLookup;

import org.ejml.ops.MatrixIO;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
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
    Pose2d currentPose;
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

    Pose2d previousLeftPose2d = getState().Pose;
    Pose2d previousRightPose2d = getState().Pose;
    public RobotPoseLookup poseLookup = new RobotPoseLookup();
    // PhotonCamera m_frontCamera;
    private static int maxCameras = 2; 
    private static int cameraIdx = 0; 
    PhotonCamera[] cameras;
    // PhotonCamera m_rightCamera;
    PhotonPoseEstimator[] m_photonPoseEstimators;
    AprilTagFieldLayout fieldLayout;
        private PhotonTrackedTarget lastTarget;
        private int longDistangePoseEstimationCount = 0;

        boolean robotIsDisabled = DriverStation.isDisabled();
        
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
            if (robotInitConstants.isCompBot) {
                cameras = new PhotonCamera[] {
                    new PhotonCamera("Left"),
                    new PhotonCamera("Right")
                };
                m_photonPoseEstimators = new PhotonPoseEstimator[] {
                    new PhotonPoseEstimator(
                        fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        VisionConstants.kleftCameraLocation),
                    new PhotonPoseEstimator(
                        fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        VisionConstants.krightCameraLocation)
                };
            } else {
                maxCameras = 1;
                cameras = new PhotonCamera[] {
                    new PhotonCamera("Front"),
                };
                m_photonPoseEstimators = new PhotonPoseEstimator[] {
                    new PhotonPoseEstimator(
                        fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        VisionConstants.kFrontCameraLocation)
                };
            }    
        }

        public static void setCameraIdx(int cameraIdx) {
            CommandSwerveDrivetrain.cameraIdx = cameraIdx;
        }

        public static void setMaxCameras(int maxCameras) {
            CommandSwerveDrivetrain.maxCameras = maxCameras;
        }

        public static void onlyUseLeftCamera() {
            setCameraIdx(VisionConstants.LEFT_CAMERA_IDX);
            setMaxCameras(1);
        }

        public static void onlyUseRightCamera() {
            setCameraIdx(VisionConstants.RIGHT_CAMERA_IDX);
            setMaxCameras(2);
        }

        public static void useBothCameras() {
            setCameraIdx(0);
            setMaxCameras(2);
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
                
        /**
         * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
         * 10m for a short amount of time.
         *
         * @param pose Estimated robot pose.
         * @return Could be empty if there isn't a good reading.
         */
        private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose, int cameraIdx)
        {
            if (pose.isPresent())
            {
                double bestTargetAmbiguity = 1; // 1 is max ambiguity
                for (PhotonTrackedTarget target : pose.get().targetsUsed)
                {
                    double ambiguity = target.getPoseAmbiguity();
                    SmartDashboard.putNumber("Ambiguity " + target.getFiducialId() + "." + cameraIdx, ambiguity);
                    if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
                    {
                        bestTargetAmbiguity = ambiguity;
                    }
                }
                //ambiguity to high dont use estimate
                if (bestTargetAmbiguity > VisionConstants.maximumAmbiguity)
                {
                    // System.out.println("Ignoring pose on camera " + cameraIdx + " with ambiguity " + bestTargetAmbiguity);
                    return Optional.empty();
                }

                //est pose is very far from recorded robot pose
                if (PhotonUtils.getDistanceToPose(currentPose, pose.get().estimatedPose.toPose2d()) > 0.5)
                {
                    longDistangePoseEstimationCount++;

                    //if it calculates that were 10 meter away for more than 10 times in a row its probably right
                    if (longDistangePoseEstimationCount < 20)
                    {
                        return Optional.empty();
                    }
                } else
                {
                    longDistangePoseEstimationCount = 0;
                }
                return pose;
            }
            return Optional.empty();
        }
        
        public Optional<Pose2d> chooseBestPose(Pose2d leftPose, Pose2d rightPose,
                                               double leftxyStd, double leftrotStd, double rightxyStd, double rightrotStd, 
                                               double leftPoseTimestamp, double rightPoseTimestamp) {
            previousRightPose2d = poseLookup.lookup(rightPoseTimestamp);
            previousLeftPose2d = poseLookup.lookup(leftPoseTimestamp);
            return chooseBestPose(leftPose, rightPose, previousLeftPose2d, previousRightPose2d, leftxyStd, leftrotStd, rightxyStd, rightrotStd, leftPoseTimestamp, rightPoseTimestamp);
        }
        public Optional<Pose2d> chooseBestPose(Pose2d leftPose, Pose2d rightPose, Pose2d previousLeftPose2d, Pose2d previousRightPose2d, 
                                               double leftxyStd, double leftrotStd, double rightxyStd, double rightrotStd, 
                                               double leftPoseTimestamp, double rightPoseTimestamp) {
            Pose2d correctionPose = null;
            double xyStd = VisionConstants.MAX_STD;
            double rotStd = VisionConstants.MAX_STD;
            double timestamp = 0;
            if (leftPose != null && rightPose != null) {
                Pose2d leftToRightDiff = leftPose.relativeTo(rightPose);
                if (leftToRightDiff.getTranslation().getNorm() < 0.3 //TODO: Temporary value
                    && (Math.abs(leftToRightDiff.getRotation().getDegrees()) < 15)) {
                    correctionPose = leftPose.interpolate(rightPose, 0.5); // TODO: Idea: bias interpolation towards pose with least standard deviation
                    xyStd = MathUtil.interpolate(leftxyStd, rightxyStd, 0.5);
                    rotStd = MathUtil.interpolate(leftrotStd, rightrotStd, 0.5);
                    timestamp = MathUtil.interpolate(leftPoseTimestamp, rightPoseTimestamp, 0.5);
                } 
                else {
                    Pose2d leftDiff = leftPose.relativeTo(previousLeftPose2d);
                    Pose2d rightDiff = rightPose.relativeTo(previousRightPose2d);
                    double leftDist = leftDiff.getTranslation().getNorm();
                    double rightDist = rightDiff.getTranslation().getNorm();
                    if ((leftDist < 2.0 || robotIsDisabled) && leftDist <= rightDist) {
                        if (robotIsDisabled || Math.abs(leftDiff.getRotation().getDegrees()) < 15) {
                            correctionPose = leftPose;
                            xyStd = leftxyStd;
                            rotStd = leftrotStd;
                            timestamp = leftPoseTimestamp;
                        }
                    } else if ((rightDist < 2.0 || robotIsDisabled) && rightDist <= leftDist) {
                        if (robotIsDisabled || Math.abs(rightDiff.getRotation().getDegrees()) < 15) {
                            correctionPose = rightPose;
                            xyStd = rightxyStd;
                            rotStd = rightrotStd;
                            timestamp = rightPoseTimestamp;
                        }
                    }
                } 
            }
            else if (leftPose != null) {
                Pose2d leftDiff = leftPose.relativeTo(previousLeftPose2d);
                double leftDist = leftDiff.getTranslation().getNorm();
                if (leftDist < 2.0 || robotIsDisabled) {
                    if (robotIsDisabled || Math.abs(leftDiff.getRotation().getDegrees()) < 15) {
                        correctionPose = leftPose;
                        xyStd = leftxyStd;
                        rotStd = leftrotStd;
                        timestamp = leftPoseTimestamp;
                    }
                }
            } else if (rightPose != null) {
                Pose2d rightDiff = rightPose.relativeTo(previousRightPose2d);
                double rightDist = rightDiff.getTranslation().getNorm();
                if (rightDist < 2.0 || robotIsDisabled) {
                    if (robotIsDisabled || Math.abs(rightDiff.getRotation().getDegrees()) < 15) {
                        correctionPose = rightPose;
                        xyStd = rightxyStd;
                        rotStd = rightrotStd;
                        timestamp = rightPoseTimestamp;
                    }
                }
            } 
            if (correctionPose != null) {
                ChassisSpeeds currentSpeeds = getState().Speeds;
                boolean isMoving = Math.abs(currentSpeeds.vxMetersPerSecond) > 0.1
                    || Math.abs(currentSpeeds.vyMetersPerSecond) > 0.1
                    || Math.abs(currentSpeeds.omegaRadiansPerSecond) > 0.1;
                boolean allowYawCorrection = (DriverStation.isDisabled() || !isMoving);
                Pose2d visionMeasurement = allowYawCorrection
                    ? correctionPose
                    : new Pose2d(correctionPose.getTranslation(), getState().Pose.getRotation());
                // Only use standard deviations if less than MAX_STD.
                if ((xyStd + rotStd) >= VisionConstants.MAX_STD * 2)
                {
                    addVisionMeasurement(visionMeasurement, Utils.fpgaToCurrentTime(timestamp));
                } else {
                    addVisionMeasurement(visionMeasurement, Utils.fpgaToCurrentTime(timestamp), VecBuilder.fill(xyStd, xyStd, rotStd));
                }

                return Optional.of(visionMeasurement);
            }
            return Optional.empty();
        }
    
        public void updatePoseEstimation() {
            currentPose = getState().Pose;
            Optional<EstimatedRobotPose> pose = Optional.empty();
            Pose2d pose2d = null;
            double xyStd = VisionConstants.MAX_STD;
            double rotStd = VisionConstants.MAX_STD;
            double leftxyStd = VisionConstants.MAX_STD;
            double leftrotStd = VisionConstants.MAX_STD;
            double rightxyStd = VisionConstants.MAX_STD;
            double rightrotStd = VisionConstants.MAX_STD;
            Pose2d leftPose = null;
            Pose2d rightPose = null;
            double leftPoseTimestamp = 0;
            double rightPoseTimestamp = 0;
            for (int cameraIdx = CommandSwerveDrivetrain.cameraIdx; cameraIdx < maxCameras; cameraIdx++) {
                var results = cameras[cameraIdx].getAllUnreadResults();
                PhotonPipelineResult result;
                if (!results.isEmpty()) {
                    // Camera processed a new frame since last
                    // Get the last one in the list.
                    result = results.get(results.size() - 1);    
                    boolean hasTargets = result.hasTargets();
                    SmartDashboard.putBoolean("Vision Has Targets", hasTargets);
                    if(!hasTargets) continue;
                    lastTarget = result.getBestTarget();
                    double latency = result.metadata.getLatencyMillis();  
                    SmartDashboard.putNumber("Vision Latency", latency); 
                    double latencyThreshold = 12.0;
                    SmartDashboard.putBoolean("Vision Latency OK", latency > latencyThreshold);
                    pose = m_photonPoseEstimators[cameraIdx].update(result);
                    if (pose != null) {
                        pose = filterPose(pose, cameraIdx);
                        if (cameraIdx == VisionConstants.LEFT_CAMERA_IDX) {
                            leftPoseTimestamp = (result.getTimestampSeconds());
                        }
                        else if (cameraIdx == VisionConstants.RIGHT_CAMERA_IDX) {
                            rightPoseTimestamp = (result.getTimestampSeconds());
                        }
                    }    
                }
                if (pose != null && pose.isPresent()) {
                    Pose3d pose3d = pose.get().estimatedPose;
                    pose2d = pose3d.toPose2d();
                    if (cameraIdx == VisionConstants.LEFT_CAMERA_IDX) {
                        leftPose = pose2d;
                    }
                    else if (cameraIdx == VisionConstants.RIGHT_CAMERA_IDX) {
                        rightPose = pose2d;
                    }
                    else {
                        System.out.println("Too many cameras");
                    }
                    if (
                        pose3d.getX() >= -VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getX() <= VisionConstants.FIELD_LENGTH + VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() >= -VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() <= VisionConstants.FIELD_WIDTH + VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getZ() >= -VisionConstants.VISION_Z_MARGIN &&
                        pose3d.getZ() <= VisionConstants.VISION_Z_MARGIN
                    ) {
                        double sum = 0.0;
                        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                            int fiducialId = target.getFiducialId();
                            ignoredTags(fiducialId);
                            Optional<Pose3d> tagPose = fieldLayout.getTagPose(fiducialId);
                            if (tagPose.isEmpty()) continue;
                            sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                        }

                        double estPoseOffset = currentPose.getTranslation().getDistance(pose2d.getTranslation());
                        SmartDashboard.putNumber("estPoseOffset", estPoseOffset);

                        int tagCount = pose.get().targetsUsed.size();
                        double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                        xyStd = VisionConstants.VISION_STD_XY_SCALE * stdScale;
                        rotStd = VisionConstants.VISION_STD_ROT_SCALE * stdScale;

                        if (cameraIdx == VisionConstants.LEFT_CAMERA_IDX) {
                            leftxyStd = xyStd;
                            leftrotStd = rotStd;
                        }
                        else if (cameraIdx == VisionConstants.RIGHT_CAMERA_IDX) {
                            rightxyStd = xyStd;
                            rightrotStd = rotStd;
                        }
    
                        //time this as well
                        SmartDashboard.putNumber("Vision x", pose2d.getX());
                        SmartDashboard.putNumber("Vision y", pose2d.getY());
                        if (rightPose != null)
                        {
                            SmartDashboard.putNumber("right pose X", rightPose.getX());
                            SmartDashboard.putNumber("right pose Y", rightPose.getY());
                        }
                        if (leftPose != null)
                        {
                            SmartDashboard.putNumber("left pose X", leftPose.getX());
                            SmartDashboard.putNumber("left pose Y", leftPose.getY());
                        }
                        SmartDashboard.putNumber("Vision rot", pose2d.getRotation().getDegrees());
                        // Timestamps
                        SmartDashboard.putNumber("pose timestamp", pose.get().timestampSeconds);
                        SmartDashboard.putNumber("FPGA timestamp", Timer.getFPGATimestamp());
                        SmartDashboard.putNumber("FPGA converted timestamp", Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()));
                        SmartDashboard.putNumber("right pose timestamp", rightPoseTimestamp);
                        SmartDashboard.putNumber("left pose timestamp", leftPoseTimestamp);
                        SmartDashboard.putNumber("right pose converted timestamp", Utils.fpgaToCurrentTime(rightPoseTimestamp));
                        SmartDashboard.putNumber("left pose converted timestamp", Utils.fpgaToCurrentTime(leftPoseTimestamp));
                        SmartDashboard.putNumber("Utils current timestamp", Utils.getCurrentTimeSeconds());
                        SmartDashboard.putNumber("Vision xyStd", xyStd);
                        SmartDashboard.putNumber("Vision rotStd", rotStd);
                        // if(xyStd < 0.15){
                        //     addVisionMeasurement(pose2d, Utils.fpgaToCurrentTime(pose.get().timestampSeconds));
                        // }

                        continue;
                    }              
                }
            }
            
            // Optional<Pose2d> finalPose = chooseBestPose(leftPose, rightPose, previousLeftPose2d, previousRightPose2d, 
            //                                             leftxyStd, leftrotStd, rightxyStd, rightrotStd, 
            //                                             leftPoseTimestamp, rightPoseTimestamp); // used when NOT using poseLookUp
            Optional<Pose2d> finalPose = chooseBestPose(leftPose, rightPose,
                                                        leftxyStd, leftrotStd, rightxyStd, rightrotStd, 
                                                        leftPoseTimestamp, rightPoseTimestamp);
            if(finalPose.isPresent()) {
                if(!m_hasAppliedVisionPose) {
                    resetPose(finalPose.get());
                    m_hasAppliedVisionPose = true;
                }
                // // Updated previous poses to current pose before next call (When NOT using poseLookUp)
                // if (leftPose != null) previousLeftPose2d = leftPose;
                // if (rightPose != null) previousRightPose2d = rightPose;
            }
        }

        private void ignoredTags(int fiducialId) {
            switch(fiducialId){
                case 1:
                    return;
                case 2:
                    return;
                case 3:
                    return;
                case 4:
                    return;
                case 5:
                    return;
                case 12:
                    return;
                case 13:
                    return;
                case 14:
                    return;
                case 15:
                    return;
                case 16:
                    return;
            }
        }

        public PhotonTrackedTarget getLastTarget() {
            return lastTarget;
        }

    @Override
    public void periodic() {
        poseLookup.addPose(getState().Pose);
        updatePoseEstimation();
        
        // SmartDashboard.putBoolean("FrontConnected", m_frontCamera.isConnected());
        SmartDashboard.putBoolean("LeftConnected", cameras[0].isConnected());
         if(robotInitConstants.isCompBot){
        SmartDashboard.putBoolean("RightConnected", cameras[1].isConnected());
            
         }
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
        Pose2d nearestFace = FieldConstants.getNearestReefFace(currentPose);
        double distanceToReef = currentPose.getTranslation().getDistance(nearestFace.getTranslation());
        SmartDashboard.putNumber("Distance to Reef", distanceToReef);
        SmartDashboard.putNumber("Distance to Left", -1*Units.metersToInches(currentPose.relativeTo(FieldConstants.getNearestReefBranch(currentPose, ReefSide.LEFT)).getTranslation().getY()));
        SmartDashboard.putNumber("Distance to Right", -1*Units.metersToInches(currentPose.relativeTo(FieldConstants.getNearestReefBranch(currentPose, ReefSide.RIGHT)).getTranslation().getY()));
        SmartDashboard.putNumber("Current Fudge Factor Left", FieldConstants.getBranchFudgeFactor(1, ReefSide.LEFT));
    }

    /*
     * Return robot heading in degrees
     */
    public double getHeading() {
        return getState().RawHeading.getDegrees();
    }

    public double getPitch() {
        return getPigeon2().getRoll().getValueAsDouble();
    }

    public double getRoll() {
        return getPigeon2().getPitch().getValueAsDouble();
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

    private void updateStartLineFCoralStartPath(PathPlannerPath startLineFCoralStartPath) {
        Pose2d currentPose = getState().Pose;
        
        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Units.degreesToRadians(166.626)));
        Pose2d endPos = new Pose2d(new Translation2d(5.158, 3.040), new Rotation2d(Units.degreesToRadians(142.879)));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        startLineFCoralStartPath = new PathPlannerPath(
            waypoints, 
            new PathConstraints(
            3.0, 2.5, 
            Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(120.0)))
        );

        // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        startLineFCoralStartPath.preventFlipping = true;
    }


}
