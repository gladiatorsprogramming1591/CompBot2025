// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final Translation2d fieldCenter =
        new Translation2d(fieldLength / 2, fieldWidth / 2);
    public static final double startingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting line

    //Left Red Branches
    public static final double LEFT_RED_BRANCH_A = 0;
    public static final double LEFT_RED_BRANCH_C = 0;
    public static final double LEFT_RED_BRANCH_E = 0;
    public static final double LEFT_RED_BRANCH_G = 0;
    public static final double LEFT_RED_BRANCH_I = 0;
    public static final double LEFT_RED_BRANCH_K = 0;
    //Left Blue Branches
    public static final double LEFT_BLUE_BRANCH_A = 0;
    public static final double LEFT_BLUE_BRANCH_C = 0;
    public static final double LEFT_BLUE_BRANCH_E = 0;
    public static final double LEFT_BLUE_BRANCH_G = 0;
    public static final double LEFT_BLUE_BRANCH_I = 0;
    public static final double LEFT_BLUE_BRANCH_K = 0;
    //Right Red Branches
    public static final double RIGHT_RED_BRANCH_B = 0;
    public static final double RIGHT_RED_BRANCH_D = 0;
    public static final double RIGHT_RED_BRANCH_F = 0;
    public static final double RIGHT_RED_BRANCH_H = 0;
    public static final double RIGHT_RED_BRANCH_J = 0;
    public static final double RIGHT_RED_BRANCH_L = 0;
    //Right Blue Branches
    public static final double RIGHT_BLUE_BRANCH_B = 0;
    public static final double RIGHT_BLUE_BRANCH_D = 0;
    public static final double RIGHT_BLUE_BRANCH_F = 0;
    public static final double RIGHT_BLUE_BRANCH_H = 0;
    public static final double RIGHT_BLUE_BRANCH_J = 0;
    public static final double RIGHT_BLUE_BRANCH_L = 0;
    

    public static class Processor {
        public static final Pose2d centerFace =
            new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
            new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
            new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
            new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
            new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(291.176),
                Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
            new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90));
    }

    public static double getBranchFudgeFactor(int face, ReefSide side) {
        double ff;
        switch (side) {
            case LEFT -> {
                switch (face) {
                    /* Red */
                    case 0 -> {ff = LEFT_BLUE_BRANCH_A;} // A
                    case 5 -> {ff = LEFT_BLUE_BRANCH_C;} // C
                    case 4 -> {ff = LEFT_BLUE_BRANCH_E;} // E
                    case 3 -> {ff = LEFT_BLUE_BRANCH_G;} // G
                    case 2 -> {ff = LEFT_BLUE_BRANCH_I;} // I
                    case 1 -> {ff = LEFT_BLUE_BRANCH_K;} // K
                    /* Blue */
                    case 6 -> {ff = LEFT_RED_BRANCH_A;} // A
                    case 11 -> {ff = LEFT_RED_BRANCH_C;} // C
                    case 10 -> {ff = LEFT_RED_BRANCH_E;} // E
                    case 9 -> {ff = LEFT_RED_BRANCH_G;} // G
                    case 8 -> {ff = LEFT_RED_BRANCH_I;} // I
                    case 7 -> {ff = LEFT_RED_BRANCH_K;} // K
                    default -> {ff = 0;}
                }
                return -ff;
            }
            case RIGHT -> {
                switch (face) {
                    /* Red */
                    case 0 -> {ff = RIGHT_BLUE_BRANCH_B;} // B
                    case 5 -> {ff = RIGHT_BLUE_BRANCH_D;} // D
                    case 4 -> {ff = RIGHT_BLUE_BRANCH_F;} // F
                    case 3 -> {ff = RIGHT_BLUE_BRANCH_H;} // H
                    case 2 -> {ff = RIGHT_BLUE_BRANCH_J;} // J
                    case 1 -> {ff = RIGHT_BLUE_BRANCH_L;} // L
                    /* Blue */
                    case 6 -> {ff = RIGHT_RED_BRANCH_B;} // B
                    case 11 -> {ff = RIGHT_RED_BRANCH_D;} // D
                    case 10 -> {ff = RIGHT_RED_BRANCH_F;} // F
                    case 9 -> {ff = RIGHT_RED_BRANCH_H;} // H
                    case 8 -> {ff = RIGHT_RED_BRANCH_J;} // J
                    case 7 -> {ff = RIGHT_RED_BRANCH_L;} // L
                    default -> {ff = 0;}
                }
                return ff;
            }
            default -> {return 0;}
        }
    }

    public static class Reef {
        public static final Translation2d centerOfReef =
            new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final double faceToZoneLine =
            Units.inchesToMeters(12); // Side of the reef to the inside of the
        // reef zone line

        public static final Pose2d[] centerFaces =
            new Pose2d[12]; // Starting facing the driver station in clockwise
        // order
        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
            new ArrayList<>(); // Starting at the right
        // branch facing the
        // driver station in
        // clockwise

        static {
            // Initialize faces
            centerFaces[0] =
                new Pose2d(
                    Units.inchesToMeters(144.003),
                    Units.inchesToMeters(158.500),
                    Rotation2d.fromDegrees(180));
            centerFaces[1] =
                new Pose2d(
                    Units.inchesToMeters(160.373),
                    Units.inchesToMeters(186.857),
                    Rotation2d.fromDegrees(120));
            centerFaces[2] =
                new Pose2d(
                    Units.inchesToMeters(193.116),
                    Units.inchesToMeters(186.858),
                    Rotation2d.fromDegrees(60));
            centerFaces[3] =
                new Pose2d(
                    Units.inchesToMeters(209.489),
                    Units.inchesToMeters(158.502),
                    Rotation2d.fromDegrees(0));
            centerFaces[4] =
                new Pose2d(
                    Units.inchesToMeters(193.118),
                    Units.inchesToMeters(130.145),
                    Rotation2d.fromDegrees(-60));
            centerFaces[5] =
                new Pose2d(
                    Units.inchesToMeters(160.375),
                    Units.inchesToMeters(130.144),
                    Rotation2d.fromDegrees(-120));

            centerFaces[6] = centerFaces[0].rotateAround(fieldCenter, Rotation2d.k180deg);
            centerFaces[7] = centerFaces[1].rotateAround(fieldCenter, Rotation2d.k180deg);
            centerFaces[8] = centerFaces[2].rotateAround(fieldCenter, Rotation2d.k180deg);
            centerFaces[9] = centerFaces[3].rotateAround(fieldCenter, Rotation2d.k180deg);
            centerFaces[10] = centerFaces[4].rotateAround(fieldCenter, Rotation2d.k180deg);
            centerFaces[11] = centerFaces[5].rotateAround(fieldCenter, Rotation2d.k180deg);
            

            // Initialize branch positions
            for (int face = 0; face < centerFaces.length; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                System.out.println(getBranchFudgeFactor(face, ReefSide.LEFT));
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection = new Pose2d();
                    if (face < 6) {
                        poseDirection =
                            new Pose2d(centerOfReef, centerFaces[face].getRotation());
                    } else {
                        poseDirection =
                            new Pose2d(centerOfReef.rotateAround(fieldCenter, Rotation2d.k180deg),
                                centerFaces[face].getRotation());
                    }

                    double adjustX = Units.inchesToMeters(30.738); // Depth of branch from reef face
                    double adjustY = Units.inchesToMeters(6.469); // Offset from reef face
                                                                  // centerline to branch

                    fillRight.put(
                        level,
                        new Pose3d(
                            new Translation3d(
                                poseDirection
                                    .transformBy(
                                        new Transform2d(adjustX, adjustY + Units.inchesToMeters(getBranchFudgeFactor(face, ReefSide.RIGHT)), new Rotation2d()))
                                    .getX(),
                                poseDirection
                                    .transformBy(
                                        new Transform2d(adjustX, adjustY + Units.inchesToMeters(getBranchFudgeFactor(face, ReefSide.RIGHT)), new Rotation2d()))
                                    .getY(),
                                level.height),
                            new Rotation3d(
                                0,
                                Units.degreesToRadians(level.pitch),
                                poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                        level,
                        new Pose3d(
                            new Translation3d(
                                poseDirection
                                    .transformBy(
                                        new Transform2d(adjustX, -adjustY + Units.inchesToMeters(getBranchFudgeFactor(face, ReefSide.LEFT)), new Rotation2d()))
                                    .getX(),
                                poseDirection
                                    .transformBy(
                                        new Transform2d(adjustX, -adjustY + Units.inchesToMeters(getBranchFudgeFactor(face, ReefSide.LEFT)), new Rotation2d()))
                                    .getY(),
                                level.height),
                            new Rotation3d(
                                0,
                                Units.degreesToRadians(level.pitch),
                                poseDirection.getRotation().getRadians())));
                }
                branchPositions.add(fillRight);
                branchPositions.add(fillLeft);
            }


        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
            new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
            new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
        public static final Pose2d rightIceCream =
            new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
        L4(Units.inchesToMeters(72), -90),
        L3(Units.inchesToMeters(47.625), -35),
        L2(Units.inchesToMeters(31.875), -35),
        L1(Units.inchesToMeters(18), 0);

        ReefHeight(double height, double pitch)
        {
            this.height = height;
            this.pitch = pitch; // in degrees
        }

        public final double height;
        public final double pitch;
    }
    //Returns true if the closest reef face is high algae
    public static boolean isAlgaeHigh(Pose2d currentPose)
    {
        Rotation2d nearestReefRotation = getNearestReefFace(currentPose).getRotation();
        SmartDashboard.putNumber("Reef Rotation", nearestReefRotation.getDegrees());
        return (nearestReefRotation.equals(new Rotation2d(0)) || nearestReefRotation.equals(new Rotation2d(Math.toRadians(120))) || nearestReefRotation.equals(new Rotation2d(Math.toRadians(-120))));
    }

    public static Pose2d toRobotRelative(Pose2d robotPose, Pose2d goalPose)
    {
        Transform2d goalTransform = goalPose.minus(robotPose);
        return new Pose2d(goalTransform.getTranslation(), goalTransform.getRotation());
    }

    public static Pose2d getNearestReefFace(Pose2d currentPose)
    {
        return currentPose.nearest(List.of(FieldConstants.Reef.centerFaces));
    }

    public enum ReefSide {
        LEFT,
        CENTER,
        RIGHT
    }

    /**
     * Returns the nearest reef location of the specified type
     * @param currentPose the current post of the robot
     * @param side left (coral), center (algae), or right (coral)
     * @return the pose
     */
    public static Pose2d getNearestReefBranch(Pose2d currentPose, ReefSide side)
    {
        if(side==ReefSide.CENTER) return getNearestReefFace(currentPose);

        return FieldConstants.Reef.branchPositions
            .get(List.of(FieldConstants.Reef.centerFaces).indexOf(getNearestReefFace(currentPose))
                * 2 + (side == ReefSide.LEFT ? 1 : 0))
            .get(FieldConstants.ReefHeight.L1).toPose2d();
    }

    public static ReefSide getNearestReefSide(Pose2d robotPose) {
        return robotPose.minus(getNearestReefFace(robotPose)).getX()<0 ? ReefSide.LEFT : ReefSide.RIGHT;
    }

    public static Pose2d getNearestCoralStation(Pose2d currentPose)
    {
        if (currentPose.getTranslation().getX() > FieldConstants.fieldLength / 2) {
            if (currentPose.getTranslation().getY() > FieldConstants.fieldWidth / 2) {
                return FieldConstants.CoralStation.rightCenterFace
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
            } else {
                return FieldConstants.CoralStation.leftCenterFace
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
            }
        } else {
            if (currentPose.getTranslation().getY() > FieldConstants.fieldWidth / 2) {
                return FieldConstants.CoralStation.leftCenterFace;
            } else {
                return FieldConstants.CoralStation.rightCenterFace;
            }
        }
    }
}