package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumMap;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorSubsystem extends SubsystemBase {
    DigitalInput lowerLimit;
    Trigger zeroTrigger;

    SparkFlex leader;
    SparkFlex follower;

    RelativeEncoder leadInternalEncoder;
    RelativeEncoder followerInternalEncoder;
    RelativeEncoder externalEncoder; // Through-bore encoder
    SparkClosedLoopController controller;
    SparkLimitSwitch bottomLimitSwitch;

    private double lastPos;
    private boolean printInternalEncZero = true; // Flag to indicate whether to print when we are zeroing during stow position
    private boolean printBothEncZero = true;

    EnumMap<elevatorPositions, Double> mapEnc = new EnumMap<>(elevatorPositions.class);

    public enum elevatorPositions {
        STOW,
        L1,
        L2,
        L3,
        L4,
        PROCESSOR, // same as stow height?
        NETSHOOT, // same as l4 height
        ALGAE_HIGH,
        ALGAE_LOW, 
        AUTO_L4
    }

    public ElevatorSubsystem() {
        leader = new SparkFlex(ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless);
        follower = new SparkFlex(ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
        leader.configure(MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        follower.configure(
                MOTOR_CONFIG.follow(ELEVATOR_LEADER_CAN_ID, FOLLOWER_INVERTED_FROM_LEADER),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        leadInternalEncoder = leader.getEncoder();
        followerInternalEncoder = follower.getEncoder();
        externalEncoder = follower.getExternalEncoder();
        controller = leader.getClosedLoopController();

        bottomLimitSwitch = leader.getReverseLimitSwitch();
        zeroTrigger = new Trigger(this::isElevatorNotAtBottom);
        zeroTrigger.onFalse(zeroElevatorInternalEncCommand().alongWith(zeroElevatorInternalEncCommand()));

        lastPos = kSTOW;

        mapEnc.put(elevatorPositions.STOW, kSTOW);
        mapEnc.put(elevatorPositions.L1, kL1);
        mapEnc.put(elevatorPositions.L2, kL2);
        mapEnc.put(elevatorPositions.L3, kL3);
        mapEnc.put(elevatorPositions.L4, kL4);
        mapEnc.put(elevatorPositions.AUTO_L4, AUTO_L4);
        mapEnc.put(elevatorPositions.PROCESSOR, kPROCESSOR);
        mapEnc.put(elevatorPositions.NETSHOOT, kNET);
        mapEnc.put(elevatorPositions.ALGAE_HIGH, ALGAE_HIGH);
        mapEnc.put(elevatorPositions.ALGAE_LOW, ALGAE_LOW);
    }

    private boolean isElevatorNotAtBottom() {
        return !bottomLimitSwitch.isPressed();
    }

    public void getHeight() {

    }

    public void setMotorSpeed(double motorSpeed) {
        leader.set(motorSpeed);
    }

    public void elevatorOff() {
        leader.set(0);
    }

    public double getInternalPositionRotations() {
        return leadInternalEncoder.getPosition();
    }

    public double getExternalPositionRotations() {
        return externalEncoder.getPosition();
    }

    /**
     * Returns the current height
     * 
     * @return the height, in inches
     */
    public double getInternalPositionInches() {
        return getInternalPositionRotations() * INCHES_PER_INTERNAL_ROTATION + INITIAL_HEIGHT_INCHES;
    }

    public double getExternalPositionInches() {
        return getExternalPositionRotations() * INCHES_PER_EXTERNAL_ROTATION + INITIAL_HEIGHT_INCHES;
    }

    /**
     * Calculates the number of rotations to be at the specified number of inches.
     * 
     * @return the number of rotations
     */
    public double inchesToInternalRotations(double inches) {
        return (inches - INITIAL_HEIGHT_INCHES) / INCHES_PER_INTERNAL_ROTATION;
    }

    public double inchesToExternalRotations(double inches) {
        return (inches - INITIAL_HEIGHT_INCHES) / INCHES_PER_EXTERNAL_ROTATION;
    }

    public void setPositionInternalRotations(double rotations) {
        if (rotations < getInternalPositionRotations()) {
            controller.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, FF_DOWN);
        } else {
            controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF_UP);
        }
    }
    
    public void setPositionExternalRotations(double rotations) {
        rotations *= (INCHES_PER_EXTERNAL_ROTATION/INCHES_PER_INTERNAL_ROTATION);
        if (rotations < getExternalPositionRotations()) {
            controller.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, FF_DOWN);
        } else {
            controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF_UP);
        }
    }

    public boolean atSetpointInternalEnc() {
        boolean atTarget = Math.abs(getInternalPositionInches() - lastPos) < TOLERANCE_INCHES;
        if (atTarget)
            System.out.println("Elevator at setpoint (Internal Enc)");
        return atTarget;
    }

    public boolean atSetpointExternalEnc() {
        boolean atTarget = Math.abs(getExternalPositionInches() - lastPos) < TOLERANCE_INCHES;
        if (atTarget)
            System.out.println("Elevator at setpoint (External Enc)");
        return atTarget;
    }

    public void ElevatorToPositionInternalEnc(elevatorPositions positions) {
        lastPos = mapEnc.get(positions);
        setPositionInternalRotations(inchesToInternalRotations(lastPos));
    }

    public void ElevatorToPositionExternalEnc(elevatorPositions positions) {
        lastPos = mapEnc.get(positions);
        setPositionExternalRotations(inchesToExternalRotations(lastPos));
    }

    public Command zeroElevatorInternalEncCommand() {
        return new InstantCommand(() -> {
            System.out.println("ZeroInternalEncCommand");
            leadInternalEncoder.setPosition(0);
        });
    }

    public Command zeroElevatorExternalEncCommand() {
        return new InstantCommand(() -> {
            System.out.println("ZeroExternalEncCommand");
            externalEncoder.setPosition(0);
        });
    }

    public void zeroElevatorInternalEnc() {
        leadInternalEncoder.setPosition(0);
    }

    public void zeroElevatorExternalEnc() {
        externalEncoder.setPosition(0);
    }

    public void zeroElevatorBothEnc() {
        zeroElevatorInternalEnc();
        zeroElevatorExternalEnc();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator lowerLimit", bottomLimitSwitch.isPressed());
        SmartDashboard.putNumber("Elevator inches (Internal Enc)", getInternalPositionInches());
        SmartDashboard.putNumber("Elevator inches (External Enc)", getExternalPositionInches());
        SmartDashboard.putNumber("Elevator current", leader.getOutputCurrent());
        SmartDashboard.putNumber("Elevator leader Vel", leadInternalEncoder.getVelocity());
        SmartDashboard.putNumber("Elevator Vel (External Enc)", externalEncoder.getVelocity());
        SmartDashboard.putNumber("Follower Output Current", follower.getOutputCurrent());
        SmartDashboard.putNumber("Elevator follower vel", followerInternalEncoder.getVelocity());
        SmartDashboard.putNumber("Elevator lastPos", lastPos);
        SmartDashboard.putNumber("Elevator throgh-bore enc inches", getExternalPositionInches());

        if ((lastPos == kSTOW) && (getInternalPositionInches() <= kSTOW + TOLERANCE_INCHES + 0.05)) {
            if (printInternalEncZero == true) {
                System.out.println("Zeroing Elevator internal encoder");
                printInternalEncZero = false;
            }
            leader.stopMotor();
            zeroElevatorInternalEnc();
        } else {
            printInternalEncZero = true;
        }

        if (bottomLimitSwitch.isPressed() && lastPos == kSTOW) {
            if (printBothEncZero == true) {
                System.out.println("Zeroing Elevator encoders (Both External & Internal)");
                printBothEncZero = false;
            }
            zeroElevatorBothEnc();
        } else {
            printBothEncZero = true;
        }
    }

}