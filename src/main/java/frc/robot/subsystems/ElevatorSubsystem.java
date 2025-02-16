package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
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

public class ElevatorSubsystem extends SubsystemBase{
    DigitalInput lowerLimit; 
    Trigger zeroTrigger; 

    SparkBase leader; 
    SparkBase follower;
    
    RelativeEncoder encoder; 
    SparkClosedLoopController controller; 
    SparkLimitSwitch bottomLimitSwitch;

    EnumMap<elevatorPositions, Double> mapEnc = new EnumMap<>(elevatorPositions.class);

    public enum elevatorPositions {
        STOW, 
        L1, 
        L2, 
        L3, 
        L4, 
        PROCESSOR, //same as stow height? 
        NETSHOOT //same as l4 height?
    }  

    public ElevatorSubsystem() {
        leader = new SparkFlex(ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless); 
        follower = new SparkFlex(ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
        follower.configure(
            MOTOR_CONFIG.follow(ELEVATOR_LEADER_CAN_ID, FOLLOWER_INVERTED_FROM_LEADER ),
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters); 

        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();
        
        mapEnc.put(elevatorPositions.STOW, kSTOW);
        mapEnc.put(elevatorPositions.L1, kL1);
        mapEnc.put(elevatorPositions.L2, kL2);
        mapEnc.put(elevatorPositions.L3, kL3);
        mapEnc.put(elevatorPositions.L4, kL4);
        mapEnc.put(elevatorPositions.PROCESSOR, kPROCESSOR);
        mapEnc.put(elevatorPositions.NETSHOOT, kNET);        
    }

    public void getHeight() { 

    }

    public void setMotorSpeed(double motorSpeed) {
        leader.set(motorSpeed);
    }

    public void elevatorOff() {
        leader.set(0);
    }

    public double getPositionRotations() {
        return encoder.getPosition(); 
    }

     /**
     * Returns the current height
     * @return the height, in inches
     */
    public double getPositionInches() {
        return getPositionRotations()*INCHES_PER_ROTATION+INITIAL_HEIGHT_INCHES;
    }

   /**
     * Calculates the number of rotations to be at the specified number of inches.
     * @return the number of rotations
     */
    public double inchesToRotations(double inches) {
        return (inches-INITIAL_HEIGHT_INCHES)/INCHES_PER_ROTATION;
    }


    public void setPositionRotations(double rotations) {
        if (rotations < getPositionRotations()) {
            controller.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, FF_DOWN); // Down case; use max motion and slot 1
        } else {
            controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0); // Up case; use plain position control, slot 0
        }
    }

    public void ElevatorToPosition(elevatorPositions positions){
        double refInches = mapEnc.get(positions); 
        setPositionRotations(inchesToRotations(refInches)); 
    }

    public Command zeroElevator() {
        return new InstantCommand(() -> encoder.setPosition(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator lowerLimit", bottomLimitSwitch.isPressed());
        SmartDashboard.putNumber("Elevator inches", getPositionInches());
        SmartDashboard.putNumber("Elevator current", leader.getOutputCurrent());
    }

}