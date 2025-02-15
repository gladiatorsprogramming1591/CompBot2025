package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ElevatorToPosition;

import java.util.EnumMap;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.SparkBase.ControlType; 

public class ElevatorSubsystem extends SubsystemBase{
    DigitalInput coralBeamBreak; 

    SparkBase leader; 
    SparkBase follower;
    
    RelativeEncoder encoder; 
    SparkClosedLoopController controller; 

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
            MOTOR_CONFIG.follow(ELEVATOR_LEADER_CAN_ID),
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

    public void setPositionRotations(double rotations) {
        ClosedLoopSlot slot;
        if (rotations < getPositionRotations()) {
            controller.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, .623); // Down case; use max motion and slot 1
        } else {
            controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0); // Up case; use plain position control, slot 0
        }
    }

    public void ElevatorToPosition(elevatorPositions positions){
        double ref = mapEnc.get(positions); 
        ElevatorToPosition(positions); 
    }

}