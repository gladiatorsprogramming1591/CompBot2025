package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import java.util.EnumMap;

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

    EnumMap<elevatorPositions, Double> mapAbs = new EnumMap<>(elevatorPositions.class);

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
        leader = new SparkFlex(ElevatorConstants.ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless); 
        follower = new SparkFlex(ElevatorConstants.ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
        follower.configure(
            ElevatorConstants.MOTOR_CONFIG.follow(0),
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters); 

        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();
        
        mapAbs.put(elevatorPositions.STOW, ElevatorConstants.kSTOW);
        mapAbs.put(elevatorPositions.L1, ElevatorConstants.kL1);
        mapAbs.put(elevatorPositions.L2, ElevatorConstants.kL2);
        mapAbs.put(elevatorPositions.L3, ElevatorConstants.kL3);
        mapAbs.put(elevatorPositions.L4, ElevatorConstants.kL4);
        mapAbs.put(elevatorPositions.PROCESSOR, ElevatorConstants.kPROCESSOR);
        mapAbs.put(elevatorPositions.NETSHOOT, ElevatorConstants.kNET);        
    }
    
    public void setPosition() {

    }

    public void getHeight() { 

    }

    public void setMotorSpeed(double motorSpeed) {
        leader.set(motorSpeed);
    }

    public void elevatorOff() {
        leader.set(0);
    }

    public void elevatorToPosition(double ref){ 
        double m_targetPos = mapAbs.get(ref); 
        elevatorToPosition(ref);
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


}
