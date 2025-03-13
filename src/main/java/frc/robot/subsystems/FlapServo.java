package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ServoConstants;

public class FlapServo {
    
    private static Servo m_FlapServo;

  public FlapServo() {
      m_FlapServo = new Servo(ServoConstants.kFlapServoChannel);
  }

  public void periodic() {
    m_FlapServo.getAngle();
    SmartDashboard.putNumber("Flap Angle", m_FlapServo.getAngle());
  }

  public void setFlapServoAngle(double angle){
    m_FlapServo.setAngle(angle);
  }

  public Command setFlapUpCommand() {
    return new RunCommand(()-> setFlapServoAngle(ServoConstants.kServoUpAngle));
  }
  
  public Command setFlapDownCommand() {
    return new RunCommand(()-> setFlapServoAngle(ServoConstants.kServoDownAngle));
  }
  
  public Command setFlapAngleCommand(DoubleSupplier angleSupplier) {
    return new RunCommand(()-> setFlapServoAngle(angleSupplier.getAsDouble() * 150 + 30));
  }
}
