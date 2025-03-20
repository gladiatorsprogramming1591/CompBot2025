// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands; 
  


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private int periodicCycle = 0; // periodic is called every 20ms by default

  public Robot() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    if (++periodicCycle == 10) // Only runs once every 200ms 10th periodic cycle (200ms)
    {
      Pose2d currentPose = m_robotContainer.drivetrain.getState().Pose;
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Units.degreesToRadians(166.626)));
      Pose2d endPos = new Pose2d(new Translation2d(5.158, 3.040), new Rotation2d(Units.degreesToRadians(142.879)));

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      m_robotContainer.startLineFCoralStartPath = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          3.0, 2.5, 
          Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(120.0)))
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      m_robotContainer.startLineFCoralStartPath.preventFlipping = true;
      periodicCycle = 0;
    }

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.endEffector.setCoralSpeed(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
