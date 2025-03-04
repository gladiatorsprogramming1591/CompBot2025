package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator.elevatorPositions;
import frc.robot.subsystems.Wrist.WristPosition;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;

public class RobotState {

    public enum GamePiece {CORAL, ALGAE, NOTHING};
    private GamePiece mode;
    public CurrentEndEffectorState currentEndEffectorState;
    public CurrentElevatorState currentElevatorState;
    public DesiredElevatorState desiredElevatorState;
    public CurrentWristPosition currentWristPosition;
    public DesiredWristPosition desiredWristPosition;

    public RobotState() {
        mode = GamePiece.CORAL;
        currentWristPosition = new CurrentWristPosition();
        currentElevatorState = new CurrentElevatorState();
        desiredWristPosition = new DesiredWristPosition();
        desiredElevatorState = new DesiredElevatorState();
        currentEndEffectorState = new CurrentEndEffectorState();
    }

    public void toggleMode() {
        if(currentEndEffectorState.gamePiece==GamePiece.NOTHING) {
            switch (mode) {
                case CORAL -> mode = GamePiece.ALGAE;
                default -> mode = GamePiece.CORAL;
            }
        }
    }

    public Command intakeCommand(EndEffector EndEffector) {
        return new ConditionalCommand(
            EndEffector.intakeCoralCommand(),
            EndEffector.intakeAlgaeCommand(),
            ()->mode==GamePiece.CORAL);
    }

    public Command scoreConfirmCommand(EndEffector EndEffector, Wrist wrist, Elevator elevator) {
        return new ConditionalCommand(
            EndEffector.ejectCoralCommand(),
            EndEffector.depositL1CoralCommand(),
            ()->{return getDesiredElevatorLevel() != elevatorPositions.L1;});
    }

    public Command stowCommand(Wrist wrist, Elevator elevator, boolean force) {
        return new ConditionalCommand(new InstantCommand(), 
                wrist.StowPositionCommand()
				.andThen(new WaitUntilCommand(wrist::atSetpoint))
				.andThen(elevator.stowCommand()
                .andThen(new WaitUntilCommand(elevator::atSetpoint))),

                ()->{return !(getDesiredElevatorLevel() != Level.L1 || force)});
    }

    public Command decreaseDeployedCoralLevelCommand() {
        return new InstantCommand(()->desiredElevatorState.deployedCoralLevel=desiredElevatorState.deployedCoralLevel.nextCoralLevelDown());
    }

    public Command increaseDeployedCoralLevelCommand() {
        return new InstantCommand(()->desiredElevatorState.deployedCoralLevel=desiredElevatorState.deployedCoralLevel.nextCoralLevelUp());
    }

    public Command setDeployedCoralLevelCommand(elevatorPositions level) {
        return new InstantCommand(()->desiredElevatorState.deployedCoralLevel=level);
    }

    public void toggleWrist(Wrist wrist) {
        desiredWristPosition.deployed = !desiredWristPosition.deployed;
        if(this.desiredWristPosition.deployed) {
            wristDown(wrist);
        } else {
            wristUp(wrist);
        }
    }

    /**
     * This is the default command for the wrist, constantly setting the real wrist to the ideal wrist's state
     * @param wrist the real wrist
     * @return the command
     */
    public void wristCommand(Wrist wrist) {
        if(this.desiredWristPosition.deployed) {

            wristDown(wrist);
        } else {
            wristUp(wrist);
        }
    }

    public Command EndEffectorCommand(EndEffector EndEffector) {
        return new RunCommand(()->{
            if(mode==GamePiece.ALGAE && currentEndEffectorState.gamePiece==GamePiece.NOTHING) {
                EndEffector.stopMotor();
            } else if(mode==GamePiece.ALGAE && currentEndEffectorState.gamePiece==GamePiece.ALGAE) {
                EndEffector.holdAlgae();
            } else {
                EndEffector.stopMotor();
            }
        }, EndEffector);
    }

    public void wristDown(Wrist wrist) {
        switch (this.mode) {
            case CORAL -> {
                switch (getDesiredElevatorLevel()) {
                    case L2, L3, L4 -> wrist.setPosition(WristPosition.DUNK);
                    case L1 -> wrist.setPosition(WristPosition.L1);
                    default -> wrist.setPosition(WristPosition.STOW);
                }
            }
            case ALGAE -> {
                switch (getDesiredElevatorLevel()) {
                    case ALGAE_HIGH -> wrist.setPosition(WristPosition.ALGAE_HIGH);
                    case ALGAE_LOW -> wrist.setPosition(WristPosition.ALGAE_LOW);
                    case STOW -> {
                        switch(this.currentEndEffectorState.gamePiece)
                        {
                            case ALGAE -> wrist.setPosition(WristPosition.PROCESSOR);
                            case NOTHING -> wrist.setPosition(WristPosition.GROUND_INTAKE);
                            default -> wrist.setPosition(WristPosition.STOW);
                        }
                    }
                    default -> wrist.setPosition(WristPosition.STOW);
                }
            }
            default -> wrist.setPosition(WristPosition.STOW);
        }
    }

    public void wristUp(Wrist wrist) {
        switch (this.mode) {
            case CORAL -> {
                switch (getDesiredElevatorLevel()) {
                    case L2, L3, L4 -> wrist.setPosition(WristPosition.HOVER);
                    case L1 -> wrist.setPosition(WristPosition.L1);
                    default -> wrist.setPosition(WristPosition.STOW);
                }
            }
            case ALGAE -> {
                switch (getDesiredElevatorLevel()) {
                    default -> wrist.setPosition(WristPosition.STOW);
                }
            }
            default -> wrist.setPosition(WristPosition.STOW);
        }
    }

    public Command deployCommand(Elevator elevator, Wrist wrist, CommandSwerveDrivetrain drive)
    {
        return wrist.StowPositionCommand()
				.andThen(new WaitUntilCommand(wrist::atSetpoint))
				.andThen(new InstantCommand(()->deployElevator(elevator, drive))
                .andThen(new WaitUntilCommand(elevator::atSetpoint))
                //TODO: Maybe push the coral back out a bit to prevent it from slapping the reef? Idk yet gotta test/tune
                .andThen(new InstantCommand(()->wristUp(wrist))));
    }

    public void deployElevator(Elevator elevator, CommandSwerveDrivetrain drive) {
        if(mode == GamePiece.CORAL)
        {
            elevator.setLevel(getDeployLevel());
        }
        else
        {
            elevator.setLevel(getAlgaeLevel(drive));
        }
    }

    public void setEndEffectorCurrentState(GamePiece gamePiece) {
        currentEndEffectorState.gamePiece = gamePiece;
    }

    public void setElevatorCurrentState(double currentInches) {
        currentElevatorState.height = currentInches;
    }

    public void setElevatorDesiredState(elevatorPositions desiredLevel) {
        desiredElevatorState.elevatorPositions = desiredLevel;
    }

    public void setCurrentWristPosition(double angle) {
        currentWristPosition.angle = angle;
    }

    public void setDesiredWristPosition(WristPosition Position) {
        desiredWristPosition.position = Position;
    }

    public elevatorPositions getDesiredElevatorLevel() {
        return desiredElevatorState.elevatorPositions;
    }

    public GamePiece getCurrentMode() {
        return mode;
    }

    /**
     * Returns the highest level the currently held game piece can be at.
     * Returns stow if the robot is holding nothing.
     * @return the highest possible target level
     */
    public elevatorPositions highestLevel() {
        switch (currentEndEffectorState.gamePiece) {
            case CORAL: return elevatorPositions.L4;
            case ALGAE: return elevatorPositions.STOW; //TODO: how to score algae
            default: return elevatorPositions.ALGAE_HIGH;
        }
    }

    private class CurrentEndEffectorState {
        private GamePiece gamePiece;

        public CurrentEndEffectorState() {
            gamePiece = GamePiece.CORAL;
        }
    }

    private class CurrentElevatorState {
        private double height;
    }

    private class DesiredElevatorState {
        private elevatorPositions elevatorPositions;
        private elevatorPositions deployedCoralLevel;
        private boolean deployed;

        public DesiredElevatorState() {
            deployed = false;
            deployedCoralLevel = elevatorPositions.L4;
            elevatorPositions = elevatorPositions.STOW;    
        }
    }

    private class CurrentWristPosition {
        private double angle;
    }

    private class DesiredWristPosition {
        private WristPosition position;
        private boolean deployed;

        public DesiredWristPosition() {
            position = WristPosition.STOW;
            deployed = false;
        }
    }

    public void putWristNetworkTable(NetworkTable table) {
        // table.putValue("Desire Deploy", NetworkTableValue.makeBoolean(desiredWristPosition.deployed));
        // table.putValue("Desired Position", NetworkTableValue.makeString(desiredWristPosition.position.name()));
    }

    public elevatorPositions getDesiredLevel()
    {
        return desiredElevatorState.elevatorPositions;
    }

    public elevatorPositions getDeployLevel() {
        return desiredElevatorState.deployedCoralLevel;
    }

    public WristPosition getDesiredWristPosition()
    {
        return desiredWristPosition.position;
    }

    public GamePiece getCurrentGamePiece() {
        return currentEndEffectorState.gamePiece;
    }

    public elevatorPositions getAlgaeLevel(CommandSwerveDrivetrain drive)
    {
        return FieldConstants.isAlgaeHigh(drive.getState().Pose) ? Level.ALGAE_HIGH : Level.ALGAE_LOW;
    }

    /**
     * Turns the initial position into center if we're in algae mode
     * @param initialPosition what to return if we are not in algae mode
     * @return the reef side we should go to
     */
    public ReefSide getReefPos(ReefSide initialPosition) {
        return mode==GamePiece.ALGAE ? ReefSide.CENTER : initialPosition;
    }

}
