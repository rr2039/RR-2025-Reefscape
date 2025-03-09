package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.elevator.Elevator;

public class MechanismControl extends SubsystemBase {

  public enum State {
    levelOne,
    levelTwo,
    levelThree,
    home,
  }

  private State currentState = State.home;

  private final Elevator elevatorSubsystem;

  public MechanismControl(Elevator elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
  }

  public void periodic() {

    switch (currentState) {
      case home -> {
        if (elevatorSubsystem.isHomed()) {
          elevatorSubsystem.requestElevatorPosition(0);
        }
        elevatorSubsystem.requestElevatorPosition(-10);
        break;
      }

      case levelOne -> {
        elevatorSubsystem.requestElevatorPosition(elevatorSubsystem.get1stStageHeight());
        // wristSubsystem.requestWristPOS(WristConstants.coralAngle);
        break;
      }

      case levelTwo -> {
        elevatorSubsystem.requestElevatorPosition(elevatorSubsystem.get2ndStageHeight());
        // wristSubsystem.requestWristPOS(WristConstants.coralAngle);
        break;
      }

      case levelThree -> {
        elevatorSubsystem.requestElevatorPosition(0);
        // wristSubsystem.requestWristPOS(WristConstants.coralAngle);
        break;
      }
    }
  }

  public void setDesiredState(State desiredState) {

    currentState = desiredState;
  }
}
