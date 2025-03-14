// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.ShooterBoxx;

public class AlgaeSuperSystem extends SubsystemBase {
  /** Creates a new AlgaeSuperSystem. */

  private ElevatorSubsystem theElevator;
  private AlgaeSubsystem theClaw;
  private ShooterBoxx coralSensor;

  public AlgaeSuperSystem(ElevatorSubsystem theMoneyElevator, AlgaeSubsystem mrKrabsLikesMoney, ShooterBoxx mrKrabsIsClear) {
   theElevator = theMoneyElevator;
   theClaw = mrKrabsLikesMoney;
   coralSensor = mrKrabsIsClear;

  }


  public void ClawGoesForAlgae(boolean LowAlgae) {
    if (coralSensor.ElevatorCoralSensorUntriggered()) {
      if (LowAlgae == true) {
        theElevator.elevatorToSomething(2);
      } else {
        theElevator.elevatorToSomething(4);
      }
      theClaw.moveArmToPointMethod(AlgaeConstants.GrabPosition);
      theClaw.intakeAlgaeMethod();
    }
  }

  public Command ClawGoesForAlgaeCommand() {
   return Commands.run(()->{if (coralSensor.ElevatorCoralSensorUntriggered()) {
      int index = theElevator.getPlayer1LevelIndex();
      if (index == 2 || index == 3 || index == 6 || index == 7 || index == 10 || index == 11) { //change to just bigger number for 6 pole
        theElevator.elevatorToSomething(2);
      } else {
        theElevator.elevatorToSomething(4);
      }
      theClaw.moveArmToPointMethod(AlgaeConstants.GrabPosition);
      theClaw.intakeAlgaeMethod();
    }},theElevator, theClaw);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}