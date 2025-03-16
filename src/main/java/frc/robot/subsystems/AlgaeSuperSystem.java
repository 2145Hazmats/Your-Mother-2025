// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ErrorConstants;
import frc.robot.Constants.elevatorConstants;
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

  public void ClawGoesForAlgaeOffFloor() {
    if (coralSensor.ElevatorCoralSensorUntriggered()) {
        theElevator.elevatorToLevel(0);
        theClaw.MoveArmToPointMethod(AlgaeConstants.FloorPosition);
        theClaw.IntakeAlgaeMethod();
    }
  }

  public void ClawPlaysProcessor() {
    if (coralSensor.ElevatorCoralSensorUntriggered()) {
        theElevator.elevatorToLevel(0); // idk where to put elevator
        theClaw.MoveArmToPointMethod(AlgaeConstants.ProcessorPosition);
        theClaw.RegurgitateAlgaeMethod();
    }
  }

  public void ClawPlaysNet() {
    if (coralSensor.ElevatorCoralSensorUntriggered()) {
      theElevator.elevatorToLevel(4);
      theClaw.MoveArmToPointMethod(AlgaeConstants.NetPosition);
      theClaw.RegurgitateAlgaeMethod();
    }

  }

  public void ClawGoesForAlgaeOffReef(boolean LowAlgae) {
    if (coralSensor.ElevatorCoralSensorUntriggered()) {
      if (LowAlgae == true) {
        theElevator.elevatorToLevel(2);
      } else {
        theElevator.elevatorToLevel(3);
      }
      theClaw.MoveArmToPointMethod(AlgaeConstants.GrabPosition);
      theClaw.IntakeAlgaeMethod();
    }
  }

  public Command ClawGoesForAlgaeCommand() {
   return Commands.run(()->{if (coralSensor.ElevatorCoralSensorUntriggered()) {
      int index = theElevator.getPlayer1LevelIndex();
      if (index == 2 || index == 3 || index == 6 || index == 7 || index == 10 || index == 11) { //change to just bigger number for 6 pole
        theElevator.elevatorToLevel(2);
      } else {
        theElevator.elevatorToLevel(3);
      }
      theClaw.MoveArmToPointMethod(AlgaeConstants.GrabPosition);
      theClaw.IntakeAlgaeMethod();
    }},theElevator, theClaw);
  }

  public Command Dealgify() {
    return Commands.run(()->{if (coralSensor.ElevatorCoralSensorUntriggered()) {
      int index = theElevator.getPlayer1LevelIndex();
      if (index == 2 || index == 3 || index == 6 || index == 7 || index == 10 || index == 11) { //change to just bigger number for 6 pole
        theElevator.elevatorToLevel(2);
        if (MathUtil.isNear(theElevator.getElevatorPosition(), elevatorConstants.L2Position, ErrorConstants.ElevatorError)) {
          theClaw.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition);
          theClaw.RegurgitateAlgaeMethod();
        }
      } else {
        theElevator.elevatorToLevel(3);
        if (MathUtil.isNear(theElevator.getElevatorPosition(), elevatorConstants.L3Position, ErrorConstants.ElevatorError)) {
          theClaw.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition);
          theClaw.RegurgitateAlgaeMethod();
        }
      }
    }});
  }

 /*  public Command DealgifyLowAlgaeCommand() { //no saftey 
    return Commands.startEnd(() -> {
      theElevator.elevatorToLevel(2);
      theClaw.IntakeAlgaeMethod();
      theClaw.MoveArmToPointMethod(AlgaeConstants.GrabPosition);
    
  }, theElevator, theClaw);
  }

  public Command DealgifyHighAlgaeCommand() { //no saftey 
    return Commands.startEnd(() -> {
      theElevator.elevatorToL3().andThen(Commands.parallel(
        theClaw.MoveArmToPointCommand(AlgaeConstants.DealgifyPosition)
      ))});
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}