// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterBoxxContants;

public class ShooterBoxx extends SubsystemBase {
  /** Creates a new ShooterBoxx. */
  private SparkMax shooterMotor = new SparkMax(Constants.shooterBoxxContants.ShooterMotorId, MotorType.kBrushless);
  private SparkMaxConfig shooterConfig = new SparkMaxConfig();

  // Sensor
  public DigitalInput BoxxCoralSensor = new DigitalInput(shooterBoxxContants.kBoxxCoralSensorChannel);
  public DigitalInput ElevatorCoralSensor = new DigitalInput(shooterBoxxContants.kElevatorCoralSensorChannel);

  public boolean fireNow = false;

  private boolean isElevatorSensorTrue = false;
  private boolean isBoxxSensorTrue = false;

  private ElevatorSubsystem elevator;

  public ShooterBoxx(ElevatorSubsystem fakeElevator) {
   elevator = fakeElevator;
  
    shooterConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    //Enable voltage compensation
    .voltageCompensation(Constants.shooterBoxxContants.kShooterMotorNominalVoltageConstant);
shooterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
  shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

public Command IntakeSolosDefaultCommand() { //yhhyitry commenting thus
  return Commands.run(() -> {
    if (BoxxCoralSensorUntriggered() && ElevatorCoralSensorUntriggered()) {
      fireNow = false;
      shooterMotor.set(0);
    } else if (fireNow == true) {
      shooterMotor.set(shooterBoxxContants.kSpitSpeed);
    } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorUntriggered()) {
      shooterMotor.set(0);
    } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorTriggered() && (elevator.isElevatorHome())) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorTriggered() && (elevator.isElevatorHome())) {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }
  }, this);
}

public void RunShooter(double CustomSpeed) {
  shooterMotor.set(-CustomSpeed);
}

public Command RunShooterJoyStick(double Joy) {
  return Commands.run(() -> shooterMotor.set(Joy));
}
public Command WorksShootCommand() {
  return Commands.run(() -> shooterMotor.set(-.5), this);
};

// public Command worksRegurgitate() {
//   return Commands.run(() -> shooterMotor.set(.4), this);
// };

public Command StopShooterMotorCommand() {
return Commands.runOnce(() -> {
  shooterMotor.set(0);
}, this);
}

public Command SuckTillSensorCommand() {
return Commands.run(() -> {
  if (ElevatorCoralSensorTriggered() && BoxxCoralSensorTriggered()) {
  shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
  } 
  else {
  shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
  }
}).until(() -> StopCoralIntake()); }

public Command SuckTillCoralSensorAutoCommand() {
  return Commands.run(() -> {
    if (ElevatorCoralSensorTriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(0); // does this even do anything lol
    }
    else {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }
  }).until(() -> StopCoralIntake());
}

public Command SuckTillElevatorSensorAuto() {
  return Commands.run(() -> {
    if (ElevatorCoralSensorTriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(0);
    }
    else {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }
  }).until(() -> ElevatorCoralSensorTriggered());
}

public Command SpitTillSensorCommand() {
return Commands.run(() -> {
  shooterMotor.set(Constants.shooterBoxxContants.kSpitSpeed);
}).until(() -> StopCoralShot()); }

public void ShootCoralMethod() {
  shooterMotor.set(Constants.shooterBoxxContants.kSpitSpeed);
}

public void StopShooterMethod() {
  //shooterMotor.set(0);
  shooterMotor.stopMotor();
}

public boolean StopCoralIntake() {
  return (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered());
}

public boolean StopCoralShot() {
  return (BoxxCoralSensorUntriggered() && ElevatorCoralSensorUntriggered());
}

public boolean getBoxxSensor() {
  return isBoxxSensorTrue;
}

public boolean getElevatorSensor() {
  return isElevatorSensorTrue;
}

public boolean getEitherSensor() {
  return (isBoxxSensorTrue || isElevatorSensorTrue);
}

public boolean BoxxCoralSensorUntriggered() {
  return BoxxCoralSensor.get();
}

public boolean ElevatorCoralSensorUntriggered() {
  return ElevatorCoralSensor.get();
}

public boolean BoxxCoralSensorTriggered() {
  return !BoxxCoralSensor.get();
}

public boolean ElevatorCoralSensorTriggered() {
  return !ElevatorCoralSensor.get();
}

// public Command IntakeDefaultCommand() {
//   return Commands.run(() -> {
//     if (BoxxCoralSensorUntriggered() && ElevatorCoralSensorUntriggered()) {
//       shooterMotor.set(0);
//     } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorUntriggered()) {
//       shooterMotor.set(0);
//     } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorTriggered()) {
//       shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
//     } else if (ElevatorCoralSensorTriggered()) {
//       shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
//     }

//   }, this);
// }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator Coral Sensor", ElevatorCoralSensorTriggered());
    SmartDashboard.putBoolean("Boxx Coral Sensor", BoxxCoralSensorTriggered());
    SmartDashboard.putBoolean("EitherSensorTrue", getEitherSensor());
    isElevatorSensorTrue = !ElevatorCoralSensor.get();
    isBoxxSensorTrue = !BoxxCoralSensor.get();
  }
}
