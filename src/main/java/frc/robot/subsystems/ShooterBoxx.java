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

  public ShooterBoxx() {
  
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

public void RunShooter(double CustomSpeed ) {
 
shooterMotor.set(CustomSpeed);
 
}



public Command worksShoot() {

  return Commands.run(() -> shooterMotor.set(-.5), this);
};

public Command worksRegurgitate() {

  return Commands.run(() -> shooterMotor.set(.4), this);
};


public Command StopShooterMotor() {
return Commands.runOnce(() -> {
  shooterMotor.set(0);
}, this);
}

public Command SuckTillSensor() {
return Commands.run(() -> {
  if (ElevatorCoralSensorTriggered() && BoxxCoralSensorTriggered()) {
  shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
  } 
  else {
  shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
  }
}).until(() -> StopCoralIntake()); }

public Command SuckTillCoralSensorAuto() {
  return Commands.run(() -> {
    if (ElevatorCoralSensorTriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(0);
    }
    else {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }
  }).until(() -> StopCoralIntake());
}

public void SuckTillCoralSensorDerekSkillIssueFix() {
    if (ElevatorCoralSensorTriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered()) {
      shooterMotor.set(0);
    }
    else {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }
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
  }).until(() -> ElevatorCoralSensorTriggered());//.until(() -> StopCoralIntake());
}

public Command SpitTillSensor() {
return Commands.run(() -> {
  shooterMotor.set(Constants.shooterBoxxContants.kSpitSpeed);
}).until(() -> StopCoralShot()); }

public void shootCoralMethod() {
  shooterMotor.set(Constants.shooterBoxxContants.kSpitSpeed);
}

public void stopShooterMethod() {
  //shooterMotor.set(0);
  shooterMotor.stopMotor();
}


public Command BanditStopCommand() {
  return runOnce(() -> {
    shooterMotor.stopMotor();
  });
}

public Command BanditSetIntakeMotorCommand(double speed) {
  return run(() -> shooterMotor.set(speed));
}

public boolean BanditNoteSensorTriggered() {
  return (ElevatorCoralSensor.get() && !BoxxCoralSensor.get());
 // return (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered());
}

public boolean StopCoralIntake() {
  return (ElevatorCoralSensorUntriggered() && BoxxCoralSensorTriggered());
}

public boolean StopCoralShot() {
  return (BoxxCoralSensorUntriggered() && ElevatorCoralSensorUntriggered());
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

public Command IntakeDefaultCommand() {
  return Commands.run(() -> {
    if (BoxxCoralSensorUntriggered() && ElevatorCoralSensorUntriggered()) {
      shooterMotor.set(0);
    } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorUntriggered()) {
      shooterMotor.set(0);
    } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }

  }, this);
}

public Command IntakeSolosDefaultCommand() {
  return Commands.run(() -> {
    if (BoxxCoralSensorUntriggered() && ElevatorCoralSensorUntriggered()) {
      fireNow = false;
      shooterMotor.set(0);
    } else if (fireNow == true) {
      shooterMotor.set(shooterBoxxContants.kSpitSpeed);
    } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorUntriggered()) {
      shooterMotor.set(0);
    } else if (BoxxCoralSensorTriggered() && ElevatorCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kFinalSpeed);
    } else if (ElevatorCoralSensorTriggered()) {
      shooterMotor.set(Constants.shooterBoxxContants.kSuckSpeed);
    }

  }, this);
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
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Elevator Coral Sensor", ElevatorCoralSensorTriggered());
    // SmartDashboard.putBoolean("Boxx Coral Sensor", BoxxCoralSensorTriggered());
    isElevatorSensorTrue = !ElevatorCoralSensor.get();
    isBoxxSensorTrue = !BoxxCoralSensor.get();
  }
}
