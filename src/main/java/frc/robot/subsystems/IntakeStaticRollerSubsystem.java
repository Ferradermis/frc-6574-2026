package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeStaticRollerSubsystem extends SubsystemBase {
  double kP = 50;
  double kI = 0;
  double kD = 0;
  double kS = 0;
  double kV = 0;
  double kA = 0;

  private SmartMotorControllerConfig rollerConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              kP, kI, kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              kP, kI, kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          .withSimFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  @AutoLog
  public static class StaticRollerInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final StaticRollerInputsAutoLogged staticRollerInputs =
      new StaticRollerInputsAutoLogged();

  private TalonFX StaticRollerMotor = new TalonFX(0);

  private SmartMotorController StaticRollerMotorController =
      new TalonFXWrapper(StaticRollerMotor, DCMotor.getKrakenX44(1), rollerConfig);

  private FlyWheelConfig staticRollerConfig =
      new FlyWheelConfig(StaticRollerMotorController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the shooter.
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel staticRoller = new FlyWheel(staticRollerConfig);

  private void updateRightInputs() {
    staticRollerInputs.velocity = staticRoller.getSpeed();
    staticRollerInputs.setpoint =
        StaticRollerMotorController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    staticRollerInputs.volts = StaticRollerMotorController.getVoltage();
    staticRollerInputs.current = StaticRollerMotorController.getStatorCurrent();
  }

  public AngularVelocity getRightVelocity() {
    return staticRoller.getSpeed();
  }

  public Command setRightVelocity(AngularVelocity speed) {
    return staticRoller.setSpeed(speed);
  }

  public Command setRight(double dutyCycle) {
    return staticRoller.set(dutyCycle);
  }

  @Override
  public void periodic() {
    updateRightInputs();
    Logger.processInputs("RobotState/StaticRoller", staticRollerInputs);
    staticRoller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    staticRoller.simIterate();
  }
}
