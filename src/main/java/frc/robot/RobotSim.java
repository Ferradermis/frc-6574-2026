package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotSim extends SubsystemBase {

  private LoggedMechanism2d shooterMechanism2d = new LoggedMechanism2d(Inches.of(27.5), Inches.of(27.5));
  private LoggedMechanismRoot2d shooterMechanismRoot = shooterMechanism2d.getRoot("ShooterRoot", 0, 0);

  // Ligaments from YAMS sim
  private LoggedMechanismLigament2d fuelRampLigament;
  private LoggedMechanismLigament2d mainRollerLigament;
  private LoggedMechanismLigament2d intakePivotLigament;
  private LoggedMechanismLigament2d staticRollerLigament;
  private LoggedMechanismLigament2d leftShooterLigament;
  private LoggedMechanismLigament2d rightShooterLigament;
  private LoggedMechanismLigament2d shooterPivotLigament;
  private LoggedMechanismLigament2d leftShooterTransitionLigament;
  private LoggedMechanismLigament2d rightShooterTransitionLigament;
  private LoggedMechanismLigament2d transitionLigament;

  public RobotSim() {
    fuelRampLigament = RobotContainer.fuelRamp.getGeneratedMechanism2d();
    mainRollerLigament = RobotContainer.intakeMainRoller.getGeneratedMechanism2d();
    intakePivotLigament = RobotContainer.intakePivot.getGeneratedMechanism2d();
    staticRollerLigament = RobotContainer.intakeStaticRoller.getGeneratedMechanism2d();
    leftShooterLigament = RobotContainer.shooter.getLeftShooterGeneratedMechanism2d();
    rightShooterLigament = RobotContainer.shooter.getRightShooterGeneratedMechanism2d();
    shooterPivotLigament = RobotContainer.shooterPivot.getGeneratedMechanism2d();
    leftShooterTransitionLigament = RobotContainer.shooterTransition.getLeftShooterGeneratedMechanism2d();
    rightShooterTransitionLigament = RobotContainer.shooterTransition.getRightShooterGeneratedMechanism2d();
    transitionLigament = RobotContainer.transition.getGeneratedMechanism2d();

    //Shooter Mechanism
    shooterMechanismRoot.append(shooterPivotLigament);
    shooterPivotLigament.append(leftShooterLigament);
    shooterPivotLigament.append(rightShooterLigament);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Mech2d/Shooter", shooterMechanism2d);
  }
}
