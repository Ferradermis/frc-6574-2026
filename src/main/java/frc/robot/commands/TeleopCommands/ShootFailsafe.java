package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFailsafe extends SequentialCommandGroup {
    public ShootFailsafe(Drive drive) {
        addCommands(
            RobotContainer.shooter.setRightVelocity(RPM.of(3000)).withTimeout(0.1),
            RobotContainer.shooterPivot.setAngle(Degrees.of(60)).withTimeout(0.5),
            RobotContainer.shooterTransition.setRightVelocity(Constants.MechanismConstants.SHOOTER_TRANSITION_SPEED).withTimeout(0.5),
            RobotContainer.transition.setVelocity(Constants.MechanismConstants.TRANSITION_SPEED_SHOOT).withTimeout(0.5),
            RobotContainer.intakeMainRoller.setVelocity(RPM.of(500)),
            new ShakeTheFuel(),
            RobotContainer.shooter.setRightVelocity(RPM.of(3000))); 
    }

}