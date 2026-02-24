package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.GetBestAngle;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(AngularVelocity shooterSpeed, AngularVelocity shootertransSpeed, AngularVelocity transSpeed) {
        addCommands(
            new GetBestAngle(RobotContainer.drive).withTimeout(0.5),
            RobotContainer.shooter.setRightVelocity(shooterSpeed).withTimeout(0.5),
            RobotContainer.shooterTransition.setRightVelocity(shootertransSpeed).withTimeout(0.5),
            RobotContainer.transition.setVelocity(transSpeed).withTimeout(5),
            new ParallelCommandGroup(
                RobotContainer.shooter.setRightVelocity(RPM.of(0)),
                RobotContainer.shooterTransition.setRightVelocity(RPM.of(0)),
                RobotContainer.transition.setVelocity(RPM.of(0))
            ));
    }
}
