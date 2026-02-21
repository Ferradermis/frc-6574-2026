package frc.robot.commands.TeleopCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.GetBestAngle;

public class Shoot extends SequentialCommandGroup {
    public Shoot(AngularVelocity shooterSpeed, AngularVelocity shootertransSpeed, AngularVelocity transSpeed) {
        addCommands(
            new GetBestAngle(RobotContainer.drive),
            RobotContainer.shooter.setRightVelocity(shooterSpeed).withTimeout(0.5),
            RobotContainer.shooterTransition.setRightVelocity(shootertransSpeed).withTimeout(0.5),
            RobotContainer.transition.setVelocity(transSpeed).withTimeout(0.5));
    }
}
