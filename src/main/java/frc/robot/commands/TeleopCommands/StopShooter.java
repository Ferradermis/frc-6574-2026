package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.GetBestAngle;

public class StopShooter extends SequentialCommandGroup {
    public StopShooter(AngularVelocity shooterSpeed, AngularVelocity shootertransSpeed, AngularVelocity transSpeed) {
        addCommands(
            RobotContainer.shooter.setRightVelocity(shooterSpeed).withTimeout(0.25),
            RobotContainer.shooterTransition.setRightVelocity(shootertransSpeed).withTimeout(0.5),
            RobotContainer.transition.setVelocity(transSpeed).withTimeout(0.5));
    }
}
