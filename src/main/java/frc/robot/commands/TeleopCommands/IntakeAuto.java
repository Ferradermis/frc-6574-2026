package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class IntakeAuto extends SequentialCommandGroup {
    
    public IntakeAuto(AngularVelocity speed, AngularVelocity transitionSpeed) {
        addCommands(
            RobotContainer.intakePivot.setAngle(Degrees.of(-8.5)).withTimeout(0.1),
            new ParallelCommandGroup(
                RobotContainer.intakeMainRoller.setVelocity(speed).withTimeout(3)),
                //RobotContainer.transition.setVelocity(transitionSpeed)).withTimeout(3),
            new Intake(RPM.of(0), RPM.of(0)).withTimeout(0.1));
    }
}
