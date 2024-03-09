package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kIntake.kPivot.IntakePosition;
import frc.robot.Constants.kIntakeShooter.kHandOff;
import frc.robot.Constants.kIntakeShooter.kShootAmp;
import frc.robot.Constants.kIntakeShooter.kShootSpeaker;
import frc.robot.Constants.kShooter.kPivot.ShooterPosition;
import frc.robot.subsystems.HandoffRollers;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;

public class IntakeShooter {

  private final HandoffRollers handoffRollers;
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final ShooterFlywheels shooterFlywheels;
  private final ShooterPivot shooterPivot;

  public IntakeShooter(
      HandoffRollers handoffRollers,
      IntakePivot intakePivot,
      IntakeRollers intakeRollers,
      ShooterFlywheels shooterFlywheels,
      ShooterPivot shooterPivot) {
    this.handoffRollers = handoffRollers;
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
    this.shooterFlywheels = shooterFlywheels;
    this.shooterPivot = shooterPivot;
  }

  public Command intakeProcess() {
    return intakeRollers
        .intake()
        .deadlineWith(intakePivot.setIntakeDown(IntakePosition.DEPLOYED))
        .andThen(intakeRollers.index());
  }

  public Command autoIntake() {
    return intakeRollers
        .intake()
        .deadlineWith(intakePivot.setIntakeDown(IntakePosition.DEPLOYED))
        .andThen(
            intakeRollers
                .index()
                .alongWith(
                    intakePivot.setIntakeDown(IntakePosition.HOME).until(intakePivot::isHome)))
        .andThen(handOff());
  }

  public Command autoShoot() {
    return handoffRollers.feedShooterCommand().deadlineWith(intakeRollers.outtakeCommand());
  }

  public Command shootSpeaker() {
    return shooterFlywheels
        .shootTest(kShootSpeaker.shootVoltage)
        .raceWith(
            Commands.waitSeconds(kShootSpeaker.delay)
                .andThen(
                    handoffRollers
                        .feedShooterCommand()
                        .deadlineWith(intakeRollers.outtakeCommand())));
  }

  public Command handOff() {
    return intakeRollers
        .outtakeCommand()
        .raceWith(handoffRollers.intakeCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withTimeout(kHandOff.timeout);
  }

  public Command pivotAmp() {
    return shooterPivot
        .goToPositionCommand(ShooterPosition.AMP)
        .deadlineWith(intakeRollers.outtakeCommand());
  }

  public Command shootAmp() {
    return shooterFlywheels
        .shootTest(kShootAmp.shootVoltage)
        .raceWith(
            Commands.waitSeconds(kShootAmp.delay).andThen(handoffRollers.feedShooterCommand()));
  }
}
