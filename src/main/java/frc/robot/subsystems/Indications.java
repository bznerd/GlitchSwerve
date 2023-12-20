package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indications.SystemState.SwerveState;
import frc.robot.utilities.LEDAnimations;
import frc.robot.utilities.LEDSubStrip;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class Indications extends SubsystemBase {
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(64);
  private final LEDSubStrip ledStrip = new LEDSubStrip(ledBuffer, 0, 63);
  private final List<Rule> rules = new ArrayList<Rule>();
  private final LEDAnimations animations = new LEDAnimations();

  public class SystemState {
    public class SwerveState {
      public enum Mode {
        DRIVE,
        HEADING_LOCK,
        POINT_OF_INTEREST,
        AUTO_DRIVE,
        PARK,
        IDLE
      }

      public Mode mode;
      public boolean boosting;

      public SwerveState() {
        mode = Mode.IDLE;
        boosting = false;
      }

      public SwerveState(Mode mode, boolean boosting) {
        this.mode = mode;
        this.boosting = boosting;
      }
    }

    public SwerveState swerveState;

    public SystemState() {
      swerveState = new SwerveState();
    }

    public SystemState(SwerveState swerveState) {
      this.swerveState = swerveState;
    }
  }

  private class Rule {
    public final int priority;
    private Predicate<SystemState> ruleCondition;
    private Consumer<LEDSubStrip> action;

    public Rule(int priority, Predicate<SystemState> ruleCondition, Consumer<LEDSubStrip> action) {
      this.priority = priority;
      this.ruleCondition = ruleCondition;
      this.action = action;
    }

    public boolean evaluate(SystemState state) {
      return ruleCondition.test(state);
    }

    public void apply(LEDSubStrip strip) {
      action.accept(strip);
    }
  }

  public Indications() {
    rules.add(
        new Rule(
            100,
            state -> state.swerveState.boosting,
            strip -> animations.monotone(strip, Color.kAliceBlue)));
    rules.add(
        new Rule(
            0,
            state -> state.swerveState.mode == SwerveState.Mode.IDLE,
            strip -> animations.monotone(strip, Color.kBlack)));
    rules.add(
        new Rule(
            20,
            state -> state.swerveState.mode == SwerveState.Mode.PARK,
            strip -> animations.monotone(strip, Color.kRed)));

    Collections.sort(rules, (rule1, rule2) -> rule2.priority - rule1.priority);
  }

  @Override
  public void periodic() {
    var exampleState = new SystemState();
    for (Rule rule : rules) {
      if (rule.evaluate(exampleState)) {
        rule.apply(ledStrip);
        break;
      }
    }
    animations.update();
  }
}
