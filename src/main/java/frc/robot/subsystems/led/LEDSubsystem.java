package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private static final int kPort = 9;
  private static final int kTotalLength = 44;
  private static final int kLogicalLength = 22;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_physicalBuffer;
  private final AddressableLEDBuffer m_logicalBuffer;

  private LEDPattern m_currentPattern;

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);

    m_physicalBuffer = new AddressableLEDBuffer(kTotalLength);
    m_logicalBuffer = new AddressableLEDBuffer(kLogicalLength);

    m_led.setLength(kTotalLength);
    m_led.start();

    // setDefaultCommand(solidColorCommand(Color.kBlack).withName("LED Off"));
  }

  @Override
  public void periodic() {
    if (m_currentPattern != null) {
      m_currentPattern.applyTo(m_logicalBuffer);
    }

    for (int i = 0; i < kLogicalLength; i++) {
      Color color = m_logicalBuffer.getLED(i);

      m_physicalBuffer.setLED(i, color);
      m_physicalBuffer.setLED(kTotalLength - 1 - i, color);
    }

    m_led.setData(m_physicalBuffer);
  }

  private void setPattern(LEDPattern pattern) {
    m_currentPattern = pattern;
  }

  public Command solidColorCommand(Color color) {
    LEDPattern solid = LEDPattern.solid(color).atBrightness(Percent.of(60));
    return run(() -> setPattern(solid)).withName("Solid Color").ignoringDisable(true);
  }

  public Command blinkCommand(Color color, double periodSeconds) {
    LEDPattern blinkPattern =
        LEDPattern.solid(color).blink(Seconds.of(periodSeconds)).atBrightness(Percent.of(60));
    return run(() -> setPattern(blinkPattern)).withName("Blink").ignoringDisable(true);
  }

  public Command strobeCommand(Color color) {
    return blinkCommand(color, 0.1).withName("Strobe").ignoringDisable(true);
  }

  public Command rainbowScrollCommand(double speed) {
    LEDPattern rainbowScroll =
        LEDPattern.gradient(GradientType.kContinuous, Color.kPurple, Color.kDarkBlue)
            .scrollAtRelativeSpeed(Percent.per(Second).of(speed))
            .atBrightness(Percent.of(60));
    return run(() -> setPattern(rainbowScroll)).withName("Rainbow Scroll").ignoringDisable(true);
  }

  public Command twoColorScrollCommand(double speed) {
    LEDPattern rainbowScroll =
        LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kPurple)
            .scrollAtRelativeSpeed(Percent.per(Second).of(speed))
            .atBrightness(Percent.of(60));
    return run(() -> setPattern(rainbowScroll)).withName("Rainbow Scroll").ignoringDisable(true);
  }

  public Command fallingBlocksCommand(Color color, double speedSecondsPerPixel) {
    return new FallingBlocksCommand(color, speedSecondsPerPixel).withName("Falling Blocks");
  }

  private class FallingBlocksCommand extends Command {

    private final Color m_color;
    private final double m_speed;
    private final Timer m_timer = new Timer();

    private int m_stackedCount;
    private int m_fallingPos;
    private double m_lastMoveTime;

    public FallingBlocksCommand(Color color, double speedSecondsPerPixel) {
      m_color = color;
      m_speed = speedSecondsPerPixel;
      addRequirements(LEDSubsystem.this);
    }

    @Override
    public void initialize() {
      m_currentPattern = null;
      m_stackedCount = 0;
      m_fallingPos = kLogicalLength - 1;
      m_timer.restart();
      m_lastMoveTime = m_timer.get();
    }

    @Override
    public void execute() {
      double currentTime = m_timer.get();

      if (currentTime - m_lastMoveTime > m_speed) {
        m_lastMoveTime = currentTime;
        m_fallingPos--;

        if (m_fallingPos < m_stackedCount) {
          m_stackedCount++;
          m_fallingPos = kLogicalLength - 1;

          if (m_stackedCount >= kLogicalLength) {
            m_stackedCount = 0;
          }
        }
      }

      for (int i = 0; i < kLogicalLength; i++) {
        if (i < m_stackedCount || i == m_fallingPos) {

          m_logicalBuffer.setLED(i, m_color);
        } else {
          m_logicalBuffer.setLED(i, Color.kBlack);
        }
      }
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
}
