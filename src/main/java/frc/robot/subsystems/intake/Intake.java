package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final SlewRateLimiter percentLimiter = new SlewRateLimiter(
			100.0 / Math.max(IntakeConstants.INTAKE_OPEN_LOOP_RAMP_SECONDS, 0.02));

	private double targetPercent = 0.0;
	private boolean enabled = false;

	public Intake(IntakeIO io) {
		this.io = io;
	}

	public void setPercent(double percent) {
		targetPercent = percent;
		enabled = true;
	}

	public void stop() {
		enabled = false;
		targetPercent = 0.0;
		percentLimiter.reset(0.0);
		io.stop();
	}

	public Command runIntake() {
		return Commands.startEnd(
				() -> setPercent(IntakeConstants.INTAKE_SPEED_PERCENT),
				this::stop,
				this);
	}

	public Command stopIntake() {
		return Commands.runOnce(this::stop, this);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		if (enabled) {
			double commandedPercent = percentLimiter.calculate(targetPercent);
			io.setPercent(commandedPercent);
			Logger.recordOutput("Intake/CommandedPercent", commandedPercent);
		} else {
			Logger.recordOutput("Intake/CommandedPercent", 0.0);
		}

		Logger.recordOutput("Intake/Enabled", enabled);
		Logger.recordOutput("Intake/TargetPercent", targetPercent);
	}
}
