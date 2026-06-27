package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final Debouncer jamDebouncer = new Debouncer(0.5,DebounceType.kBoth);
	
	LoggedNetworkNumber jamCurrentTune = new LoggedNetworkNumber("jamCurrent", 60);
	
	private double jamCurrent;
	private boolean jam;
	

	private double targetPercent = 0.0;
	private boolean enabled = false;

	public Intake(IntakeIO io) {
		this.io = io;
	}


	public void setPercent(double percent) {
		targetPercent = percent;
		enabled = true;
		io.setPercent(percent);
	}

	public void stop() {
		enabled = false;
		targetPercent = 0.0;
		io.stop();
	}

	public Command runIntake() {
		return Commands.startEnd(
				() -> setPercent(IntakeConstants.INTAKE_SPEED_PERCENT),
				this::stop,
				this);
	}
	public Command runIntake100() {
		return Commands.startEnd(
				() -> setPercent(100),
				this::stop,
				this);
	}

	public Command runOuttake() {
		return Commands.startEnd(
				() -> setPercent(-IntakeConstants.INTAKE_SPEED_PERCENT),
				this::stop,
				this);
	}
	public Command runOuttakeWithRollers() {
		return Commands.startEnd(
				() -> setPercent(-100),
				this::stop,
				this);
	}


	public Command stopIntake() {
		return Commands.runOnce(this::stop, this);
	}

	public Command smartIntakeHold() {
    	return Commands.repeatingSequence(

        Commands.run(() -> setPercent(IntakeConstants.INTAKE_SPEED_PERCENT), this)
            .until(() -> jam ),

        
    	Commands.run(() -> setPercent(-30), this)
            .withTimeout(0.75)
    ).finallyDo(() -> stop());
}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		if (enabled) {
			Logger.recordOutput("Intake/CommandedPercent", targetPercent);
		} else {
			Logger.recordOutput("Intake/CommandedPercent", 0.0);
		}
	    jamCurrent = inputs.currentAmps;
		jam = jamDebouncer.calculate( jamCurrent >= 70);
		Logger.recordOutput("Intake/Enabled", enabled);
		Logger.recordOutput("Intake/TargetPercent", targetPercent);
		Logger.recordOutput("Jam", jam);
		
	}
}
