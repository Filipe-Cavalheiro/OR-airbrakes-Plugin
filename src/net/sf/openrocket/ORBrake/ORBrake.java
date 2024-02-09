package net.sf.openrocket.ORBrake;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;

public class ORBrake extends AbstractSimulationExtension {
	/**
	 * GUI back end and listener instantiation.
	 */
	@Override
	public String getName() {
		return "HPRC - ORBrake";
	}

	@Override
	public String getDescription() {
		return "Controls vehicle air brakes to dynamically adjust vehicle drag.";
	}

	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(
				new ORBrakeSimulationListener(getSetpoint(), getKp(), getKi(), getKd(), getTau(), getCd(), getMass(), getArea()));
	}

	public double getSetpoint() {
		return config.getDouble("setpoint", 3000.0);
	}

	public void setSetpoint(double setpoint) {
		config.put("setpoint", setpoint);
		fireChangeEvent();
	}

	public double getKp() {
		return config.getDouble("Kp", 8.0);
	}

	public void setKp(double Kp) {
		config.put("Kp", Kp);
		fireChangeEvent();
	}

	public double getKi() {
		return config.getDouble("Ki", 1.0);
	}

	public void setKi(double Ki) {
		config.put("Ki", Ki);
		fireChangeEvent();
	}

	public double getKd() {
		return config.getDouble("Kd", 1.0);
	}

	public void setKd(double Kd) {
		config.put("Kd", Kd);
		fireChangeEvent();
	}

	public double getTau() {
		return config.getDouble("tau", 1.0);
	}

	public void setTau(double tau) {
		config.put("tau", tau);
		fireChangeEvent();
	}

	public double getCd() {
		return config.getDouble("Cd", 0.5);
	}

	public void setCd(double Cd) {
		config.put("Cd", Cd);
		fireChangeEvent();
	}

	public double getMass() {
		return config.getDouble("Mass", 23.088);
	}

	public void setMass(double mass) {
		config.put("Mass", mass);
		fireChangeEvent();
	}
	
	public double getArea(){
		return config.getDouble("Area", 1121.0);
	}

	public void setArea(double area) {
		config.put("Area", area);
		fireChangeEvent();
	}

}