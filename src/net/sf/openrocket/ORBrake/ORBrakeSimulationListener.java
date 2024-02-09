package net.sf.openrocket.ORBrake;
import net.sf.openrocket.simulation.FlightData;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;

import java.lang.Math;
import java.util.stream.IntStream;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.aerodynamics.FlightConditions;

public class ORBrakeSimulationListener extends AbstractSimulationListener {
	/**
	 * The simulation listener connects to and influences simulations that is
	 * attached to.
	 */

	//constants
	final double RocketArea = 0.01767;
	
	// Input parameters for PID controller
	double setpoint; // Target altitude in feet
	double Kp; // Proportional gain constant
	double Ki; // Integral gain constant
	double Kd; // Derivative gain constant
	double tau; // Low pass filter time constant
	double T = .05; // Sample time in sec

	// Input parameters for apogee estimator
	double Cd;
	double mass;
	double AB_area;

	// Memory variables for PID controller
	double inte = 0; // Integral term
	double prev_err = 0; // Previous error
	double diff = 0; // Differential term
	double prev_measure = 0; // Previous measurement

	public ORBrakeSimulationListener(double setpoint, double Kp, double Ki, double Kd, double tau, double Cd,
			double mass, double area){
		super();
		this.setpoint = setpoint;
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.tau = tau;
		this.Cd = Cd;
		this.mass = mass;
		this.AB_area = area * 0.000001;
	}

	@Override
	public void startSimulation(SimulationStatus status)
	/**
	 * Gets the time step at the start of the simulation.
	 * 
	 * @param status The status object at the start of the sim.
	 * @return void
	 */
	{
		T = status.getSimulationConditions().getTimeStep();
	}

	@Override
	public double postSimpleThrustCalculation(SimulationStatus status, double thrust) // throws SimulationException
	/**
	 * Influences the thrust after it is computed at each time step but before it is
	 * applied to the vehicle.
	 * 
	 * @param status Object that contains simulation status details.
	 * @param thrust The computed motor thrust.
	 * @return The modified thrust to be actually applied.
	 */
	{
		double drag = airbrakeForce(status, thrust);
		return thrust + drag;
	}

	double airbrakeForce(SimulationStatus status, double thrust) {
		double requiredDrag = requiredDrag(status, thrust);
		double maxDrag = dragSurface(status.getRocketPosition().z, status.getRocketVelocity().length(), AB_area, 1.17)
				+ dragSurface(status.getRocketPosition().z, status.getRocketVelocity().length(), RocketArea, Cd);
		if (requiredDrag > maxDrag){
			requiredDrag = maxDrag;
		} else if (requiredDrag < 0){
			requiredDrag = 0;
		}
		return -requiredDrag;
	}

	double requiredDrag(SimulationStatus status, double thrust)
	/**
	 * Computes required drag using a PID controller.
	 * 
	 * @param status The current simulation status object.
	 * @param thrust The current thrust of the vehicle.
	 * 
	 * @return required drag
	 */
	{
		// Initial conditions
		double out = 0;
		double alt = status.getRocketPosition().z;
		double velocity = status.getRocketVelocity().length();
		double vertVelocity = status.getRocketVelocity().z;
		//double mass = status.getSimulationConditions().getRocket().getMass();

		double gravity = status.getSimulationConditions().getGravityModel().getGravity(status.getRocketWorldPosition());
		double refArea = status.getConfiguration().getReferenceArea();

		double termVelocity = Math.sqrt((2 * mass * gravity) / (Cd * refArea * 1.225));
		double predApogee = alt + (((Math.pow(termVelocity, 2)) / (2 * gravity))
				* Math.log((Math.pow(vertVelocity, 2) + Math.pow(termVelocity, 2)) / (Math.pow(termVelocity, 2))));

		// PID Controller
		double minInte = dragSurface(predApogee, velocity, RocketArea, 0.5);
		double maxInte = dragSurface(predApogee, velocity, AB_area, 1.17) + minInte; 
		
		// Error function
		double err = setpoint - predApogee;

		// Proportional term
		double prop = -Kp * err;

		// Integral term
		inte += 0.5 * Ki * T * (err + prev_err);
		
		// Differential term
		diff = (-2 * Kd * (predApogee - prev_measure) + (2 * tau - T) * diff) / (2 * tau + T);

	    // Anti-wind up (integral clamping)
	    if (inte > maxInte)
	        inte = maxInte;
	    else if (inte < minInte)
	        inte = minInte;
	    
		// Output
		out = prop + inte + diff;

		// Update memory
		prev_err = err;
		prev_measure = predApogee;

		return out;
	}
	
	double airDensity(double altitude) 
	/**
	 * Using a linear approximation (Y = -1.05^{-4} + 1.225) finds the air density for the given altitude.
	 * 
	 * @param altitude
	 * 
	 * @return The air density
	 */{
		return -0.000105 * altitude + 1.225;
	}

	double dragSurface(double altitude, double velocity, double area,double C_drag)
	/**
	 * Finds the drag force of a surfaces given the current velocity, 
	 * altitude and area.
	 * 
	 * @return The drag given altitude, velocity, and area.
	 */{
		return (C_drag * airDensity(altitude)* Math.pow(velocity, 2) * area / 2);
	}

}