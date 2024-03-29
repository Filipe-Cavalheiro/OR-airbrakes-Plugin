package net.sf.openrocket.ORBrake;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.gui.components.BasicSlider;
import net.sf.openrocket.gui.components.UnitSelector;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.unit.UnitGroup;

@Plugin
public class ORBrakeConfigurator extends AbstractSwingSimulationExtensionConfigurator<ORBrake> {
	/**
	 * The configurator is responsible for creating the config GUI when the
	 * extension is loaded.
	 */

	public ORBrakeConfigurator() {
		super(ORBrake.class);
	}

	@Override
	protected JComponent getConfigurationComponent(ORBrake extension, Simulation simulation, JPanel panel) {
		panel.add(new JLabel("Setpoint:"));
		DoubleModel S = new DoubleModel(extension, "Setpoint", UnitGroup.UNITS_DISTANCE, 0);

		JSpinner spinS = new JSpinner(S.getSpinnerModel());
		spinS.setEditor(new SpinnerEditor(spinS));
		panel.add(spinS, "w 65lp!");

		UnitSelector unitS = new UnitSelector(S);
		panel.add(unitS, "w 25");

		BasicSlider sliderS = new BasicSlider(S.getSliderModel(0, 3100));
		panel.add(sliderS, "w 75lp, wrap");

		panel.add(new JLabel("Proportional Gain:"));
		DoubleModel P = new DoubleModel(extension, "Kp", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinP = new JSpinner(P.getSpinnerModel());
		spinP.setEditor(new SpinnerEditor(spinP));
		panel.add(spinP, "w 65lp!");

		UnitSelector unitP = new UnitSelector(P);
		panel.add(unitP, "w 25");

		BasicSlider sliderP = new BasicSlider(P.getSliderModel(0, 10));
		panel.add(sliderP, "w 75lp, wrap");

		panel.add(new JLabel("Integral Gain:"));
		DoubleModel I = new DoubleModel(extension, "Ki", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinI = new JSpinner(I.getSpinnerModel());
		spinI.setEditor(new SpinnerEditor(spinI));
		panel.add(spinI, "w 65lp!");

		UnitSelector unitI = new UnitSelector(I);
		panel.add(unitI, "w 25");

		BasicSlider sliderI = new BasicSlider(I.getSliderModel(0, 5));
		panel.add(sliderI, "w 75lp, wrap");

		panel.add(new JLabel("Differential Gain:"));
		DoubleModel D = new DoubleModel(extension, "Kd", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinD = new JSpinner(D.getSpinnerModel());
		spinD.setEditor(new SpinnerEditor(spinD));
		panel.add(spinD, "w 65lp!");

		UnitSelector unitD = new UnitSelector(D);
		panel.add(unitD, "w 25");

		BasicSlider sliderD = new BasicSlider(D.getSliderModel(0, 5));
		panel.add(sliderD, "w 75lp, wrap");

		panel.add(new JLabel("Time Constant:"));
		DoubleModel Tau = new DoubleModel(extension, "Tau", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinTau = new JSpinner(Tau.getSpinnerModel());
		spinTau.setEditor(new SpinnerEditor(spinTau));
		panel.add(spinTau, "w 65lp!");

		UnitSelector unitTau = new UnitSelector(Tau);
		panel.add(unitTau, "w 25");

		BasicSlider sliderTau = new BasicSlider(Tau.getSliderModel(0, 3));
		panel.add(sliderTau, "w 75lp, wrap");

		panel.add(new JLabel("Rocket Drag Coefficient:"));
		DoubleModel Rocket_Cd = new DoubleModel(extension, "Rocket_Cd", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinRocket_Cd = new JSpinner(Rocket_Cd.getSpinnerModel());
		spinRocket_Cd.setEditor(new SpinnerEditor(spinRocket_Cd));
		panel.add(spinRocket_Cd, "w 65lp!");

		UnitSelector unitRocket_Cd = new UnitSelector(Rocket_Cd);
		panel.add(unitRocket_Cd, "w 25");

		BasicSlider sliderRocket_Cd = new BasicSlider(Rocket_Cd.getSliderModel(0, 2));
		panel.add(sliderRocket_Cd, "w 75lp, wrap");
		
		panel.add(new JLabel("AB Drag Coefficient:"));
		DoubleModel AB_Cd = new DoubleModel(extension, "AB_Cd", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinAB_Cd = new JSpinner(AB_Cd.getSpinnerModel());
		spinAB_Cd.setEditor(new SpinnerEditor(spinAB_Cd));
		panel.add(spinAB_Cd, "w 65lp!");

		UnitSelector unitAB_Cd = new UnitSelector(AB_Cd);
		panel.add(unitAB_Cd, "w 25");

		BasicSlider sliderAB_Cd = new BasicSlider(AB_Cd.getSliderModel(0, 2));
		panel.add(sliderAB_Cd, "w 75lp, wrap");

		panel.add(new JLabel("Estimate Mass:"));
		DoubleModel Mass = new DoubleModel(extension, "Mass", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinMass = new JSpinner(Mass.getSpinnerModel());
		spinMass.setEditor(new SpinnerEditor(spinMass));
		panel.add(spinMass, "w 65lp!");

		UnitSelector unitMass = new UnitSelector(Mass);
		panel.add(unitMass, "w 25");

		BasicSlider sliderMass = new BasicSlider(Mass.getSliderModel(0, 40));
		panel.add(sliderMass, "w 75lp, wrap");
		
		panel.add(new JLabel("AB Area (mm^2):"));
		DoubleModel Area = new DoubleModel(extension, "Area", UnitGroup.UNITS_COEFFICIENT, 0);

		JSpinner spinArea = new JSpinner(Area.getSpinnerModel());
		spinArea.setEditor(new SpinnerEditor(spinArea));
		panel.add(spinArea, "w 65lp!");

		UnitSelector unitArea = new UnitSelector(Area);
		panel.add(unitArea, "w 25");

		BasicSlider sliderArea = new BasicSlider(Area.getSliderModel(0, 40));
		panel.add(sliderArea, "w 75lp, wrap");
		
		return panel;
	}

}