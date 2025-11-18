package info.openrocket.swing.gui.configdialog;

import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;

import info.openrocket.swing.gui.adaptors.IntegerModel;
import net.miginfocom.swing.MigLayout;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.startup.Application;
import info.openrocket.core.unit.UnitGroup;

import info.openrocket.swing.gui.SpinnerEditor;
import info.openrocket.swing.gui.adaptors.DoubleModel;
import info.openrocket.swing.gui.components.BasicSlider;
import info.openrocket.swing.gui.components.UnitSelector;

@SuppressWarnings("serial")
public class AirbrakeSetConfig extends RocketComponentConfig {
	private static final Translator trans = Application.getTranslator();

	public AirbrakeSetConfig(OpenRocketDocument d, RocketComponent component, JDialog parent) {
		super(d, component, parent);
        JPanel mainPanel = new JPanel(new MigLayout());
        mainPanel.add(generalTab(), "grow, wrap");
        tabbedPane.insertTab("Airbrakes", null, mainPanel,
                "General", 0);

	}

	protected JPanel generalTab() {
		JPanel primary = new JPanel(new MigLayout());

		JPanel panel = new JPanel(new MigLayout("gap rel unrel, ins 0", "[][65lp::][30lp::]", ""));
		DoubleModel m;
		DoubleModel locd;
		JSpinner spin;

		//// Length (y dimension)
		panel.add(new JLabel("Length (l):"));
		m = new DoubleModel(component, "Length", UnitGroup.UNITS_LENGTH, 0,2);
		register(m);
		spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "growx");
		order.add(((SpinnerEditor) spin.getEditor()).getTextField());
		panel.add(new UnitSelector(m), "growx");
		panel.add(new BasicSlider(m.getSliderModel(0, 0.5)), "w 100lp, wrap");

		//// Width (z dimension)
		panel.add(new JLabel("Width (w):"));
		m = new DoubleModel(component, "Width", UnitGroup.UNITS_LENGTH, 0,4e-2);
		register(m);
		spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "growx");
		order.add(((SpinnerEditor) spin.getEditor()).getTextField());
		panel.add(new UnitSelector(m), "growx");
		panel.add(new BasicSlider(m.getSliderModel(0, 0.2)), "w 100lp, wrap");

		//// Thickness (x dimension)
		panel.add(new JLabel("Thickness (h):"));
		m = new DoubleModel(component, "Thickness", UnitGroup.UNITS_LENGTH, 0,4e-2);
		register(m);
		spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "growx");
		order.add(((SpinnerEditor) spin.getEditor()).getTextField());
		panel.add(new UnitSelector(m), "growx");
		panel.add(new BasicSlider(m.getSliderModel(0, 0.05)), "w 100lp, wrap");

		//// Fraction exposed (fE)
		panel.add(new JLabel("Fraction exposed (fE):"));
		m = new DoubleModel(component, "FracExposed", UnitGroup.UNITS_COEFFICIENT, 0, 1);
		register(m);
		spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "growx");
		order.add(((SpinnerEditor) spin.getEditor()).getTextField());
		panel.add(new BasicSlider(m.getSliderModel(0, 1.0)), "w 100lp, wrap");

		//// Mass per airbrake (m)
		panel.add(new JLabel("Mass per airbrake (m):"));
		m = new DoubleModel(component, "IndivAirbrakeMass", UnitGroup.UNITS_MASS, 0,10e-3);
		register(m);
		spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "growx");
		order.add(((SpinnerEditor) spin.getEditor()).getTextField());
		panel.add(new UnitSelector(m), "growx");
		panel.add(new BasicSlider(m.getSliderModel(0, 1.0)), "w 100lp, wrap");

        //// CD
        panel.add(new JLabel("Override CD Factor"));
        locd = new DoubleModel(component, "LocalOverrideCD", UnitGroup.UNITS_NONE, 0,10);
        register(locd);
        spin = new JSpinner(locd.getSpinnerModel());
        spin.setEditor(new SpinnerEditor(spin));
        panel.add(spin, "growx");
        order.add(((SpinnerEditor) spin.getEditor()).getTextField());
        panel.add(new UnitSelector(locd), "growx");
        panel.add(new BasicSlider(locd.getSliderModel(0, 10.0)), "w 100lp, wrap");

		// Number of airbrakes
		panel.add(new JLabel("Number of airbrakes:"));
        IntegerModel numModel = new IntegerModel(component, "NumAirbrakes", 2, 16);
		register(numModel);
		spin = new JSpinner(numModel.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "growx");
		order.add(((SpinnerEditor) spin.getEditor()).getTextField());
		panel.add(new BasicSlider(numModel.getSliderModel()), "w 100lp, wrap");

		primary.add(panel, "grow, gapright 40lp");

		// Right side: placement and material similar to RingComponentConfig
		JPanel rightSide = new JPanel(new MigLayout("gap rel unrel, ins 0", "[][65lp::][30lp::]", ""));
		primary.add(rightSide, "cell 4 0, aligny 0, spany");

		PlacementPanel pp = new PlacementPanel(component, order);
		register(pp);
		rightSide.add(pp, "span, grow");

		MaterialPanel materialPanel = new MaterialPanel(component, document, info.openrocket.core.material.Material.Type.BULK, order);
		register(materialPanel);
		rightSide.add(materialPanel, "span, grow, wrap");

		return primary;
	}

}

