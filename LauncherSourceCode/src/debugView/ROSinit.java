package debugView;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.BoxLayout;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;

import java.awt.Dimension;
import java.awt.GridLayout;

/**
 * Project 1: Group robotic behaviour simulation using Robot Operating System (ROS) 
 * Group 1: UGBots
 * 
 * Members: Andy Choi, Kevin Choi, Andrew Jeoung, Jay Kim, Jenny Lee, Namjun Park, Harvey Rendell, Chuan-Yu Wu
 * 
 * This class is responsible for creating a JPanel object which will display the
 * GUI elements the user can interact with.
 */


@SuppressWarnings("serial")
public class ROSinit extends JPanel {
	
	//Creating Labels for dynamic elements
	private JLabel _picker = new JLabel("Picker Robots:");
	private JLabel _carrier = new JLabel("Carrier Robots:");
	private JLabel _worker = new JLabel("Workers:");
	private JLabel _visitor = new JLabel("Visitors:");
	private JLabel _dog = new JLabel("Dogs:");
	private JLabel _cat = new JLabel("Cats:");
	private JLabel _possum = new JLabel("Possums:");
	private JLabel _tractor = new JLabel("Tractors:");
	private JLabel _rows = new JLabel("Orchard Rows:");
	
	//Creating bound spinners for dynamic elements
	private JSpinner _pickerVal = new JSpinner(new SpinnerNumberModel(0,0,4,1));
	private JSpinner _carrierVal = new JSpinner(new SpinnerNumberModel(0,0,4,1));
	private JSpinner _workerVal = new JSpinner(new SpinnerNumberModel(1,1,1,1));
	private JSpinner _visitorVal = new JSpinner(new SpinnerNumberModel(0,0,4,1));
	private JSpinner _dogVal = new JSpinner(new SpinnerNumberModel(0,0,6,1));
	private JSpinner _catVal = new JSpinner(new SpinnerNumberModel(0,0,4,1));
	private JSpinner _possumVal = new JSpinner(new SpinnerNumberModel(0,0,4,1));
	private JSpinner _tractorVal = new JSpinner(new SpinnerNumberModel(0,0,1,1));
	private JSpinner _rowVal = new JSpinner(new SpinnerNumberModel(2,2,10,1));
	
	//Creating start button
	private JButton _startButton = new JButton("Start");
	
	
	//Instantiator
	public ROSinit() {
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		
		JLabel prompt = new JLabel ("Enter the number of components to be added:");
		JPanel top = new JPanel();
		top.setMaximumSize(new Dimension(900, 40));
		top.add(prompt);
		
		this.setMaximumSize(new Dimension(900,450));
		this.add(top);
		addPromptPanel();
	}
	
	//Method to add the prompting elements
	private void addPromptPanel(){
		
		JPanel panel = new JPanel();
		panel.setLayout(new GridLayout(0, 6, 0, 0));
		
		panel.add(_picker);
		panel.add(_pickerVal);
		panel.add(_carrier);
		panel.add(_carrierVal);
		panel.add(_worker);
		panel.add(_workerVal);
		panel.add(_visitor);
		panel.add(_visitorVal);
		panel.add(_dog);
		panel.add(_dogVal);
		panel.add(_cat);
		panel.add(_catVal);
		panel.add(_possum);
		panel.add(_possumVal);
		panel.add(_tractor);
		panel.add(_tractorVal);
		panel.add(_rows);
		panel.add(_rowVal);
		
		this.add(panel);
	}
	
	//Method to disable all interactive components
	public void disableAll(){
		_pickerVal.setEnabled(false);
		_carrierVal.setEnabled(false);
		_workerVal.setEnabled(false);
		_visitorVal.setEnabled(false);
		_dogVal.setEnabled(false);
		_catVal.setEnabled(false);
		_possumVal.setEnabled(false);
		_tractorVal.setEnabled(false);
		_rowVal.setEnabled(false);
		_startButton.setEnabled(false);
	}
	
	//Method to enable all interactive components
	public void enableAll(){
		_pickerVal.setEnabled(true);
		_carrierVal.setEnabled(true);
		_workerVal.setEnabled(true);
		_visitorVal.setEnabled(true);
		_dogVal.setEnabled(true);
		_catVal.setEnabled(true);
		_possumVal.setEnabled(true);
		_tractorVal.setEnabled(true);
		_rowVal.setEnabled(true);
		
	}
	
	//Method to get the user input
	public int[] getVals(){
		int[] vals = new int[9];
		vals[0] = (int) _pickerVal.getValue();
		vals[1] = (int) _carrierVal.getValue();
		vals[2] = (int) _workerVal.getValue();
		vals[3] = (int) _visitorVal.getValue();
		vals[4] = (int) _dogVal.getValue();
		vals[5] = (int) _catVal.getValue();
		vals[6] = (int) _possumVal.getValue();
		vals[7] = (int) _tractorVal.getValue();
		vals[8] = (int) _rowVal.getValue();
		return vals;
	}
}
