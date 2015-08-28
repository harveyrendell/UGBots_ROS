package debugView;

import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.BoxLayout;
import javax.swing.JScrollPane;
import javax.swing.JSeparator;


/**
 * Project 1: Group robotic behaviour simulation using Robot Operating System (ROS) 
 * Group 1: UGBots
 * 
 * Members: Andy Choi, Kevin Choi, Andrew Jeoung, Jay Kim, Jenny Lee, Namjun Park, Harvey Rendell, Chuan-Yu Wu
 * 
 * This class is responsible for creating a JPanel object which wraps the content of the GUI
 */

@SuppressWarnings("serial")
public class ContentPanel extends JPanel {

	//Instantiating the required components
	private ROSinit _top = new ROSinit();
	private JButton _startButton = new JButton("Start");
	private JButton _finishButton = new JButton("Finish");
	private JButton _rosmake = new JButton("Compile");
	private JButton _runTests = new JButton("Run Tests");
	private JLabel _waitFor = new JLabel("Please wait 5 seconds for roscore to start");
	private JScrollPane _debug = new JScrollPane();
	private JPanel _infoWrapper = new JPanel();
	private HashMap<String, infoPanel> _map = new HashMap<String, infoPanel>();
	private Processes processes= new Processes(this, _top.getVals());
	
	public ContentPanel() {
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		processes.runRos();
		
		addContent();
		addListeners();
	}
	
	
	//Method to add the content 
	private void addContent() {
		_finishButton.setVisible(false);
		
		JPanel panel = new JPanel();
		panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
		panel.add(_waitFor);
		panel.add(_rosmake);
		panel.add(_runTests);
		panel.add(_startButton);
		panel.add(_finishButton);
		waitForRos();
		panel.add(new JSeparator());
		panel.setMaximumSize(new Dimension(900, 30));
		_infoWrapper.setLayout(new GridLayout(0,2,0,0));
		_debug.setViewportView(_infoWrapper);
		_debug.getVerticalScrollBar().setUnitIncrement(20);
		
		this.add(_top);
		this.add(panel);
		this.add(_debug);
	}
	
	//Method to create the labels which display basic information of each node
	private void createLabels(){
		String[] names = {"Picker","Carrier","Worker","Visitor","Dog","Cat","Possum","Tractor"};
		String[] code = {"P","C","W","V","D","C","PO","T",};
		int vals[] = _top.getVals();
		int k = (vals[8]) * 2 - 1;
		String name = "robot_";
		
		for (int i = 0; i < vals.length - 1; i++){
			for (int j = 0; j <vals[i]; j++){
				infoPanel panel = new infoPanel(name + k, names[i] + "-" + code[i] + j);
				panel.setMinimumSize(new Dimension(900, 200));
				_map.put(name + k, panel);
				k++;
			}
		}
	}
	
	//Method to paint the labels onto the GUI
	private void paint(){
		String name = "robot_";
		int vals[] = _top.getVals();
		int start = (vals[8]) * 2 - 1;
		for (int i =start; i < _map.size() + start; i++){
			_infoWrapper.add(_map.get(name + i));
		}
		
	}
	
	//Method to return the hashmap mapping the name to the corresponding Panel
	public HashMap<String, infoPanel> getMap(){
		return this._map;
	}
	
	//Adding listeners to the buttons made
	private void addListeners() {
		
		//Listener of start button to start the process creating world
		_startButton.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent arg0) {
				processes.update(_top.getVals());
				_infoWrapper.removeAll();
				_top.disableAll();
				_startButton.setVisible(false);
				_finishButton.setVisible(true);
				processes.runRos();
				createLabels();
				paint();
				
				processes.start();
				
			}
		});	
		
		//Listener of finish button to end the appropriate ros related processes
		_finishButton.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				_finishButton.setVisible(false);
				_startButton.setVisible(true);
				_top.enableAll();
				
				_map.clear();
				processes.finishProcs();
			}
		});
		
		//Listener of test button to start tests
		_runTests.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				processes.runTests();
			}
		});
		
		//Listener of compile button to run rosmake
		_rosmake.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				processes.rosmake();
			}
		});
	}

	//Method to kill all ros related processes
	public void killProcs() {
		if (processes != null){
			processes.killProcs();
		}
	}
	
	//Method called when GUI is opened to wait for roscore to start
	public void waitForRos(){
		_startButton.setEnabled(false);
		_rosmake.setEnabled(false);
		_runTests.setEnabled(false);
		processes.waitFive();
	}
	
	//method to enable the buttons when ROS is ready.
	public void enableButtons(){
		_startButton.setEnabled(true);
		_rosmake.setEnabled(true);
		_runTests.setEnabled(true);
		_waitFor.setVisible(false);
	}
	
}
