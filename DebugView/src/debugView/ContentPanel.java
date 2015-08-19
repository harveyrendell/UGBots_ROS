package debugView;

import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.BoxLayout;
import javax.swing.JScrollPane;
import javax.swing.JSeparator;

@SuppressWarnings("serial")
public class ContentPanel extends JPanel {

	/**
	 * Create the panel.
	 */
	
	private ROSinit _top = new ROSinit();
	private JButton _startButton = new JButton("Start");
	private JButton _finishButton = new JButton("Finish");
	private JScrollPane _debug = new JScrollPane();
	private JPanel _infoWrapper = new JPanel();
	private HashMap<String, infoPanel> _map = new HashMap<String, infoPanel>();
	private Processes processes = new Processes(this);
	
	public ContentPanel() {
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		
		addContent();
		addListeners();
	}

	private void addContent() {
		_finishButton.setVisible(false);
		
		JPanel panel = new JPanel();
		panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
		panel.add(_startButton);
		panel.add(_finishButton);
		panel.add(new JSeparator());
		panel.setMaximumSize(new Dimension(900, 30));
		_infoWrapper.setLayout(new GridLayout(0,1,0,0));
		_debug.setViewportView(_infoWrapper);
		
		this.add(_top);
		this.add(panel);
		this.add(_debug);
	}
	
	private void createLabels(){
		String[] names = {"Picker","Carrier","Worker","Visitor","Dog","Cat","Possum","Tractor"};
		int vals[] = _top.getVals();
		int k = 0;
		String name = "Robot_";
		
		for (int i = 0; i < vals.length; i++){
			for (int j = 0; j <vals[i]; j++){
				infoPanel panel = new infoPanel(name + k, names[i]);
				panel.setMinimumSize(new Dimension(900, 200));
				_map.put(name + k, panel);
				k++;
			}
		}
	}
	
	private void paint(){
		String name = "Robot_";
		
		for (int i =0; i< _map.size(); i++){
			_infoWrapper.add(_map.get(name + i));
		}
		
	}
	
	private void addListeners() {
		_startButton.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent arg0) {
				_top.disableAll();
				_startButton.setVisible(false);
				_finishButton.setVisible(true);
				
				createLabels();
				paint();
				
				processes.start();
				
			}
		});	
		
		_finishButton.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				_finishButton.setVisible(false);
				_startButton.setVisible(true);
				_top.enableAll();
				_map.clear();
				_infoWrapper.removeAll();
			}
			
		});
	}
}
