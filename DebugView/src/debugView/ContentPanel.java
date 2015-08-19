package debugView;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.BoxLayout;
import javax.swing.JScrollPane;

public class ContentPanel extends JPanel {

	/**
	 * Create the panel.
	 */
	
	private ROSinit _top = new ROSinit();
	private JButton _startButton = new JButton("Start");
	private JScrollPane _debug = new JScrollPane();
	
	
	public ContentPanel() {
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		
		addContent();
		addListeners();
	}

	private void addContent() {
		
		JPanel panel = new JPanel();
		panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
		panel.add(_startButton);
		
		this.add(_top);
		this.add(panel);
		this.add(_debug);
	}
	
	private void addListeners() {
		_startButton.addActionListener(new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent arg0) {
				_top.disableAll();
				_startButton.setEnabled(false);
			}
		});	
	}
}
