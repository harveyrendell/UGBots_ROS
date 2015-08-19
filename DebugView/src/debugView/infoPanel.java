package debugView;

import javax.swing.JPanel;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JProgressBar;
import javax.swing.border.EtchedBorder;

@SuppressWarnings("serial")
public class infoPanel extends JPanel {

	/**
	 * Create the panel.
	 */
	
	//Labels saved as fields for getting and setting
	private JLabel _title = new JLabel();
	private JLabel _type = new JLabel();
	private JLabel _xPos = new JLabel("x position: ");
	private JLabel _yPos = new JLabel("y position: ");
	private JLabel _status = new JLabel("status: ");
	private JLabel _message = new JLabel("message: ");
	private JProgressBar _binProgress = new JProgressBar();
	
	public infoPanel(String title, String type) {
		
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		_title.setText(title);
		_type.setText(type);
		
		//adding the labels when panel is created
		addPanels();
		
		this.setBorder(new EtchedBorder(EtchedBorder.LOWERED));
	}
	
	private void addPanels(){
		this.add(_title);
		this.add(_type);
		this.add(_xPos);
		this.add(_yPos);
		this.add(_status);
		this.add(_message);
		//Add progress bar to show the bins status
		if (_type.getText().equals("Picker")){
			this.add(_binProgress);
		}
	}
	
//Getters and setters to change the information in the label
	public String getXPos(){return _xPos.getText();}
	public void setXPos(String info){_xPos.setText(info);}
	
	public String getYPos(){return _yPos.getText();}
	public void setYPos(String info){_yPos.setText(info);}
	
	public String getStatus(){return _status.getText();}
	public void setStatus(String info){_status.setText(info);}
	
	public String getMessage(){return _message.getText();}
	public void setMessage(String info){ _message.setText(info);}
	
	public String getBinProg(){return _binProgress.getString();}
	public void setBinProg(String info){_binProgress.setString(info);}
}
