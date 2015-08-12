import java.awt.Color;
import java.awt.EventQueue;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JScrollPane;
import javax.swing.BoxLayout;
import javax.swing.JPanel;
import javax.swing.SwingWorker;
import javax.swing.BorderFactory;
import javax.swing.border.Border;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.ProcessBuilder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

public class DebugView {

	private JFrame frame;
	private JPanel panel = new JPanel();
	private Border border = BorderFactory.createLineBorder(Color.BLACK, 3);
	
	//list to dynamically store the current the labels containing the information of nodes
	private List<JLabel> labels = new ArrayList<JLabel>();

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					DebugView window = new DebugView();
					window.frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public DebugView() {
		initialize();
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		//Run background processes
		runRos();
		
		frame = new JFrame();
		frame.setBounds(100, 100, 450, 300);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.X_AXIS));
		
		JScrollPane scrollPane = new JScrollPane();
		frame.getContentPane().add(scrollPane);
		
		scrollPane.setViewportView(panel);
		panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
		
	}

	//Update the GUI using updated hashmap
	public void updateLabel(String key, String[] statuses){
		
			//Creating the status display using the input info
			String info= "<HTML>" + key;
			for (int i=0; i<3; i++){
				info = info +"<br>" + statuses[i];
			}
			
			info = info + "<br><HTML>";
			
			boolean exists = false;
			//Check to see if a label exists for the input info
			for (JLabel label: labels){
				if(label.getText().subSequence(0,13).equals(info.subSequence(0, 13))){
					//Replace the text with new info
					label.setText(info);
					exists = true;
				}
			}
			
			//if it doesnt exist, add a label for it
			if (!exists){
				JLabel nodeStatus = new JLabel(info);
				nodeStatus.setBorder(border);
				panel.add(nodeStatus);
				panel.updateUI();
				labels.add(nodeStatus);
			}
	}
	
	
	//Create background thread to run the appropriate ros and updating processes
	public void runRos(){
		SwingWorker<Void, Void> worker = new SwingWorker<Void, Void>(){

			private HashMap<String, String[]> robotStatus = new HashMap<String, String[]>();
			
			@Override
			protected Void doInBackground() throws Exception {
				
				/**Testing dynamic changing
				 * 
				for (int i = 0; i<10; i++){
					String[] test = new String[]{"X", "Y", "Status" + i};
					String name = "Robot_" + i;
					robotStatus.put(name, test);
					
					updateGUI(robotStatus);
					Thread.sleep(1000);
				}
				for (int i = 0; i < 10; i++){
					robotStatus.get("Robot_9")[2] = "Status" + i;
					updateGUI(robotStatus);
					Thread.sleep(1000);
				}
				*
				**/
				
				//TRY WHEN GUI WORKS
				try {
					BufferedReader br;
					//Running run.bash process to create roscore, and stage with moving nodes
					//This process will also be responsible reading all published messages from topic rosout
					Process p = new ProcessBuilder("/bin/bash","run.bash").start();
						while(true){
							try{
								//Reading all the published messages
								br = new BufferedReader(new InputStreamReader(p.getInputStream()));
								String line = "";
								//Sending update information to update GUI 
								while(true){

									
									//Spring.split instead of substring
									if((line = br.readLine()) != null && line.contains("name:")){
										updateData(line.split("/")[1], br.readLine());
									}
								}
					
							} catch(Exception e){
							}
						}

				} catch (IOException e1) {
		        // TODO Auto-generated catch block
		        e1.printStackTrace();
		    	}
				return null;
			}
			
			
			//Updating the hashmap with the input data
			public void updateData(String name, String msg){

				//Searching to see whether robot exists or not.
				if (robotStatus.containsKey(name)){
					
					//If exists edit field appropriately.
					String[] statuses =robotStatus.get(name);
					addStatuses(name, msg, statuses);
				}
				//If doesnt exist, edit information accordingly
				else{
					String[] statuses= new String[]{"x: ","y: ","Status: "};
					
					addStatuses(name, msg, statuses);
				}
				
			}
			
			//Changing statuses of existing or new Robots
			public void addStatuses(String name, String msg, String[] statuses){

				String[] msgSplit = msg.split("/");				


				if (msgSplit[1].equals("position")){
					msgSplit[3] = msgSplit[3].substring(0,5);
					if (msgSplit[2].equals("y")){
						if (statuses[1] != msg){
							statuses[1] = "y: " + msgSplit[3];
							robotStatus.put(name, statuses);
							updateLabel(name, statuses);
						}
					}
					else if (msgSplit[2].equals("x")){
						if (statuses[0] != msg){
							statuses[0] = "x: " + msgSplit[3];
							robotStatus.put(name, statuses);
							updateLabel(name, statuses);
						}
					}
				}
				else if (msgSplit[1].equals("status")){
					statuses[2] ="Status: " + msgSplit[2];
					robotStatus.put(name, statuses);
					updateLabel(name, statuses);
				}
			}

			protected void done(){};
		};
		worker.execute();
	}

}


