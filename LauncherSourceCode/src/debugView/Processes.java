package debugView;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.HashMap;

import javax.swing.SwingWorker;


/**
 * Project 1: Group robotic behaviour simulation using Robot Operating System (ROS) 
 * Group 1: UGBots
 * 
 * Members: Andy Choi, Kevin Choi, Andrew Jeoung, Jay Kim, Jenny Lee, Namjun Park, Harvey Rendell, Chuan-Yu Wu
 * 
 * This class is responsible for creating creating the appropriate processes 
 * when actions are taken
 */
public class Processes {
	private ContentPanel _panel;
	private int[] _vals;
	
	//Fields for the swingworkers and processes
	private SwingWorker<Void, Void> rosWorker;
	private SwingWorker<Void, Void> worldWorker;
	private SwingWorker<Void, Void> launchWorker;
	private SwingWorker<Void, Void> worldWriter;
	private SwingWorker<Void, Void> topicWorker;
	
	private Process _core;
	private Process _writeWorldFiles;
	private Process _world;
	private Process _launch;
	private Process _topic;

	public Processes(ContentPanel panel, int[] vals){
		_panel = panel;
		_vals = vals;
		
	}
	
	//Enables GUI components after 5 seconds
	//Used when GUI is first opened to wait for roscore to start
	public void waitFive(){
		SwingWorker<Void,Void> wait = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				Thread.sleep(5000);
				_panel.enableButtons();
				return null;
				
				
			}
		};
		
		wait.execute();
	}
	
	
	//Method to get the current count for each node on the JSpinners
	public void update(int[] vals){
		_vals = vals;
	}

	//Method to call the appropriate Swing workers when start button is clicked
	public void start(){
		writeWorldFiles();
		runWorld();
		launchNodes();
		readTopics();
	}
	
	//Process to run roscore
	public void runRos(){
		rosWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				_core = new ProcessBuilder("/bin/bash", "JarResources/runcore.bash").start();
				return null;
				
				
			}
		};
		
		rosWorker.execute();
	}
	
	//Process to edit the world files
	private void writeWorldFiles(){
		worldWriter = new SwingWorker<Void, Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				
				
				_writeWorldFiles = new ProcessBuilder("/bin/bash","JarResources/writeWorld.bash", "" + _vals[0],
						"" + _vals[1], "" + _vals[2], "" + _vals[3], "" + _vals[4], "" + _vals[5],
						"" + _vals[6], "" + _vals[7], "" + _vals[8]).start();
				return null;
			}
		};
		
		worldWriter.execute();
		
	}
	
	//Process to start stageros
	private void runWorld(){
		worldWorker = new SwingWorker<Void,Void>(){


			@Override
			protected Void doInBackground() throws Exception {
				_world = new ProcessBuilder("/bin/bash", "JarResources/runworld.bash").start();
				
				return null;
			}
			
			
		};
		
		worldWorker.execute();
	}
	
	//Process to run roslaunch and tests
	private void launchNodes(){
		launchWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				_launch = new ProcessBuilder("/bin/bash", "JarResources/launchnodes.bash").start();
				return null;
			}
		};
		launchWorker.execute();
	}
	
	//Read the rosout calls and then edit the GUI.
	private void readTopics(){
		topicWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				_topic = new ProcessBuilder("/bin/bash","JarResources/runtopic.bash").start();
				BufferedReader br;
				//Running run.bash process to create roscore, and stage with moving nodes
				//This process will also be responsible reading all published messages from topic rosout					
					while(true){
						try{
							//Reading all the published messages
							br = new BufferedReader(new InputStreamReader(_topic.getInputStream()));
							String line = "";
							//Sending update information to update GUI 
							while(true){
								//Spring.split instead of substring
								if((line = br.readLine()) != null && line.contains("name:")){
									String msg = br.readLine();
									updateGUI(line.split("/")[1], msg);
								}
							}
				
						} catch(Exception e){
						}
					}
			}
			
			//Method to check which robot the information belongs to and update GUI accordinly
			private void updateGUI(String name, String info){

				HashMap<String, infoPanel> map = _panel.getMap();
				infoPanel temp = map.get(name);
				String[] split = info.split("/");

				
				if (split[1].equals("position")){
					split[3] = split[3].substring(0,5);
					if(split[2].equals("x")){
						temp.setXPos("x position: " + split[3]);
					}
					else if(split[2].equals("y")){
						temp.setYPos("y position: " + split[3]);
					}
				}
				else if (split[1].equals("status")){
					temp.setStatus("status: " + split[2]);
					
				}
				else if (split[1].equals("message")){
					temp.setMessage("message: " + split[2]);
				}
				else if (split[1].equals("bin")){
					temp.setBinProg(split[2]);
				}
				
				
			}
		};
		topicWorker.execute();
	}
	
	//Kill all processes but roscore
	public void finishProcs(){
		SwingWorker<Void, Void> killAll = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				Process kill = new ProcessBuilder("/bin/bash", "JarResources/finish.bash").start();
				return null;
			}
		};
		killAll.execute();
		if(_topic != null){
			_topic.destroy();
		}
	}
	
	//Method to run test cases
	public void runTests(){
		SwingWorker<Void, Void> runTests = new SwingWorker<Void,Void>(){
			
			@Override
			protected Void doInBackground() throws Exception {
				Process testBuilder = new ProcessBuilder("/bin/bash", "JarResources/runtests.bash").start();
				return null;
			}
			
		};
		runTests.execute();
		
	}

	//Method to run rosmake 
	public void rosmake() {
		SwingWorker<Void, Void> rosmake = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				Process makeBuilder = new ProcessBuilder("/bin/bash", "JarResources/runmake.bash").start();
				return null;
			}
		};
		rosmake.execute();
	}
	
	
	//Method to kill all processes including roscore
	public void killProcs(){
		SwingWorker<Void, Void> killAll = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				Process kill = new ProcessBuilder("/bin/bash", "JarResources/kill.bash").start();
				return null;
			}
		};
		killAll.execute();
		if(_topic != null){
			_topic.destroy();
		}
	}
}
