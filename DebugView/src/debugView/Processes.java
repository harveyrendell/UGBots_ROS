package debugView;

import java.io.BufferedReader;
import java.io.InputStreamReader;

import javax.swing.SwingWorker;

public class Processes {
	private ContentPanel _panel;
	private SwingWorker<Void, Void> rosWorker;
	private SwingWorker<Void, Void> worldWorker;
	private SwingWorker<Void, Void> topicWorker;
	private SwingWorker<Void, Void> worldWriter;
	
	private Process _core;
	private Process _writeWorldFiles;
	private Process _world;
	private Process _launch;
	
	public Processes(ContentPanel panel){
		_panel = panel;
	}

	
	public void start(){
		runRos();
		runWorld();
		writeWorldFiles();
		launchNodes();
	}
	
	//Process to run roscore
	private void runRos(){
		rosWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				_core = new ProcessBuilder("/bin/bash", "runcore.bash").start();
				
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
				_writeWorldFiles = new ProcessBuilder("/bin/bash","").start();
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
				_world = new ProcessBuilder("/bin/bash", "runworld.bash").start();
				
				return null;
			}
			
			
		};
		
		worldWorker.execute();
	}
	
	//Process to run roslaunch and tests
	private void launchNodes(){
		topicWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				_launch = new ProcessBuilder("/bin/bash", "launchnodes.bash").start();
				return null;
			}
		};
		topicWorker.execute();
	}
	
	//Kill all processes
	public void killProcs(){
		SwingWorker<Void, Void> killAll = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				Process kill = new ProcessBuilder("/bin/bash", "kill.bash").start();
				return null;
			}
			
		};
		killAll.execute();
		_launch.destroy();
		_world.destroy();
		_writeWorldFiles.destroy();
		_core.destroy();
		
	}
}
