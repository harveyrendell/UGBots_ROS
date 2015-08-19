package debugView;

import javax.swing.SwingWorker;

public class Processes {
	private ContentPanel _panel;
	private SwingWorker<Void, Void> rosWorker;
	private SwingWorker<Void, Void> worldWorker;
	private SwingWorker<Void, Void> topicWorker;
	
	public Processes(ContentPanel panel){
		_panel = panel;
	}

	
	public void start(){
		runRos();
		runWorld();
		runTopic();
	}
	
	private void runRos(){
		rosWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				// TODO Auto-generated method stub
				return null;
			}
			
			
		};
		
		rosWorker.execute();
	}
	
	private void runWorld(){
		worldWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				// TODO Auto-generated method stub
				return null;
			}
			
			
		};
		
		worldWorker.execute();
	}
	
	private void runTopic(){
		topicWorker = new SwingWorker<Void,Void>(){

			@Override
			protected Void doInBackground() throws Exception {
				// TODO Auto-generated method stub
				return null;
			}
			
			
		};
		topicWorker.execute();
	}
}
