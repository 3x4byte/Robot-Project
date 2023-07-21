package pack1;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;

import java.util.concurrent.Semaphore;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.internal.ClientComms;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import com.google.gson.*;

public class RobotMqttClient {
	
	static MqttClient client;
	static Gson gson = new Gson();
	
	static EV3LargeRegulatedMotor left = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor right = new EV3LargeRegulatedMotor(MotorPort.B);
	static EV3MediumRegulatedMotor lifter = new EV3MediumRegulatedMotor(MotorPort.C);
	static EV3MediumRegulatedMotor grabber = new EV3MediumRegulatedMotor(MotorPort.D);
	
	public static void main(String[] args) throws MqttException {
		
		String address = "141.46.233.97";
		String port = "1883";
		
		client = new MqttClient( 
			    "tcp://" + address + ":" + port, //URI
			    MqttClient.generateClientId(), //ClientId
			    new MemoryPersistence()); //Persistence
		
		client.setCallback(new MqttCallback() {
		
		    @Override
		    public void connectionLost(Throwable cause) { //Called when the client lost the connection to the broker 
		    }
		
		    @Override
		    public void messageArrived(String topic, MqttMessage message) throws Exception {
		        parseMessage(new String(message.getPayload()));
		    }
		
		    @Override
		    public void deliveryComplete(IMqttDeliveryToken token) {//Called when a outgoing publish is complete 
		    }
		});
		
		client.connect();
		
		while(!client.isConnected()) {
			try {
				Thread.sleep(500);
			} catch (Exception e) {}
		}
		
		System.out.println("connected to Server");
		
		client.subscribe("robots/collector", 1);
		
		while(true) {// !Button.ENTER.isDown()
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {}
		}
		
		
	}
	
	private static void sendFinishedMessage() {
		JsonObject finishedMessage = new JsonObject();
		finishedMessage.addProperty("type", "information");
		finishedMessage.addProperty("message", "completed");
		
		try {
			client.publish("laptop/navigator", finishedMessage.toString().getBytes(), 1, false);
		} catch (MqttPersistenceException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (MqttException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	private static void sendPausedMessage(boolean value) {
		JsonObject pausedMessage = new JsonObject();
		pausedMessage.addProperty("type", "paused");
		pausedMessage.addProperty("value", value+"");
		
		try {
			client.publish("laptop/navigator", pausedMessage.toString().getBytes(), 1, false);
		} catch (MqttPersistenceException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (MqttException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	private static void parseMessage(String message) throws InterruptedException {
		
		System.out.println("received: " + message);
		
		JsonObject jsonMsg = gson.fromJson(message, JsonObject.class);
		
		String type = jsonMsg.get("type").getAsString();
		
		switch(type) {
			case "initializasion" : initialization(jsonMsg); break;	
			case "movement" : movement(jsonMsg); break;
			case "pause" : pause(jsonMsg); break;
		}
		
	}
	
	private static void initialization(JsonObject json) throws InterruptedException {
		System.out.println("--initialization message--");
		lifter.setStallThreshold(5, 10);
		grabber.setStallThreshold(5, 10);
		
		lifter.setSpeed(200);
		grabber.setSpeed(200);
		
		lifter.backward();
		
		while(!lifter.isStalled())
			Thread.sleep(5);
		
		lifter.setStallThreshold(1000, 1000);
			
		lifter.rotate(1225);
		lifter.rotate(-360);
		
		grabber.backward();
		
		while(!grabber.isStalled())
			Thread.sleep(5);
		
		grabber.setStallThreshold(1000, 1000);
		
		grabber.rotate(600);
		grabber.rotate(-300);
		
		sendFinishedMessage();
	}
	
	private static void pause(JsonObject json) throws InterruptedException {
		System.out.println("--pause message--");
		boolean pause = json.get("value").getAsBoolean();
		sendPausedMessage(pause);
		
	}
	
	private static void movement(JsonObject json) throws InterruptedException {
		System.out.println("--movement message--");
		String operation = json.get("operation").getAsString();
		int speed = json.get("speed").getAsInt();
		final int steps = json.get("steps").getAsInt();
		
		left.setSpeed(speed);
		right.setSpeed(speed);
		grabber.setSpeed(speed);
		lifter.setSpeed(speed);
		left.setAcceleration(200);
		right.setAcceleration(200);	
		
		if(operation.equals("forward")){
			Thread t1 = new Thread() {
				@Override
				public void run() {
					left.rotate(steps);
				}
			};
			Thread t2 = new Thread() {
				@Override
				public void run() {
					right.rotate(steps);
				}
			};
			t1.start();
			t2.start();
			
			t1.join();
			t2.join();
			
			sendFinishedMessage();
			
		}else if(operation.equals("right")) {
			Thread t1 = new Thread() {
				@Override
				public void run() {
					left.rotate(steps);
				}
			};
			Thread t2 = new Thread() {
				@Override
				public void run() {
					right.rotate(-steps);
				}
			};
			t1.start();
			t2.start();
			
			t1.join();
			t2.join();
			
			sendFinishedMessage();
			
		}else if(operation.equals("grab")) {
			grabber.rotate(-steps);
			
			sendFinishedMessage();
			
		}else if(operation.equals("release")) {
			grabber.rotate(steps);
			
			sendFinishedMessage();
			
		}else if(operation.equals("up")) {
			lifter.rotate(-steps);
			
			sendFinishedMessage();
			
		}else if(operation.equals("down")) {
			lifter.rotate(steps);
			
			sendFinishedMessage();
		}
		
	}
	
}
