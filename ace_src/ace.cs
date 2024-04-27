using Ace.Services.NameLookup;
using Ace.Server;
using System;
using System.Collections.Generic;
using System.Diagnostics;


using System.Net.Sockets;
using System.Net;
using System.Text;
using System.IO;

using Ace.Server.Adept.Robots.Types.Parallel;
using Ace.Server.Adept.Robots.Motion;

namespace Ace.Custom {

	public class Program : IDisposable {

		public INameLookupService ace;
		//public AceServer ace;

		IAbstractHornetRobot robot;
		
		public struct RobotCtrl
		{
			public bool incremental;
			public bool wait_done;
			public double dx;
			public double dy;
			public double dz;
			public double dr;
			public int Accel; //Get or set the motion acceleration. Must be greater than 0, with 100 representing full acceleration. 
			public int Decel; //Get or set the motion deceleration. Must be greater than 0, with 100 representing full deceleration. 
			public int Speed; //Get or set the motion speed. Must be greater than 0, with 100 representing rated speed. 
			public bool Straight; //Get or set a flag indicating whether the motion should be straight-line. If false, motion will be joint-interpolated. 
			public int SCurveProfile; //Get or set the S-curve profile number, from 0 (for trapezoidal) to 8, or -1 to leave unchanged. 
		}; 

		public string ACK_OK="ok|";
		public string ACK_ERROR="error|";
		
		public void motion_init(){
			
			// Make sure the robot controller communications is enabled
			if (robot.IsAlive == false){
				Trace.WriteLine("Robot communications don't enabled");
				throw new InvalidOperationException();
			}
			
			// Make sure power is enabled
			if (robot.Power == false){
				Trace.WriteLine("Robot power don't enabled");
				throw new InvalidOperationException();
			}
			
			// Enable programe control
			robot.AutomaticControlActive = true;

			// Print out information on the robot
			Trace.WriteLine("Robot Number: " + robot.RobotNumber);
			Trace.WriteLine("Enable Status: " + robot.Enabled);
			Trace.WriteLine("Model Number: " + robot.ModelNumber);
			Trace.WriteLine("Serial Number: " + robot.SerialNumber);
			
			// If not calibrated, calibrate the robot
			try {
				if (robot.IsCalibrated == false) {
					robot.Power = true;
					robot.Calibrate();
				}
			} catch (Exception ex) {
				Trace.WriteLine("calibrate error: "+ex.Message);
			}
			
			// Read various properties of the robot
			Trace.WriteLine("Number of joints = " + robot.JointCount);
		}
		
		public int motion_deinit(){
			robot.AutomaticControlActive = false;
			return 0;
		}
		
		public int controller_set_enable(bool en){
			robot.Controller.Enabled = en;
			return 0;
		}
		
		public int HighPower_set_enable(bool en){
			robot.Power = en;
			return 0;
		}
		
		public int motion_move(RobotCtrl ctrl){
			int ret=1;
			
			// Check if the current location is inrange            
			Transform3D currentPosition = robot.WorldLocation;
			int inRange = robot.InRange(currentPosition);
			Trace.WriteLine("cur_Pos="+currentPosition + " inrange check = " + inRange);

			// Create a cartesian move command for the robot
			Transform3D input = new Transform3D(ctrl.dx, ctrl.dy, ctrl.dz, ctrl.dr, 0, 0);
			Trace.WriteLine("input:"+input);
			var destinationPosition = input;
			if(ctrl.incremental){
				destinationPosition = currentPosition * input;
			}else{
				
			}
			
			if(destinationPosition.DZ < -970.0){ // 确保不撞传送带
				Trace.WriteLine("Z Exceed range!");
				return ret;
			}
			
			Transform3D r30 = Transform3D.RZ(-30) * destinationPosition;
			//Trace.WriteLine("==" + destinationPosition +"=="+r30);
			if(r30.DX > 396 || r30.DX < -396 || r30.DY > 396 || r30.DY < -396){ // 确保夹爪不撞框架 368*368+147*147
				Trace.WriteLine("XY Exceed range!");
				return ret;
			}

			ICartesianMove cartesianMove = robot.CreateCartesianMove(destinationPosition);
			Trace.WriteLine("dest_pos:"+cartesianMove.WorldLocation);
			cartesianMove.Param.Accel = ctrl.Accel;
			cartesianMove.Param.Decel = ctrl.Decel;
			cartesianMove.Param.Speed = ctrl.Speed;
			cartesianMove.Param.Straight = ctrl.Straight;
			cartesianMove.Param.MotionEnd = MotionEnd.Blend;
			cartesianMove.Param.SCurveProfile = ctrl.SCurveProfile;
			
			try {
				// Issue the move and wait until it is done
				robot.Move(cartesianMove);
				if(ctrl.wait_done){
					robot.WaitMoveDone();
				}
			
				// Force the robot to issue a DETACH
				//robot.AutomaticControlActive = false;
				ret = 0;
			} catch (Exception ex) {
				Trace.WriteLine("Error: "+ex.Message);
			}
			cartesianMove.Dispose();
			return ret;
		}
		
		public int parse_string(string input, ref string reply, ref RobotCtrl output){
			int ret = 1;
			if(input.EndsWith(">")){
				input = input.Replace(">", "");
			}else{
				reply = "command error";
				return ret;
			}
			
			string[] strValues =input.Split(':');
			string command = strValues[0];
			string[] parameters = strValues[1].Split(',');

			if(command == "R"){
				switch(strValues[1][0]){
					case 'p':
						//Transform3D currentPosition, basePosition, tooloffset;
						//double[] jointPosion = new double[4];
						//robot.GetPosition(out currentPosition, out basePosition, out tooloffset, out jointPosion);
						//Trace.WriteLine("-- "+currentPosition+" | "+basePosition+" | "+tooloffset+" | "+jointPosion[0]+jointPosion[1]+jointPosion[2]+jointPosion[3]);
						Transform3D currentPosition = robot.WorldLocation;
						reply = ACK_OK+"Position: "+currentPosition;
					break;
					case 'i':
						reply = ACK_OK;
						reply = reply + ",Controller: " + robot.Controller.Enabled;
						try{
							reply = reply + ",Robot Number: " + robot.RobotNumber;
							reply = reply + ",Enable Status: " + robot.Enabled;
							reply = reply + ",Model Number: " + robot.ModelNumber;
							reply = reply + ",Serial Number: " + robot.SerialNumber;
							reply = reply + ",Power: " + robot.Power;
						}catch(Exception ex) {
							Trace.WriteLine("robot info read error:"+ex.Message);
						}
						
					break;
				}
				
			}else if(command == "POWER"){
				int p = Convert.ToInt32(strValues[1]);
				ret = HighPower_set_enable(p!=0 ? true : false);
				reply = ret == 0?ACK_OK:ACK_ERROR;
			}else if(command == "CONTROLLER"){
				int p = Convert.ToInt32(strValues[1]);
				ret = controller_set_enable(p!=0 ? true : false);
				reply = ret == 0?ACK_OK:ACK_ERROR;
			}else if(command == "HALT"){
				robot.Halt();
				Trace.WriteLine("Robot halt ok");
				reply = ACK_OK;
			}else if(command == "MA" || command == "MI"){
				bool need_move=false;
				output.incremental=false;
				if(command == "MI"){
					output.incremental=true;
				}
				output.dx=0;
				output.dy=0;
				output.dz=0;
				output.dr=0;
				output.wait_done=true;

				for (int i=0; i<parameters.Length; i++){
					
					char[] s = parameters[i].ToCharArray();
					double v = Convert.ToDouble(parameters[i].Substring(1));
					
					if('x'==s[0]){
						output.dx = v;
						need_move=true;
					}else if('y'==s[0]){
						output.dy = v;
						need_move=true;
					}else if('z'==s[0]){
						output.dz = v;
						need_move=true;
					}else if('r'==s[0]){
						output.dr = v;
						need_move=true;
					}else if('f'==s[0]){
						output.Speed = (int)v;
					}else if('a'==s[0]){
						output.Accel = (int)v;
					}else if('d'==s[0]){
						output.Decel = (int)v;
					}else if('s'==s[0]){
						output.SCurveProfile = Math.Min(Math.Max((int)v, -1), 8);
					}else if('l'==s[0]){
						output.Straight = (0==(int)v ? false : true);
					}else if('w'==s[0]){
						output.wait_done = (0==(int)v ? false : true);
					}
				}
				ret = 0;
				if(need_move){
					ret = motion_move(output);
				}
				reply = (ret==0? ACK_OK : ACK_ERROR);
			}
			reply = "<"+reply+">";
			return ret;
		}

		public void my_main(){
			Trace.WriteLine("Script Starting");
			
			// Get a robot in the workspace
			robot = ace["R1 Hornet565"] as IAbstractHornetRobot;
			
			if (robot.IsAlive == false){
				Trace.WriteLine("Robot communications don't enabled, enable it now");
				controller_set_enable(true);
				Trace.WriteLine("Robot communications ok");
			}
			
			if (robot.Power == false){
				Trace.WriteLine("Robot power don't enabled, enable it now");
				HighPower_set_enable(true);
				Trace.WriteLine("Robot power ok");
			}
			
			RobotCtrl ctrlparam = new RobotCtrl();
			ctrlparam.Straight=true;
			ctrlparam.Speed=50;
			ctrlparam.Accel = 100;
			ctrlparam.Decel = 100;
			ctrlparam.SCurveProfile=0;

            // Establish TCP Listener for incoming connection
			int port_num=43000;
            TcpListener server = new TcpListener(System.Net.IPAddress.Any, port_num);
            server.Start();
			Trace.WriteLine("TCP Listener started");
            // Infinite 1oop 
            while (true){
				
                // Wait for the server to be ready 
                if(!server.Pending()){
					System.Threading.Thread.Sleep(20); 
					continue;
                }
				Trace.WriteLine("TCP Listener started"+" port:"+port_num);
                TcpClient client =server.AcceptTcpClient();
				Trace.WriteLine("Client "+client.Client.RemoteEndPoint.ToString());
                NetworkStream clientStream= client.GetStream();
				motion_init(); // enable motion
				
                bool newData = false;
                // Get the data in the socket
                byte[] myReadBuffer=new byte[16384];
				byte[] sendbuffer =new byte[16384];
				int bytesRead=0;
                
				while(true){
					Trace.WriteLine("wait for data");
					
					try{
						//Read Data
						bytesRead=clientStream.Read(myReadBuffer, 0, myReadBuffer.Length);
						if(bytesRead>0){
							System.Threading.Thread.Sleep(10);
							newData=true;
						}else{
							Trace.WriteLine("Disconnected");
							break;
						}

					}catch(Exception ex) {
						newData=false;
						//ace.AppendToLog("VData 1ost");
						//ace.AppendToLog(ex.Message); while(clientStream.DataAvailable); 
						Trace.WriteLine("Exception:" +ex.Message+"Disconnected");
						break;
					}
					
					// If data is recieved
					if (newData == true) {
						string msg =Encoding.ASCII.GetString(myReadBuffer,0,bytesRead);
						Trace.WriteLine("bytes:"+bytesRead.ToString()+"->"+msg);
						
						string[] commands =msg.Split('<');
						for (int i=1; i<commands.Length; i++){	
							string reply = 	Encoding.UTF8.GetString(sendbuffer);
							parse_string(commands[i], ref reply, ref ctrlparam);
							byte[] buffer = Encoding.UTF8.GetBytes(reply);
							try{
								clientStream.Write(buffer, 0, buffer.Length);
							}catch(Exception ex) {
								Trace.WriteLine("network Write error");
								break;
							}
						}
					}
				}
				clientStream.Close();
				client.Close();
				motion_deinit();
            }
			server.Stop();
		}

		public void Main () {
			
			my_main();
			
		}

		public void Dispose () {
			Trace.WriteLine("Dispose");
		}
	}
}