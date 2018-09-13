#include "Common.h"
#include "Trajectory.h"
#include "Utility/Timer.h"

#include "Drawing/Visualizer_GLUT.h"
#include "Simulation/QuadDynamics.h"
#include "Simulation/Simulator.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Drawing/GraphManager.h"
#include "MavlinkNode/MavlinkTranslation.h"
#include "Simulation/SimulatedGPS.h"


using SLR::Quaternion;
using SLR::ToUpper;
using namespace std;

void KeyboardInteraction(V3F& force, shared_ptr<Visualizer_GLUT> vis);
bool receivedResetRequest = true;
bool paused = false;
void PrintHelpText();
void ProcessConfigCommands(shared_ptr<Visualizer_GLUT> vis);
void LoadScenario(string scenarioFile);
void ResetSimulation();

vector<QuadcopterHandle> quads;

shared_ptr<Visualizer_GLUT> visualizer;
shared_ptr<GraphManager> grapher;

float dtSim = 0.001f;
const int NUM_SIM_STEPS_PER_TIMER = 5;
Timer lastDraw;
V3F force, moment;

float simulationTime=0;
int randomNumCarry=-1;

void OnTimer(int v);

vector<QuadcopterHandle> CreateVehicles();
string _scenarioFile="../config/01_Intro.txt";

#include "MavlinkNode/MavlinkNode.h"
shared_ptr<MavlinkNode> mlNode;

int main(int argcp, char **argv)
{
	PrintHelpText();

	// load parameters
	ParamsHandle config = SimpleConfig::GetInstance();

	// initialize visualizer
	visualizer.reset(new Visualizer_GLUT(&argcp, argv));
	grapher.reset(new GraphManager(false));

	// re-load last opened scenario
	FILE *f = fopen("../config/LastScenario.txt", "r");
	if (f)
	{
		char buf[100]; buf[99] = 0;
		fgets(buf, 99, f);
		_scenarioFile = SLR::Trim(buf);
		fclose(f);
	}

	LoadScenario(_scenarioFile);

  glutTimerFunc(1,&OnTimer,0);
  
  glutMainLoop();

  return 0;
}

/// ############################################################
/// Calculate the standard deviation of a vector

float calculate_STD(vector<float> &data) {
	float sum = 0;
	vector<float>::iterator iter;
	for (iter = data.begin(); iter != data.end(); ++iter) {
		sum += *iter;
	}
	float mean = sum / data.size();

	float diff = 0;
	for (iter = data.begin(); iter != data.end(); ++iter) {
		diff += (*iter - mean)*(*iter - mean);
	}
	float variance = diff / (data.size() - 1);
	float dataSTD = sqrt(variance);

	return dataSTD;
}

/// ############################################################

void LoadScenario(string scenarioFile)
{
  FILE *f = fopen("../config/LastScenario.txt","w");
  if(f)
  {
    fprintf(f, "%s", scenarioFile.c_str());
    fclose(f);
  }

  ParamsHandle config = SimpleConfig::GetInstance();
  _scenarioFile = scenarioFile;
  config->Reset(scenarioFile);

  grapher->graph1->RemoveAllElements();
  grapher->graph2->RemoveAllElements();


  // create a quadcopter to simulate
  quads = CreateVehicles();

  ResetSimulation();

  /// ############################################################
  /// Additional part written to calculate the standard deviation of sensor measurements
  /// works in with "calculate_STD" function to find the std

  string scenario_flag = "../config/06_SensorNoise.txt";
  int flag = _scenarioFile.string::compare(scenario_flag);
  if (flag == 0 ) {
	  ifstream QuadGPS_x_stream("../config/log/Scenario6-Quad-GPS_x.txt");
	  ifstream IMU_Accel_x_stream("../config/log/Scenario6-IMU-Accel_x.txt");

	  string str_Quad_IMU_AX, str_Quad_GPS_X, str_time_step;
	  float read_Quad_IMU_AX, read_Quad_GPS_X;
	  vector<float> Quad_GPS_X;
	  vector<float> Quad_IMU_Ax;
	  string line;
	  while (getline(QuadGPS_x_stream, line)) {
		  stringstream ss(line);
		  getline(ss, str_time_step, ',');
		  getline(ss, str_Quad_GPS_X, '\n');
		  read_Quad_GPS_X = strtof(str_Quad_GPS_X.c_str(), NULL);
		  Quad_GPS_X.push_back(read_Quad_GPS_X);
	  }
	  cout << calculate_STD(Quad_GPS_X) << endl;

	  while (getline(IMU_Accel_x_stream, line)) {
		  stringstream ss(line);
		  getline(ss, str_time_step, ',');
		  getline(ss, str_Quad_IMU_AX, '\n');
		  read_Quad_IMU_AX = strtof(str_Quad_IMU_AX.c_str(), NULL);
		  Quad_IMU_Ax.push_back(read_Quad_IMU_AX);
	  }
	  cout << calculate_STD(Quad_IMU_Ax) << endl;

	  QuadGPS_x_stream.close();
	  IMU_Accel_x_stream.close();
  }

  /// #################################################################


  visualizer->OnLoadScenario(_scenarioFile);
  visualizer->InitializeMenu(grapher->GetGraphableStrings());
  visualizer->quads = quads;
  visualizer->graph = grapher;

  ProcessConfigCommands(visualizer);

  mlNode.reset();
  if(config->Get("Mavlink.Enable",0)!=0)
  { 
    mlNode.reset(new MavlinkNode());
  }

  
}

int _simCount = 0;

void ResetSimulation()
{
  _simCount++;
  ParamsHandle config = SimpleConfig::GetInstance();

  printf("Simulation #%d (%s)\n", _simCount, _scenarioFile.c_str());

  randomNumCarry = -1;

  receivedResetRequest = false;
  simulationTime = 0;
  config->Reset(_scenarioFile);
  dtSim = config->Get("Sim.Timestep", 0.005f);

  for (unsigned i = 0; i < quads.size(); i++)
  {
    quads[i]->Reset();
  }
  grapher->Clear();

  // reset data sources
  grapher->_sources.clear();
  grapher->RegisterDataSource(visualizer);
  for (auto i = quads.begin(); i != quads.end(); i++)
  {
    grapher->RegisterDataSource(*i);
    grapher->RegisterDataSources((*i)->sensors);
    grapher->RegisterDataSource((*i)->estimator);
		grapher->RegisterDataSource((*i)->controller);
  }
}

void OnTimer(int)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  
  // logic to reset the simulation based on key input or reset conditions
  float endTime = config->Get("Sim.EndTime",-1.f);
  if(receivedResetRequest ==true ||
     (ToUpper(config->Get("Sim.RunMode", "Continuous"))=="REPEAT" && endTime>0 && simulationTime >= endTime))
  {
    ResetSimulation();
  }
  
  visualizer->OnMainTimer();
  
  // main loop
  if (!paused)
  {
    for (int i = 0; i < NUM_SIM_STEPS_PER_TIMER; i++)
    {
      for (unsigned i = 0; i < quads.size(); i++)
      {
        quads[i]->Run(dtSim, simulationTime, randomNumCarry, force, moment);
      }
      simulationTime += dtSim;
    }
    grapher->UpdateData(simulationTime);
  }
  
  KeyboardInteraction(force, visualizer);
  
  if (lastDraw.ElapsedSeconds() > 0.030)
  {
    if (quads.size() > 0)
    {
      visualizer->SetArrow(quads[0]->Position() - force, quads[0]->Position());
    }
    visualizer->Update(simulationTime);
    grapher->DrawUpdate();
    lastDraw.Reset();

    // temporarily here
    if (mlNode)
    {
      mlNode->Send(MakeMavlinkPacket_Heartbeat());
      mlNode->Send(MakeMavlinkPacket_Status());
      mlNode->Send(MakeMavlinkPacket_LocalPose(simulationTime, quads[0]->Position(), quads[0]->Velocity()));
      mlNode->Send(MakeMavlinkPacket_Attitude(simulationTime, quads[0]->Attitude(), quads[0]->Omega()));
    }
    
  }
  
  glutTimerFunc(5,&OnTimer,0);
}

vector<QuadcopterHandle> CreateVehicles()
{
  vector<QuadcopterHandle> ret;

  ParamsHandle config = SimpleConfig::GetInstance();
  int i = 1;
  while (1)
  {
    char buf[100];
    sprintf_s(buf, 100, "Sim.Vehicle%d", i);
    if (config->Exists(buf))
    {
      QuadcopterHandle q = QuadDynamics::Create(config->Get(buf, "Quad"), (int)ret.size());
      ret.push_back(q);
    }
    else
    {		
      break;
    }
    i++;
  }
  return ret;

}

void KeyboardInteraction(V3F& force, shared_ptr<Visualizer_GLUT> visualizer)
{
  bool keyPressed = false;
  const float forceStep = 0.04f;

  if (visualizer->IsSpecialKeyDown(GLUT_KEY_LEFT))
  {
    force += V3F(0, -forceStep, 0);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_UP))
  {
    force += V3F(0, 0, -forceStep);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_RIGHT))
  {
    force += V3F(0, forceStep, 0);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_DOWN))
  {
    force += V3F(0, 0, forceStep);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('w') || visualizer->IsKeyDown('W'))
  {
    force += V3F(forceStep, 0, 0);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('s') || visualizer->IsKeyDown('S'))
  {
    force += V3F(-forceStep, 0, 0);
    keyPressed = true;
  }

  if (!keyPressed)
  {
    force = V3F();
  }
  if (force.mag() > 2.f)
  {
    force = force / force.mag() * 2.f;
  }

  if (visualizer->IsKeyDown('c') || visualizer->IsKeyDown('C'))
  {
    visualizer->graph->graph1->RemoveAllElements();
    visualizer->graph->graph2->RemoveAllElements();
  }

  if (visualizer->IsKeyDown('r') || visualizer->IsKeyDown('R'))
  {
    receivedResetRequest = true;
  }

  static bool key_space_pressed = false;

  if (visualizer->IsKeyDown(' '))
  {
    if (!key_space_pressed)
    {
      key_space_pressed = true;
      paused = !paused;
      visualizer->paused = paused;
    }
  }
  else
  {
    key_space_pressed = false;
  }
}

void ProcessConfigCommands(shared_ptr<Visualizer_GLUT> vis)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  int i = 1;
  while (1)
  {
    char buf[100];
    sprintf_s(buf, 100, "Commands.%d", i);
    string cmd = config->Get(buf, "");
    if (cmd == "") break;
    vis->OnMenu(cmd);
    i++;
  }
}

void PrintHelpText()
{
  printf("SIMULATOR!\n");
  printf("Select main window to interact with keyboard/mouse:\n");
  printf("LEFT DRAG / X+LEFT DRAG / Z+LEFT DRAG = rotate, pan, zoom camera\n");
  printf("W/S/UP/LEFT/DOWN/RIGHT - apply force\n");
  printf("C - clear all graphs\n");
  printf("R - reset simulation\n");
  printf("Space - pause simulation\n");
}
