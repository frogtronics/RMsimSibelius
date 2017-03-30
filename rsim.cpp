//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdlib.h"
#include "string.h"
#include <mutex>
#include <time.h>
#include <stdio.h>
#include <tchar.h>
#include <string>
//
#include "SerialClass.h"
#include "Serial.h"
#include "compress12bit.h"

//for file input
#include <array>
#include<fstream>
#include <string>
#include <sstream>

//for keyboard input
#include <iostream>
#include <conio.h>
#include <ctype.h>

//-------------------------------- SERIAL PORT COMMUNICATION -------------------------------------
char incomingData[5] = "0000";			// don't forget to pre-allocate memory
int readResult = 0;
int timeout_counter = 0;
float forceIn;
float forcei;
float liveforce = 0;
float forceAdj = 0;
float forcePrev = 0;
float posOut;
float lf = 0;
char sendbuf[3]; // for position data out (rs232)
Serial* SP = new Serial("\\\\.\\COM1");    // initialize serial port

//----------------------------------------------------------------------------------------


//-------------------------------- global variables -------------------------------------

// synchronization
std::mutex gui_mutex;
char lastfile[1000] = "";

char error[1000];
// model
//mjModel* m = 0;
mjModel* m = mj_loadXML("3SegRMdevel01.xml", NULL, error, 1000);
mjData* d = 0;

//mjModel* m;
//mjData* d;
//save data
char const *outfile = "savedData.txt";
// char const *outfile_kinematics = "savedData_kinematics.txt";
//allocate array for output data
long double arrayout[10000] = {0};
float dataout[10000][20];
float pos_dataout[10000][20];

int counter = 0;
clock_t tStart = clock();
float realTime = 0;
float prevTime = 0;
int max_steps = 200;
float li = 0; //initial tendon length

std::array<std::array<float, 10>, 1> inputdata_qpos;//i rows, j cols
std::array<std::array<float, 3>, 1000> inputdata_inforces;//3 input forces, 1000 rows
std::ifstream datfile_qpos("qposInit.txt");
std::ifstream datfile_inforces("inforces.txt");

char opt_title[1000] = "";
char opt_content[1000];





// generate sinusoidal control waveform
void setcontrol(mjtNum time, mjtNum* ctrl, int nu)
{
    int switches[2] = {1};
	float force_offset = forcei;
	forceAdj = -liveforce * (forceIn - force_offset);
	// if ((counter != 0) & (forceIn > 1.2* abs(forcePrev))) {
		// forceAdj = -(forcePrev - force_offset);
	// }
	
	ctrl[0] = forceAdj;//ankle extensor
	ctrl[1] = inputdata_inforces[counter][1];//knee extensor force
	ctrl[2] = inputdata_inforces[counter][2];//hip extensor force
	forcePrev = forceIn;
    // for( int i=0; i<nu; i++ )
        //qfrc_applied[i] = mju_sin(time);
        //qfrc_applied[i] = -1* switches[i];
		
        // ctrl[i] = -(forceIn - 0.1);
		//printf("%f\n", forceAdj);
}

// simulation
void advance(void)
{
	forceIn = (decompress12bit(incomingData));
	
	
	
	//set initial pose
	// if (counter == 0) {
		// for (int i = 0; i<=10; i++) {
			// d->qpos[i] = inputdata_qpos[0][i];
		// }
	// }
    setcontrol(d->time, d->ctrl, m->nu);
    mj_step(m, d);
	if (counter == 0) {
		forcei = forceIn;
		li = d->sensordata[0];
		printf("Li = %f\n", li);
		printf("Li = %f\n", d->sensordata[0]);
	}
	
	if (counter == max_steps - 1) {
		lf = d->sensordata[0];
		
	}
	

    counter ++;
	
	
	incomingData[readResult] = 0;
	posOut = (d->sensordata[0] - li) * 500; //tendon length multiplied and offset to be near max of DUE dac output limit
	//posOut = forceIn; // for loopback testing
	

	compress12bit(sendbuf, posOut); //second joint angle
	sendbuf[2] = 65;
	SP->WriteData(sendbuf, sizeof(sendbuf));

	prevTime = realTime;
    realTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;//calculate real time sim takes
	// timeout_counter = 0;
	// while ((realTime - prevTime) == 0 && timeout_counter < 3000) {
		// timeout_counter ++;//wait
	// }


    arrayout[counter] = d->time;
    dataout[counter][0] = d->time;
    dataout[counter][1] = realTime;
	dataout[counter][4] = posOut;
	dataout[counter][5] = forceAdj;
	//dataout[counter][]
    for (int j = 0; j < (m->nbody - 1) * 3; j++) {
		pos_dataout[counter][j] = d->xpos[j + 3];//skip world body
	}
        


}





//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
    int maxsteps = 1000;
	// char input;
	// using namespace std;
	// cout << "hellofdgdf\n";
	// cin >> input;
	int ch = 89;
	_cputs("Press 'n' to ignore live force data\notherwise press any other key to start the simulation");

	while (ch == 89) {
		ch = _getch();
		if (ch == 110) {
			//n key
			liveforce = 0;
		}
		else {
			liveforce = 1;
		}
	}

    // print version, check compatibility
    printf("MuJoCo Pro library version %.2lf\n\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");

    // activate MuJoCo license
    mj_activate("mjkey.txt");
    
    // install control callback
    //mjcb_control = mycontroller;

    
    //glfwSetDropCallback(window, drop);
   

    //loadmodel(window, "nothing.nothing", 0);


    mjModel* mnew = 0;
    
    mnew = mj_loadXML("3SegRMdevel01.xml", 0, error, 1000);

    
    {
        printf("%s\n", error);

    }
	//load initial pose data
	for (int i = 0; i <= 1; i++) {
		for (int j = 0; j <= 10; j++) {
			datfile_qpos >> inputdata_qpos[i][j];
		}
	}
	// load force data from other muscles - note we ignore the first column - it's a placeholder for the ankle
	for (int i = 0; i < 1000; i++) {
		for (int j = 0; j < 3; j++) {
			datfile_inforces >> inputdata_inforces[i][j];
		}
	}	

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    //m = mj_loadXML("hello.xml", NULL, error, 1000);
    d = mj_makeData(m);
    mj_step(m, d);



    printf("fool...\n");
	printf("nq %i\n", m->nq);
	printf("nv %i\n", m->nv);
	printf("nbody %i\n", m->nbody);
	printf("nv %i\n", m->nv);
    //mju_copy(d->qpos, m->key_qpos, m->nq*1);// tells MJ to load keyframe angles!!
	//---MUST set qpos0 as reference position.  MJ ignores the free joint, so skip that
	for (int i = 0; i<=m->nq; i++) {
		m->qpos0[i] = inputdata_qpos[0][i];
	}	
	//now account for free joint
	for (int i = 0; i < 7; i++) {
		d->qpos[i] = inputdata_qpos[0][i];
	}
	printf("pos ref\n");
	for (int i = 0; i < m->nq; i++) {
		printf("%f\n", m->qpos0[i]);
	}
    // main loop
    
    tStart = clock();
	
	
    // while( counter <= maxsteps )
    // {
        // advance();  

    // }
	if (SP->IsConnected()) {
		printf("We're connected\n");
		// printf("Five\n");
		// Sleep(1000);
		// printf("Four\n");
		// Sleep(1000);
		// printf("Three\n");
		// Sleep(1000);
		// printf("Two\n");
		// Sleep(1000);
		// printf("One\n");
		// Sleep(1000);
	  }

	 while (SP->IsConnected() && counter < max_steps) 
	 {
		while (SP->ReadData(incomingData, 3) == 3 && incomingData[2] == 65)
		{
			// forceIn = (decompress12bit(incomingData));
			// incomingData[readResult] = 0;
			timeout_counter = 0;
			// while ((realTime - prevTime) < 0.004 && timeout_counter < 3000) {
				// timeout_counter ++;//wait
				// printf("%f\n", realTime - prevTime);
			// }
			advance();
			// printf("%f\n", posOut);
			//printf("%f\n", d->sensordata[0]);
			//mj_step(m, d);
			//counter ++;
			// posOut = 0.001 * counter; //d->qpos[1]; //second joint angle
			// compress12bit(sendbuf, posOut); //second joint angle
			// sendbuf[2] = 65;
			// SP->WriteData(sendbuf, sizeof(sendbuf));
			//printf("%f\n", posOut);
		}
	 }
	 //RESET to initial offset value so AURURA doesn't jump!!
	compress12bit(sendbuf, 2.75); //second joint angle
	sendbuf[2] = 65;
	SP->WriteData(sendbuf, sizeof(sendbuf));
	 
    //mj_printData(m, d, outfile);
    printf("quat1 %f %f %f %f\n", d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3]);
    printf("%d %f\n", counter, d->time);
    printf("%d\n", m->nu);
    printf("size %d \n", m->nq);

    FILE *f = fopen("savedData.txt", "w");//a for append, t for text mode
    //fwrite(arrayout, sizeof(char), sizeof(arrayout), f);
    
    for(int i = 1; i < max_steps; i++) {
        for (int j = 0; j < m->nq + 4; j++)
           fprintf(f, "%f\t", dataout[i][j]); 
        fprintf(f, "\n");

        //fprintf(f, "%Lf\n", arrayout[i]);
    }
	fclose(f);
	
	FILE *f1 = fopen("savedData_kinematics.txt", "w");//a for append, t for text mode
    //fwrite(arrayout, sizeof(char), sizeof(arrayout), f);
    
    for(int i = 1; i < max_steps; i++) {
        for (int j = 0; j < (m->nbody - 1) * 3; j++)
           fprintf(f1, "%f\t", pos_dataout[i][j]); 
        fprintf(f1, "\n");

    }
	fclose(f1);
	
	// char incomingData[256] = "";	
	// int counter = 0;
	// Serial* SP = new Serial("\\\\.\\COM1");    // initialize serial port
	 // if (SP->IsConnected()) {
		// printf("We're connected\n"); 
	 // }
	// else {
		// printf(".....!NOT CONNECTED!....");
	// }	
	// while (SP->ReadData(incomingData, 1) > 0 && counter < 1000) {
		// counter ++;
	// }
	

	


	// if (SP->IsConnected()) {
		// printf("We're connected\n"); 
	 // }
	 // while (SP->IsConnected() && scounter < 3000) 
	 // {
		// while (SP->ReadData(incomingData, 3) == 3 && incomingData[2] == 65)
		// {
			// scounter ++;
			// forceIn = (decompress12bit(incomingData));
			// incomingData[readResult] = 0;
			// printf("%f\n", forceIn);

		// }
	 // }
	
	
    printf("Simulation run time %f sec, Lfinal = %f\n", realTime, lf);
	printf("Forcei = %f \n", forcei);
    //fprintf(f, "%Lf\n", arrayout[0]);
    



    // delete everything we allocated
 

    // terminate

    mj_deactivate();
    


    return 0;
}

