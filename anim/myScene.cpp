////////////////////////////////////////////////////
// // Template code for  CS 174C
////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <shared/defs.h>

#include "shared/opengl.h"

#include <string.h>
#include <util/util.h>
#include <GLModel/GLModel.h>
#include "anim.h"
#include "animTcl.h"
#include "myScene.h"
#include "SampleParticle.h"
#include "SampleGravitySimulator.h"
//#include <util/jama/tnt_stopwatch.h>
//#include <util/jama/jama_lu.h>

//#include "Hermite.h"
#include "Classroom.h"
#include "Bob.h"
#include "IKSim.h"
#include "HermiteSystem.h"

// register a sample variable with the shell.
// Available types are:
// - TCL_LINK_INT 
// - TCL_LINK_FLOAT

int g_testVariable = 10;

SETVAR myScriptVariables[] = {
	"testVariable", TCL_LINK_INT, (char *) &g_testVariable,
	"",0,(char *) NULL
};


//---------------------------------------------------------------------------------
//			Hooks that are called at appropriate places within anim.cpp
//---------------------------------------------------------------------------------

/*
void screenToWorld(int x, int y, double& worldX, double& worldY) {
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble worldZ;

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	winZ = 0.0;

	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &worldX, &worldY, &worldZ);
}
*/
bool isInRange(int value, int min, int max) {
	return value >= min && value <= max;
}


// start or end interaction
void myMouse(int button, int state, int x, int y)
{

	HermiteSystem* part1 = dynamic_cast<HermiteSystem*>(GlobalResourceManager::use()->getSystem("hermite"));

	bool inblackboard = isInRange(x, -9, 9) && isInRange(y, -6, 6);

	if (inblackboard && part1 && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		//double worldX, worldY;
		//screenToWorld(x, y, worldX, worldY);
		part1->addControlPoint(x, y, 0);
		animTcl::OutputMessage("Current (x, y) is: (%i, %i)", x, y);
	}
	else if (!inblackboard) {
		animTcl::OutputMessage("Please click on the blackboard to add control points");
	}
	else
	{
			animTcl::OutputMessage("Could not find the Hermite system\n");
		}

	// let the global resource manager know about the new state of the mouse 
	// button
	GlobalResourceManager::use()->setMouseButtonInfo( button, state );

	if( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		animTcl::OutputMessage(
			"My mouse received a mouse button press event\n");

	}
	if( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		animTcl::OutputMessage(
			"My mouse received a mouse button release event\n") ;
	}
}	// myMouse

// interaction (mouse motion)
void myMotion(int x, int y)
{

	GLMouseButtonInfo updatedMouseButtonInfo = 
		GlobalResourceManager::use()->getMouseButtonInfo();

	if( updatedMouseButtonInfo.button == GLUT_LEFT_BUTTON )
	{
		animTcl::OutputMessage(
			"My mouse motion callback received a mousemotion event\n") ;
	}

}	// myMotion

void MakeScene(void)
{
	bool success;

	Classroom* classroom= new Classroom("classroom");
	success = GlobalResourceManager::use()->addSystem(classroom, true);
	assert(success);

	HermiteSystem* hermite = new HermiteSystem("hermite");
	success = GlobalResourceManager::use()->addSystem(hermite, true);
	assert(success);

	Bob* bob = new Bob("bob");
	success = GlobalResourceManager::use()->addSystem(bob, true);
	assert(success);

	IKSim* iksim = new IKSim("iksim", bob);
	success = GlobalResourceManager::use()->addSimulator(iksim);
	assert(success);

	iksim->setHermite(hermite);


	

}	// MakeScene

// OpenGL initialization
void myOpenGLInit(void)
{
	animTcl::OutputMessage("Initialization routine was called.");

}	// myOpenGLInit

void myIdleCB(void)
{
	
	return;

}	// myIdleCB

void myKey(unsigned char key, int x, int y)
{
	 animTcl::OutputMessage("My key callback received a key press event\n");
	return;

}	// myKey

static int testGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	 animTcl::OutputMessage("This is a test command!");
    animTcl::OutputResult("100") ;
	return TCL_OK;

}	// testGlobalCommand

void mySetScriptCommands(Tcl_Interp *interp)
{

	// here you can register additional generic (they do not belong to any object) 
	// commands with the shell

	Tcl_CreateCommand(interp, "test", testGlobalCommand, (ClientData) NULL,
					  (Tcl_CmdDeleteProc *)	NULL);

}	// mySetScriptCommands
