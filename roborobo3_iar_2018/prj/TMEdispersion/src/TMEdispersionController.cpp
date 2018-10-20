/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEdispersion/include/TMEdispersionController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEdispersionController::TMEdispersionController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEdispersionController::~TMEdispersionController()
{
    // nothing to do.
}

void TMEdispersionController::reset()
{
    // nothing to do.
}

int TMEdispersionController::isRobotAt(int id) {
	if(getRobotIdAt(id) != -1)
		return 1;
	return 0;
}

double TMEdispersionController::getmin(int a, int b, int c)
{
	double a_dist = getDistanceAt(a);
	double b_dist = getDistanceAt(b);
	double c_dist = getDistanceAt(c);
	if(isRobotAt(a) && (a_dist < b_dist || !isRobotAt(b)) && (a_dist < c_dist || !isRobotAt(c))){
		return a_dist;
	}
	else if(isRobotAt(b) && (b_dist < a_dist || !isRobotAt(a)) && (b_dist < c_dist || !isRobotAt(c))){
		return b_dist;
	}
	else if(isRobotAt(c) && (c_dist < a_dist || !isRobotAt(a)) && (c_dist < b_dist || !isRobotAt(b))){
		return c_dist;
	}
	return 10000; //no robot seen
}

void TMEdispersionController::step()
{  

    setTranslation( sin( ( ( getDistanceAt(SENSOR_FFL) + getDistanceAt(SENSOR_FFR) ) / 2.0 )* M_PI / 2.0) );
    
    double distanceOnMyLeft = getDistanceAt(SENSOR_L) + getDistanceAt(SENSOR_FL) + getDistanceAt(SENSOR_FFL);
    double distanceOnMyRight = getDistanceAt(SENSOR_R) + getDistanceAt(SENSOR_FR) + getDistanceAt(SENSOR_FFR);
    
    double rotationDelta = 0.3;
    double noiseAmplitude = 0.01;

    distanceOnMyRight *= 1;
    distanceOnMyLeft *= 1; 

    double mingauche = getmin(SENSOR_FFL, SENSOR_FL, SENSOR_L);
    double mindroit = getmin(SENSOR_FFR, SENSOR_FR, SENSOR_R);

    int robotagauche = (isRobotAt(SENSOR_FFL) || isRobotAt(SENSOR_FL) || isRobotAt(SENSOR_L));
    int robotadroite = (isRobotAt(SENSOR_FFR) || isRobotAt(SENSOR_FR) || isRobotAt(SENSOR_R));

	if(robotagauche) {
		if( !robotadroite){
		    setRotation( +rotationDelta); // tourne à droite
		}
		else if(mingauche < mindroit) {
		    setRotation( +rotationDelta); // tourne à droite
		}
		else {
		    setRotation( -rotationDelta); // tourne à gauche
		}

	}
	if(robotadroite) {
		if( !robotagauche){
		    setRotation( -rotationDelta); // tourne à gauche
		}
		else if(mindroit < mingauche){
		    setRotation( -rotationDelta); // tourne à gauche
		}
		else {
		    setRotation( +rotationDelta); // tourne à droite

		}

	}
}
