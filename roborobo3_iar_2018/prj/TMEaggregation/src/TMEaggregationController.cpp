/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEaggregation/include/TMEaggregationController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEaggregationController::TMEaggregationController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEaggregationController::~TMEaggregationController()
{
    // nothing to do.
}

void TMEaggregationController::reset()
{
    // nothing to do.
}

int TMEaggregationController::isRobotAt(int id) {
	if(getRobotIdAt(id) != -1)
		return 1;
	return 0;
}

double TMEaggregationController::getmin(int a, int b, int c)
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

void TMEaggregationController::step()
{
	// * How to use sensors to drive the robot
    // This is an example that code for wall avoidance
    
    setTranslation( sin( ( ( getDistanceAt(SENSOR_FFL) + getDistanceAt(SENSOR_FFR) ) / 2.0 )* M_PI / 2.0) );
    
    double distanceOnMyLeft = getDistanceAt(SENSOR_L) + getDistanceAt(SENSOR_FL) + getDistanceAt(SENSOR_FFL);
    double distanceOnMyRight = getDistanceAt(SENSOR_R) + getDistanceAt(SENSOR_FR) + getDistanceAt(SENSOR_FFR);
    
    double rotationDelta = 0.3;
    double noiseAmplitude = 0.01;

    double mingauche = getmin(SENSOR_FFL, SENSOR_FL, SENSOR_L);
    double mindroit = getmin(SENSOR_FFR, SENSOR_FR, SENSOR_R);

    /*

    if(mingauche != 10000) // robot at left
    {
    	if(mingauche < mindroit) {
    		 setRotation( -rotationDelta ); // tourne a gauche
    	}
    }
    else if(mindroit != 10000) // robot at right
    {
    	if(mingauche > mindroit) {
    		 setRotation( +rotationDelta ); // tourne a droite
    	}
    }
    else if(isRobotAt(SENSOR_BR)) {
    	if(!isRobotAt(SENSOR_BL) || getDistanceAt(SENSOR_BR) < getDistanceAt(SENSOR_BL)) {
    		 setRotation( +rotationDelta ); // tourne a droite
    	}
    }
    else if(isRobotAt(SENSOR_BL)) {
    	if(!isRobotAt(SENSOR_BR) || getDistanceAt(SENSOR_BR) > getDistanceAt(SENSOR_BL)) {
    		 setRotation( -rotationDelta ); // tourne a gauche
    	}
    }
    else {
		//rotation aléatoire
		//setRotation( noiseAmplitude * ( 1.0 - (double)(random01()*2.0) ) );

		//evitement des murs braitenberg
	    setRotation( +rotationDelta * (distanceOnMyRight - distanceOnMyLeft )*0.1);
	}
*/
  
    //aggregation qui bouge pas
/*
	if(isRobotAt(SENSOR_FFL) || isRobotAt(SENSOR_FL) || isRobotAt(SENSOR_L)) {
 		//robot sur la gauche
	    setRotation( +rotationDelta * (mingauche - mindroit));
	}
	else if(isRobotAt(SENSOR_FFR) || isRobotAt(SENSOR_FR) || isRobotAt(SENSOR_R)) {
		//robot à droite
	    setRotation( -rotationDelta * (mingauche - mindroit));
	}
	else {
		//rotation aléatoire
		//setRotation( noiseAmplitude * ( 1.0 - (double)(random01()*2.0) ) );

		//evitement des murs braitenberg
	    setRotation( +rotationDelta * (distanceOnMyRight - distanceOnMyLeft ));
	}
*/
	//agrégation qui tourne
	

    int robotagauche = (isRobotAt(SENSOR_FFL) || isRobotAt(SENSOR_FL) || isRobotAt(SENSOR_L));
    int robotadroite = (isRobotAt(SENSOR_FFR) || isRobotAt(SENSOR_FR) || isRobotAt(SENSOR_R));

    int iswall = (getWallAt(SENSOR_FFL) || getWallAt(SENSOR_FL) || getWallAt(SENSOR_L) || getWallAt(SENSOR_FFR) || getWallAt(SENSOR_FR) || getWallAt(SENSOR_R));

    //version aggregation braitenberg
	if(robotagauche) {
		if( !robotadroite){
	 		//robot sur la gauche
		    setRotation( -rotationDelta);
		}
		else {
		   	 setRotation( -rotationDelta * (mingauche - mindroit));
		}

	}
	if(robotadroite) {
		if( !robotagauche){
		    setRotation( +rotationDelta);
		}
		else {
		   	 setRotation( -rotationDelta * (mindroit - mingauche));
		}

	}
	else if(iswall){
   		 setRotation( +rotationDelta * (distanceOnMyRight - distanceOnMyLeft ));
	}
	else {
		setRotation(0);
	}
	
	

}

