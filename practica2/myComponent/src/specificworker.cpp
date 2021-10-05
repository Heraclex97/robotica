/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#define MAX_ROT 2
#define MAX_ADV 1000

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}
void SpecificWorker::compute( ) {
    const float threshold = 300; // millimeters
    static float rot = MAX_ROT;  // rads per second
    float speed = 200;
    const float thSpiral = 2000;
    try
    {
            // read laser data
            //timer.interval() < this->Period
            RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
            //sort laser data from small to large distances using a lambda function.
            std::sort(ldata.begin()+10, ldata.end()-10,
                      [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });

        if (ldata[10].dist < threshold ) {
            currentS = State::SHOCK;
        }
        if (ldata[10].dist >thSpiral && currentS == State::STRAIGHT) {
            currentS = State::SPIRAL;
        }



        switch (currentS) {
            case State::SPIRAL:
                std::cout << "Spiral: " << ldata[10].dist << " " << rot << std::endl;
                if (rot > 1) {
                    speed = 300;
                    rot -= 0.005;
                }
                if (rot < 1) {
                rot -= rot / 200;
                speed = 500;
                }
                differentialrobot_proxy->setSpeedBase(speed, rot);
                break;

            case State::SHOCK:
                std::cout << "Shock: " << ldata[10].dist << " "<< std::endl;
                differentialrobot_proxy->setSpeedBase(5, 0.6);
                usleep(rand() % (1500000 - 100000 + 1) + 100000);
                currentS = State::STRAIGHT;
                break;

            case State::STRAIGHT:
                std::cout << "Straight: " << ldata[10].dist << " " << std::endl;
                differentialrobot_proxy->setSpeedBase(500, 0);
                rot = 1.4;
                break;
                
            default:
                std::cout << "Default: " << ldata[10].dist << std::endl;
                differentialrobot_proxy->setSpeedBase(800, 0);


            }
    }catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}



/**************************************/
//            case State::SHOCK: //Random shock
//                rot2 = -1.0 + (drand48() * 3.0);
//                std::cout << "Shock: " << ldata[10].dist << " "<<rot2<< std::endl;
//                differentialrobot_proxy->setSpeedBase(5, rot2);
//                usleep(rand() % (1500000 - 100000 + 1) + 100000);
//                currentS = State::STRAIGHT;
//                break;
//static float rot2 = rot;

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

