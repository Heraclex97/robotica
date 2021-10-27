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


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
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
    QRectF *dimensions = new QRectF(-5000,-2500,10000,5000);
    viewer = new AbstractGraphicViewer(this, *dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    //    Cada vez que se pulse la pantalla, se crea la ecuación general de la recta
    line.A=bState.z-target.pos.y();
    line.B=target.pos.x()-bState.x;
    line.C=(bState.x-target.pos.x())*bState.z+(target.pos.y()-bState.z)*bState.x;


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

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState baseState;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        std::sort(ldata.begin()+10, ldata.end()-10,
                  [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
    try
    {
        differentialrobot_proxy->getBaseState(baseState);
        robot_polygon->setRotation(baseState.alpha*180/M_PI);
        robot_polygon->setPos(baseState.x, baseState.z);
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
    //p4
    //maquina de estados ,idle(esperar),avanzar(si choque sale a otro estado bordear), bordear (haber llegado al target, tener target a la vista, atravesar la linea de target te devuelve a la linea principal
    QPointF ppr;
    float d;
    float min = std::min_element(ldata.begin()+10, ldata.end()-10,
                     [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
    switch (currentS)
    {
        case State::IDLE:
            differentialrobot_proxy->setSpeedBase(0, 0);

            if (target.active && currentS != State::SHOCK)
                currentS = State::GOTO;
            break;

        case State::GOTO:
            gotoTarget(baseState,ppr,ldata);
            break;

        case State::SHOCK:
            doShock(ldata);
            break;

        case State::DODGE:
            std::cout << "Dodge " << std::endl;
//            d = abs(line.A*posx+line.B*posy+line.C)/ sqrt(pow(line.A,2)+ pow(line.B,2));
//            if (d < umbral) currentS = State::GOTO;
            currentS = State::IDLE;
            break;
    }
}

void SpecificWorker::gotoTarget(RoboCompGenericBase::TBaseState baseState,QPointF pr,RoboCompLaser::TLaserData ldata)
{
    float mod;
    float s;
    float reduce_speed_if_turning;
    float adv;
    float beta;

    //pasar target a coordenadas del robot (está en coordenadas del mundo)
    pr = world_to_robot(baseState, target);//devuelve un QPointF
    mod = sqrt(pow(pr.x(),2)+pow(pr.y(),2));
    //calcular el ang que forma el robot con el target deltaRot1
    beta = atan2(pr.x(),pr.y()); //velocidad de giro
    //calcular una velocidad de avance que depende de la distancia y si se esta girando
    s = 0.1;
    reduce_speed_if_turning = exp(-pow(beta,2)/s);
    adv = MAX_ADV_VEL * reduce_speed_if_turning * reduce_speed_if_close_to_target(mod);

    if (ldata[10].dist < 500)
    {
        differentialrobot_proxy->setSpeedBase(0, beta);
        currentS = State::SHOCK;
    }

    try
    {
        if(mod<=150)
        {
            beta = 0;
            target.active = false;
            currentS = State::IDLE;
        }
        differentialrobot_proxy->setSpeedBase(adv, beta);
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::doShock(RoboCompLaser::TLaserData ldata)
{
    std::cout << "Shock " << std::endl;
    differentialrobot_proxy->setSpeedBase(0, 0.6);
    usleep(rand() % (1500000 - 100000 + 1) + 100000);
    if (ldata[10].dist > 300)
        currentS = State::DODGE;
}

void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo()<<t;
    target.pos = t;
    target.active = true;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    float x,y;
    // code to delete any existing laser graphic element
    if(laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &p : ldata)
    {
        x = p.dist * sin(p.angle);
        y = p.dist * cos(p.angle);
        poly << QPointF(x,y);
    }

    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's  // reference system
    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

QPointF SpecificWorker::world_to_robot(RoboCompGenericBase::TBaseState state, SpecificWorker::Target target)
{
//    declarar matriz, con el angulo y la pos libreria de algebra lineal (mult por vector)
    float alfa = state.alpha;
    Eigen::Vector2f TW(target.pos.x(),target.pos.y()); //target
    Eigen::Vector2f RW(state.x,state.z); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = sin(alfa);
    R(1,0) = -sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * (TW-RW);

    actual_point = QPointF(TR.x(),TR.y());

    return actual_point;
}

float SpecificWorker::reduce_speed_if_close_to_target(float mod)
{

    if ( mod<=150){
        return 0;
    }else if ( mod >1000){
        return 1;
    }
    return 0.5;
}

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

