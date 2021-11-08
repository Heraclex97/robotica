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
    float adv = 200;
    static float beta = 2;
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
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

    switch (currentS)
    {
        case State::IDLE:
//            std::cout << "Idle " << std::endl;
            movDodge = 0;
            if (target.active)
                currentS = State::GOTO;
            break;

        case State::GOTO:
//            std::cout << "Goto " << std::endl;
            gotoTarget(ldata,baseState,ppr,adv,beta);
            break;

        case State::SHOCK:
//            std::cout << "Shock " << std::endl;
            doShock(ldata);
            break;

        case State::DODGE:
//            std::cout << "Dodge " << movDodge << std::endl;
            doDodge(ldata, baseState,300, 0.7);
            break;
    }
}

void SpecificWorker::gotoTarget(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState baseState,QPointF pr, float adv, float beta)
{
    float mod;
    float s;
    float reduce_speed_if_turning;

    //pasar target a coordenadas del robot (está en coordenadas del mundo)
    pr = world_to_robot(baseState, target);//devuelve un QPointF
    mod = sqrt(pow(pr.x(),2)+pow(pr.y(),2));
    //calcular el ang que forma el robot con el target deltaRot1
    beta = atan2(pr.x(),pr.y()); //velocidad de giro
    //calcular una velocidad de avance que depende de la distancia y si se esta girando
    s = 0.1;
    reduce_speed_if_turning = exp(-pow(beta,2)/s);
    adv = MAX_ADV_VEL * reduce_speed_if_turning * reduce_speed_if_close_to_target(mod);

    if (obstacle_ahead(ldata, 700))
    {
        differentialrobot_proxy->setSpeedBase(adv-200, beta);
        currentS = State::SHOCK;
    }
    else {

        try {
            if (mod <= 150) {
                beta = 0;
                target.active = false;
                currentS = State::IDLE;
            }
            differentialrobot_proxy->setSpeedBase(adv, beta);
        }
        catch (const Ice::Exception &ex) {
            std::cout << ex << std::endl;
        }
    }
}

void SpecificWorker::doShock(const RoboCompLaser::TLaserData &ldata)
{
    if (!obstacle_ahead(ldata,1000,10)) {
        std::cout << "Dodge " << std::endl;
        currentS = State::DODGE;
    }
    else
        differentialrobot_proxy->setSpeedBase(0,0.6);
}

void SpecificWorker::doDodge(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState &base, float speed, float rot)
{
    if(check_free_path_to_target(ldata) && lateral_distance(ldata,300,true)) {
        std::cout << "Goto por check" << std::endl;
        currentS = State::GOTO;
    }
    else if (line_dist(base,10) && movDodge > 15) {
        std::cout << "Goto por line_dist " << movDodge << std::endl;
        currentS = State::GOTO;
        movDodge = 0;
    } else {
        movDodge++;
        if (lateral_distance(ldata,500,true))
            differentialrobot_proxy->setSpeedBase(speed, rot);
        else
            differentialrobot_proxy->setSpeedBase(speed, -rot);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////
bool SpecificWorker::line_dist(RoboCompGenericBase::TBaseState &base, int threshold)
{
    auto d = abs(line.A*robot_polygon->x()+line.B*robot_polygon->y()+line.C)/ sqrt(pow(line.A,2)+ pow(line.B,2));
    if (d < threshold)    return true;
    else    return false;
}

bool SpecificWorker::obstacle_ahead(const RoboCompLaser::TLaserData &ldata, int dist, int semiwidth)
{
    auto min = std::min_element(ldata.begin()+(ldata.size()/2 - semiwidth), ldata.end()-(ldata.size()/2 + semiwidth), [](auto a, auto b) { return a.dist < b.dist; });
    return (*min).dist < dist;
}

bool SpecificWorker::lateral_distance(const RoboCompLaser::TLaserData &ldata, int dist, bool left)
{
    if (left)
        return (std::min_element(ldata.begin()+35, ldata.begin()+65, [](auto a, auto b) { return a.dist < b.dist; }))->dist < dist;
    else
        return (std::min_element(ldata.end()-65, ldata.end()-35, [](auto a, auto b) { return a.dist < b.dist; }))->dist < dist;
}

bool SpecificWorker::check_free_path_to_target( const RoboCompLaser::TLaserData &ldata)
{
    // lambda to convert from Eigen to QPointF
    auto toQPointF = [](const Eigen::Vector2f &p){ return QPointF(p.x(),p.y());};
    bool reachable = true;

    // create polyggon
    QPolygonF pol;
    pol << QPointF(0,0);
    for(const auto &l: ldata)
        pol << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));

    // create tube lines
    Eigen::Vector2f robot(robot_polygon->x(),robot_polygon->y());
    Eigen::Vector2f robot2(0.0,0.0);
    QPointF tar = world_to_robotTest(robot,target);
    Eigen::Vector2f goal_r(tar.x(),tar.y());
    // number of parts the target vector is divided into
    float parts = (goal_r).norm()/(ROBOT_LENGTH/4);
    Eigen::Vector2f rside(220, 200);
    Eigen::Vector2f lside(-220, 200);
    if(parts < 1) return false;

    QPointF p,q,r;
    for(float l=0.0; l <= 1.0; l+=1.0/parts)
    {
        p = toQPointF(robot*(1-l) + goal_r*l);
        q = toQPointF((robot+rside)*(1-l) + (goal_r+rside)*l);
        r = toQPointF((robot+lside)*(1-l) + (goal_r+lside)*l);
        if( not pol.containsPoint(p, Qt::OddEvenFill) or
            not pol.containsPoint(q, Qt::OddEvenFill) or
            not pol.containsPoint(r, Qt::OddEvenFill)) {
            reachable = false;
            break;
        }
    }

    // draw
    QLineF line_center(toQPointF(robot), world_to_robotTest(robot2,target));
    QLineF line_right(toQPointF(robot+rside), world_to_robotTest(robot2,target));
    QLineF line_left(toQPointF(robot+lside), world_to_robotTest(robot2,target));
    static QGraphicsItem *graphics_line_center = nullptr;
    static QGraphicsItem *graphics_line_right = nullptr;
    static QGraphicsItem *graphics_line_left = nullptr;
    static QGraphicsItem *graphics_target = nullptr;
    if (graphics_line_center != nullptr)
        viewer->scene.removeItem(graphics_line_center);
    if (graphics_line_right != nullptr)
        viewer->scene.removeItem(graphics_line_right);
    if (graphics_line_left != nullptr)
        viewer->scene.removeItem(graphics_line_left);
    if (graphics_target != nullptr)
        viewer->scene.removeItem(graphics_target);
    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Blue"), 30));
    graphics_line_right = viewer->scene.addLine(line_right, QPen(QColor("Orange"), 30));
    graphics_line_left = viewer->scene.addLine(line_left, QPen(QColor("Magenta"), 30));
    graphics_target = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Blue")), QBrush(QColor("Blue")));
    graphics_target->setPos(target.pos.x(), target.pos.y());

    return reachable;
}

void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo()<<t;
    target.pos = t;
    target.active = true;

    QPointF origin;
    origin.setX(robot_polygon->x());
    origin.setY(robot_polygon->y());
    line.A=origin.y()-target.pos.y();
    line.B=target.pos.x()-origin.x();
    line.C=(origin.x()-target.pos.x())*origin.y()+(target.pos.y()-origin.y())*origin.x();
    movDodge = 0;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{ // robot coordinates
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

QPointF SpecificWorker::world_to_robotTest(Eigen::Vector2f RW, SpecificWorker::Target target)
{
    float alfa = atan2(RW.x(),RW.y());
    Eigen::Vector2f TW(target.pos.x(),target.pos.y()); //target

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

