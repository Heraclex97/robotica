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
#include <cppitertools/range.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
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
    QRectF dimensions(-5100,-2600,10200,5200);
    viewer = new AbstractGraphicViewer(this, dimensions);
    grid.initialize(dimensions,TILE_SIZE,&viewer->scene,false);
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

	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(50);
}

void SpecificWorker::compute()
{
    double initial_angle;
    RoboCompLaser::TLaserData ldata;
    float adv = 200;
    float beta = 2;
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
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }

    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    update_map(ldata);
    switch (currentS)
    {
        case State::IDLE:
            currentS = State::INIT_EXPLORE;
            break;

        case State::GOTO:
            gotoPoint(ldata);
            break;

        case State::SHOCK:
            differentialrobot_proxy->setSpeedBase(5, 0.6);
            currentS = State::EXPLORE;
            break;
        case State::INIT_EXPLORE:
            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            currentS = State::EXPLORE;
            break;

        case State::EXPLORE:
            differentialrobot_proxy->setSpeedBase(0, 0.6);
            explore(ldata,initial_angle);
            break;
    }
}

//////////////////////////////////////////////
void SpecificWorker::explore(const RoboCompLaser::TLaserData &ldata, double initial_angle)
{
    float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
    isDoor(ldata);
    if (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1))
    {
        differentialrobot_proxy->setSpeedBase(0, 0);
        gotoDoor(ldata);
        currentS = State::GOTO;
    }
}
void SpecificWorker::gotoPoint(const RoboCompLaser::TLaserData &ldata){
    Eigen::Vector2f tar(target.pos.x(),target.pos.y());
    auto tr = world_to_robot2(tar);
    float dist = tr.norm();
    QPointF a;
    if(dist < 150)  // at target
    {
        qInfo() << __FUNCTION__ << "    Robot reached target room" << r_state.x << r_state.y ;
        differentialrobot_proxy->setSpeedBase(0, 0);
        target.active = false;
        currentS = State::INIT_EXPLORE;
    }
    else  // continue to room
    {

        qInfo() << __FUNCTION__ << "    Robot reached target room2222" ;
        // call dynamic window
        QPolygonF laser_poly;
        for(auto &&l : ldata)
            laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
        auto [_, __, adv, rot, ___] = dw.compute(tr, laser_poly,
                                                 Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                 Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                 nullptr /*&viewer_robot->scene*/);
        const float rgain = 0.8;
        float rotation = rgain*rot;
        float dist_break = std::clamp(dist/1000, 0.0f, 1.0f);
        float advance = MAX_ADV_VEL * dist_break * exp(-pow(rotation,2)/0.1);
        differentialrobot_proxy->setSpeedBase(advance, rotation);
    }

}
void SpecificWorker::gotoDoor(const RoboCompLaser::TLaserData &ldata)
{
    Door nearDoor = doors.front();
    for (Door d: doors) {
        if (!d.visited && (distance(d.midpoint) < distance(nearDoor.midpoint)))
                nearDoor = d;


    }
    nearDoor.visited = true;

    //Saltar a otro estado que atreviese a la puerta más cercana

    Eigen::Vector2f p1 = Eigen::Vector2f (nearDoor.A.x(),nearDoor.A.y());
    Eigen::Vector2f p2 = Eigen::Vector2f (nearDoor.B.x(),nearDoor.B.y());

    Eigen::ParametrizedLine<float, 2> r = Eigen::ParametrizedLine<float, 2>(nearDoor.midpoint,(p1 - p2).unitOrthogonal());
    qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
    Eigen::Vector2f external_midpoint = r.pointAt(1000.0);

    target.pos = QPointF(external_midpoint.x(),external_midpoint.y());
    target.active = true;
    qInfo() << __FUNCTION__ << "  TARGET" << target.pos.x() << target.pos.y();

    //gotoPoint(ldata);

}

Eigen::Vector2f SpecificWorker::newMidPoint (Door d){
    Eigen::Vector2f p1 = Eigen::Vector2f (d.A.x(),d.A.y());
    Eigen::Vector2f p2 = Eigen::Vector2f (d.B.x(),d.B.y());

    return p1 + ((p2 - p1) / 2.0);
}

float SpecificWorker::distance (Eigen::Vector2f A) {
    QPointF pA = QPointF(A.x(),A.y());
    QPointF pRstate = QPointF(r_state.x,r_state.y);
    QLineF line = QLineF (pA,pRstate);
    return line.length();
}

void SpecificWorker::isDoor(const RoboCompLaser::TLaserData &ldata) {
    QPointF a, b;

    vector <QPointF> peaks;

    for (long unsigned int i = 1; i < ldata.size()-1; i++){
        a = QPointF(ldata[i].dist*sin(ldata[i].angle),ldata[i].dist*cos(ldata[i].angle));
        b = QPointF(ldata[i+1].dist*sin(ldata[i+1].angle),ldata[i+1].dist*cos(ldata[i+1].angle));
        QLineF dist(a,b);
        if (dist.length()>800){
           if(ldata[i].dist<ldata[i+1].dist)
               peaks.push_back(a);
           else
               peaks.push_back(b);
        }
    }

    checkDoors(peaks);

    drawDoors();
}

void SpecificWorker::drawDoors ()
{
    static std::vector<QGraphicsItem*> door_points;
    for(auto dp : door_points) viewer->scene.removeItem(dp);
    door_points.clear();
    for(const auto p: doors)
    {
        QPointF d = p.A;
        QPointF d2 = p.B;

        door_points.push_back(viewer->scene.addLine(QLineF(d,d2),QPen(QColor("Magenta"),100)));
        door_points.back()->setZValue(200);
    }
}

void SpecificWorker::checkDoors (vector <QPointF> peaks) {
    for (auto &&c: iter::combinations_with_replacement(peaks,2)) { // CHEK IF DISTANCE BETWEEN POINTS IS BETWEEN 1100 AND 900;

        QPointF x = robot_to_world2(Eigen::Vector2f (c[0].x(),c[0].y()));
        QPointF y = robot_to_world2(Eigen::Vector2f (c[1].x(),c[1].y()));

        bool iD = false;

        QLineF check(x,y);
        if (check.length()>=400 && check.length()<=1300) {
            for (auto p: doors) {
                if(!checkCoordinates(p.A, x) && !checkCoordinates(p.B, y))
                    iD = false;

                else {
                    iD = true;
                    break;
                }
            }

            if (!iD) {
                Door d;
                d.A = x;
                d.B = y;
                d.visited = false;
                d.midpoint = newMidPoint(d);
                doors.push_back(d);
            }
        }
    }
}

bool SpecificWorker::checkCoordinates (QPointF p1, QPointF p2) {
    int threshold = 1000;
    if (abs(p1.x()-p2.x()) < threshold && abs(p1.y()-p2.y()) < threshold)
        return true;
    else
        return false;
}

bool SpecificWorker::checkTiles ()
{
    int totTiles = grid.count_total();
    int changingTiles;

    if ((changingTiles/totTiles) < 0.5)
        return true;
    else
        return false;
}

void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata)
{
    QPointF lineP;
    Eigen::Vector2f TW;


    for (auto &p:ldata) {
        float step = ceil(p.dist / (TILE_SIZE / 2.0));
        TW = Eigen::Vector2f(p.dist * sin(p.angle), p.dist * cos(p.angle));
        lineP = robot_to_world2(TW);
        float lastX = -1000000;
        float lastY = -1000000;
        float tarX = (lineP.x() - grid.dim.left()) / grid.TILE_SIZE;
        float tarY = (lineP.y() - grid.dim.bottom()) / grid.TILE_SIZE;
        for (const auto &&step: iter::range(0.0, 1.0 - (1.0 / step), 1.0 / step)) {
            lineP = robot_to_world2( TW * step);
            float kx = (lineP.x() - grid.dim.left()) / grid.TILE_SIZE;
            float ky = (lineP.y() - grid.dim.bottom()) / grid.TILE_SIZE;
            if (kx != lastX && kx != tarX && ky != lastY && ky != tarY) {
                lineP = robot_to_world2(TW * step);
                grid.add_miss(Eigen::Vector2f(lineP.x(), lineP.y()));
            }
            lastX = kx;
            lastY = ky;
        }

        if (p.dist <= MAX_LASER_DIST) {
            lineP = robot_to_world2(TW);
            grid.add_hit(Eigen::Vector2f(lineP.x(), lineP.y()));
        }
    }
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

    return QPointF(TR.x(),TR.y());
}

QPointF SpecificWorker::robot_to_world(RoboCompGenericBase::TBaseState state, Eigen::Vector2f TW)
{
    float alfa = state.alpha;
    Eigen::Vector2f RW(state.x,state.z); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = -sin(alfa);
    R(1,0) = sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * TW + RW;

    return QPointF(TR.x(),TR.y());
}

Eigen::Vector2f SpecificWorker::world_to_robot2(Eigen::Vector2f point)
{
//    declarar matriz, con el angulo y la pos libreria de algebra lineal (mult por vector)
    float alfa = r_state.rz;
    Eigen::Vector2f RW(r_state.x,r_state.y); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = sin(alfa);
    R(1,0) = -sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * (point-RW);

    return TR;
}
QPointF SpecificWorker::robot_to_world2(Eigen::Vector2f TW)
{
    float alfa = r_state.rz;
    Eigen::Vector2f RW(r_state.x,r_state.y); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = -sin(alfa);
    R(1,0) = sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * TW + RW;

    return QPointF(TR.x(),TR.y());
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

