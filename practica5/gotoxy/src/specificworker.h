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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>
#include <grid2d/grid.h>
#include <dynamic_window.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    void draw_laser(const RoboCompLaser::TLaserData &ldata);

public slots:
    void compute();
    int startup_check();
    void initialize(int period);
    void new_target_slot(QPointF);

private:
    Grid grid;
    Dynamic_Window dw;
    const int TILE_SIZE = 100;
    const float MAX_ADV_VEL = 1000;
    const float MAX_LASER_DIST = 4000;
    std::shared_ptr < InnerModel > innerModel;
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;
    float deltaRot1, deltaRot2, deltaTrans;
    struct Target
    {
        QPointF pos;
        bool active;
    };
    Target target;
    Target target_front;
    struct Line
    {
        float A;
        float B;
        float C;
    };
    Line line;

    struct Door
    {
        QPointF A;
        QPointF B;
        bool visited;
        Eigen::Vector2f midpoint;

    };
    vector <Door> doors;
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    QPointF world_to_robot(RoboCompGenericBase::TBaseState state, Target target);
    enum class State {IDLE, GOTO, INIT_EXPLORE, EXPLORE};
    State currentS = State::IDLE;

    bool obstacle_ahead(const RoboCompLaser::TLaserData &ldata, int dist, int semiwidth=10);

    bool target_visible(const RoboCompLaser::TLaserData &ldata, QPointF tar);

    bool lateral_distance(const RoboCompLaser::TLaserData &ldata, int dist, bool left);

    void update_map(const RoboCompLaser::TLaserData &ldata);

    QPointF robot_to_world(RoboCompGenericBase::TBaseState state, Eigen::Vector2f TW);
    QPointF robot_to_world2(Eigen::Vector2f TW);

    float reduce_speed_if_close_to_target(float mod);

    void explore(const RoboCompLaser::TLaserData &ldata,double initial_angle);

    bool checkTiles();

    void isDoor(const RoboCompLaser::TLaserData &ldata);

    void checkDoors (vector <QPointF> peaks);

    bool checkCoordinates (QPointF p1, QPointF p2);

    void drawDoors();

    void gotoDoor(const RoboCompLaser::TLaserData &ldata);

    Eigen::Vector2f newMidPoint (Door d);

    float distance(Eigen::Vector2f A);

    void gotoPoint(const RoboCompLaser::TLaserData &ldata);
    void gotoFront(const RoboCompLaser::TLaserData &ldata);


        Eigen::Vector2f  world_to_robot2(Eigen::Vector2f point);
};

#endif
