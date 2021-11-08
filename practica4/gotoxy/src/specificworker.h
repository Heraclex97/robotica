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
    const float MAX_ADV_VEL = 1000;
    std::shared_ptr < InnerModel > innerModel;
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;
    QPointF actual_point;
    int movDodge;

    struct Target
    {
        QPointF pos;
        bool active;
    };
    Target target;

    struct Line
    {
        float A;
        float B;
        float C;
    };
    Line line;

    void gotoTarget(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState baseState,QPointF pr, float adv, float beta);
    void doShock(const RoboCompLaser::TLaserData &ldata);
    void doDodge(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState &base, float speed, float rot);
    QPointF world_to_robot(RoboCompGenericBase::TBaseState state, Target target);
    enum class State {IDLE, GOTO, SHOCK, DODGE};
    State currentS = State::IDLE;
    float reduce_speed_if_close_to_target(float mod);
    bool obstacle_ahead(const RoboCompLaser::TLaserData &ldata, int dist, int semiwidth=10);
    bool target_visible(const RoboCompLaser::TLaserData &ldata, QPointF tar);
    bool lateral_distance(const RoboCompLaser::TLaserData &ldata, int dist, bool left);
    bool check_free_path_to_target(const RoboCompLaser::TLaserData &ldata);
    QPointF world_to_robotTest(Eigen::Vector2f RW, Target target);
    bool line_dist(RoboCompGenericBase::TBaseState &base, int threshold);

};

#endif
