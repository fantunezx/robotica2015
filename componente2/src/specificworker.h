/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	
	float go(const TargetPose &target);
	NavState getState();
	void stop();

public slots:
	void compute(); 	

private:
	
	void goal();
	void freeway();
	void newway();
	void advance();
	bool calcularCamino(QVec mem);
	typedef struct{
		bool sub;
		bool newtarget;
		TargetPose target;
		TargetPose subtarget;
	}Coordenadas;
	
	enum class State {IDLE, INIT, GOAL, FREEWAY, NEWWAY, ADVANCE, FINISH};
	State state = State::IDLE;
	RoboCompLaser::TLaserData ldata;
	NavState estado;
	Coordenadas coor;
	InnerModel *inner;
	float modulo;
	QVec mem;	
	TargetPose t;
	bool encontrado = false;
	bool encontradode = false;
	bool encontradoiz = false;
	int contador;
	int punto;
	float angulo, distancia;
	QGraphicsScene scene;
	void histogram();
};

#endif

