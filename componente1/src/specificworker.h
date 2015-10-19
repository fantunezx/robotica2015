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
#include <AprilTags.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);

public slots:
	void compute(); 	
	

private:
	
	void search();
	void orientation();
	void navegar();
	void vagabundear();
	
	struct TagsList
	{
	
		typedef struct
		{
			int id;
			float tx;
			float ty;
			float tz;
			float rx;
			float ry;
			float rz;
			QTime timem;
		} Tag;
		
		QMap<int, Tag> map;
		
	    QMutex mutex;
		void add( const RoboCompAprilTags::tag t)
		{
			QMutexLocker ml(&mutex);
			Tag tag;
			tag.id = t.id;
			
			tag.tx=t.tx;
			tag.ty=t.ty;
			tag.tz=t.tz;
			tag.rx=t.rx;
			tag.ry=t.ry;
			tag.rz=t.rz;
			tag.timem=QTime::currentTime ();
			map.insert(t.id, tag);
			
		}
		bool exists(int id)
		{
			QMutexLocker ml(&mutex);
			limpiar();
			return map.contains(id);
		}
		
		float distanceMark(int mark){
			QMutexLocker ml(&mutex);
			limpiar();
			if(map.contains(mark)){
				Tag tag = map.value(mark);
				return sqrt(pow(tag.tx,2)+pow(tag.tz,2));
			}
			return -1;
		}
		Tag get(int mark){
			QMutexLocker ml(&mutex);
			limpiar();
			return map.value(mark);
		}
		void limpiar(){
			for(int i=0;i<4;i++){
				if(map.contains(i)){
					QTime timeb=QTime::currentTime();
					if((timeb.msec()-map.value(i).timem.msec())>150){
						map.remove(i);
						std::cout << "marca "<< i<<" desactualizada." << std::endl;
					}
				}
			}
		}
	};
	
	TagsList tList; 
	int marca;
	
	enum class State {INIT, SEARCH, ORIENTATION, ADVANCE, FINISH};
	State state = State::INIT;
	RoboCompLaser::TLaserData ldata;
	
};

#endif

