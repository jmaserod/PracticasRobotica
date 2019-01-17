/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
	void obtenerMarcas();
    int obtenerTag(int t);


public slots:
	void compute();

private:
	InnerModel *innerModel;
    int current = 0;
    enum class State{INIT, SEARCH, WAIT, FINISH};
    State state = State::SEARCH;

    float RobotInitX, RobotInitZ;



	struct Tag{

    		QMutex mutex;
    		float tagx,tagz;
    		int id;

    		bool vacio = true;

    		Tag(){};

    		bool insertarCoordenadas(int idT, float x, float z){

    			QMutexLocker lm(&mutex);
    			tagx = x;
    			tagz = z;
    			id = idT;
    			vacio = false;
    			return true;
    		}

    		std::pair<float, float> extraerCoordenadas(){

    			std::pair<float, float> Ctag;
    			QMutexLocker lm(&mutex);
    			Ctag.first = tagx;
    			Ctag.second = tagz;

    			return Ctag;
    		}

    		int obtenerIdTag(){

    			QMutexLocker lm(&mutex);
    			int tagId = id;

    			return tagId;
    		}

    		bool esVacio(){

    			QMutexLocker lm(&mutex);
    			return vacio;
    		}

    		void vaciarTag(){

    			QMutexLocker lm(&mutex);
    			vacio = true;
    		}

    		};

    		Tag T;
    		RoboCompGetAprilTags::listaMarcas listaMarcas;



    };

    #endif


