/*
	TUIO C++ Server Demo - part of the reacTIVision project
	http://reactivision.sourceforge.net/

	Copyright (c) 2005-2009 Martin Kaltenbrunner <mkalten@iua.upf.edu>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#include "TUIOSender.h"

TUIOSender::TUIOSender() {
	m_tuioServer = new TuioServer();
}

TUIOSender::TUIOSender(const char* host, int port) {
	m_tuioServer = new TuioServer(host, port);
}


void TUIOSender::addTuioCursor(float const& x, float const& y)
{
	m_newTuioPoints.push_back(std::make_pair(x,y));
}

void TUIOSender::cleanCursors()
{
	for (auto i = m_TUIOCursorMap.begin(); i != m_TUIOCursorMap.end(); ++i) {
		i->second->incrementMissCounter();
		if (i->second->getMissCounter() >= 10)
		{
			std::cout << "Remove point with ID " << i->second->getCursorID() << std::endl;
			m_tuioServer->removeTuioCursor(i->second);
			m_TUIOCursorMap.erase(i);
			//std::cout << "Num current points: " << m_TUIOCursorMap.size() << std::endl;
		}
	}
}



void TUIOSender::updateCursors()
{
	//std::cout << "Num new points: " << m_newTuioPoints.size() << std::endl;

	for (auto & i : m_newTuioPoints)
	{
		//treshhold to filter bad points out 
		if (i.first < 0.1 || i.second < 0.1)
		{
			break;
		}

		//std::cout << "position: [" << i.first << ", " << i.second << "]" << std::endl;
		bool update = false;

		for (auto & cursor : m_TUIOCursorMap)
		{
			if (cursor.second->getDistance(i.first, i.second) < 0.1)
			{
				std::cout << "Update point with ID: " << cursor.second->getCursorID() << std::endl;
				m_tuioServer->updateTuioCursor(cursor.second, i.first, i.second);
				update = true;
				cursor.second->resetMissCounter();
				break;
			}
		}

		if (!update) {
			int id = 0;
			if (0 < m_TUIOCursorMap.size()) {
				id = m_TUIOCursorMap.rbegin()->first + 1;
			}
			m_TUIOCursorMap[id] = m_tuioServer->addTuioCursor(i.first, i.second);
			std::cout << "Add point with ID: " << m_TUIOCursorMap[id]->getCursorID() << std::endl;
		}
		//std::cout << "Num current points: " << m_TUIOCursorMap.size() << std::endl;
	}
	m_newTuioPoints.erase(m_newTuioPoints.begin(), m_newTuioPoints.end());
}

void TUIOSender::sendCursors() {
	m_tuioServer->stopUntouchedMovingCursors();
	m_tuioServer->commitFrame();
}


/*
void TUIOSender::run() {
	running=true;
	while (running) {
		currentTime = TuioTime::getSessionTime();
		tuioServer->initFrame(currentTime);
		processEvents();
		tuioServer->stopUntouchedMovingCursors();
		tuioServer->commitFrame();
		drawFrame();
		SDL_Delay(20);
	} 
}

int main(int argc, char* argv[])
{
	if (( argc != 1) && ( argc != 3)) {
        	std::cout << "usage: TUIOSender [host] [port]\n";
        	return 0;
	}

#ifndef __MACOSX__
	glutInit(&argc,argv);
#endif
	
	TUIOSender *app;
	if( argc == 3 ) {
		app = new TUIOSender(argv[1],atoi(argv[2]));
	} else app = new TUIOSender("default",0);
	
	app->run();
	delete(app);

	return 0;
}
*/