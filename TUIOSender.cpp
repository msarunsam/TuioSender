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
#include <sstream>

TUIOSender::TUIOSender() {
	m_tuioServer = new TuioServer();
}

TUIOSender::TUIOSender(const char* host, int port) {
	m_tuioServer = new TuioServer(host, port);
}


void TUIOSender::addTuioCursor(float const& x, float const& y)
{
	m_temp.push_back(TuioCursor(m_tuioServer->getSessionID(),0,x,y));
}

void TUIOSender::drawEllipses(cv::Mat& img) const
{
	for (auto const& i : m_TUIOCursorMap) {
		cv::ellipse(img, cv::Point(i->getX() * 950, i->getY() * 600), cv::Size(10, 10), .0, .0, 360.0, cv::Scalar(0, 0, 255), 5);
		
		std::ostringstream convert;
		convert << i->getCursorID();
		cv::putText(img, convert.str(), cv::Point(i->getX() * 950, i->getY() * 600), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0,255,0), 1);
	}
}

void TUIOSender::initFrame()
{
	TuioTime currentTime = TuioTime::getSessionTime();
	m_tuioServer->initFrame(currentTime);
}


void TUIOSender::updateCursors()
{
	std::vector<TuioCursor> tempCollapsed;

	// collapse all points
	while (!m_temp.empty()) {
		TuioCursor cursor = m_temp.back();
		m_temp.pop_back();

		bool similarFound = false;
		for (auto& i : m_temp) {
			if (cursor.getDistance(i.getX(), i.getY()) < COLLAPSE_THRESHOLD) {
				similarFound = true;
			}
		}
		if (!similarFound) {
			tempCollapsed.push_back(cursor);
		}
	}

	// update existing cursors, remove stall ones
	for (auto j = tempCollapsed.begin(); j != tempCollapsed.end(); ) {
		// build vector with distances
		std::vector<std::pair<std::set<TuioCursor*>::iterator, float>> distances;

		for (auto i = m_TUIOCursorMap.begin(); i != m_TUIOCursorMap.end(); ++i) {
			// check if cursors has stalled
			if ((*i)->getDistance(j->getX(), j->getY()) < COLLAPSE_THRESHOLD_EXISTING) {
				distances.push_back(std::make_pair(i, (*i)->getDistance(j->getX(), j->getY())));
			}
		}

		// sort distances to find nearest neighbor
		if (!distances.empty()) {
			std::sort(distances.begin(), distances.end(),
				[](std::pair<std::set<TuioCursor*>::iterator, float> const& a, std::pair<std::set<TuioCursor*>::iterator, float> const& b) {
					return a.second < b.second;
			});

			auto it = distances[0].first;
			m_tuioServer->updateTuioCursor(*it, j->getX(), j->getY());
			(*it)->resetMissCounter();
			j = tempCollapsed.erase(j);
			continue;
		}

		++j;
	}

	// remove stall cursors
	for (auto i = m_TUIOCursorMap.begin(); i != m_TUIOCursorMap.end(); ) {
		(*i)->incrementMissCounter();
		std::cout << "Increment miss counter for cursor " << (*i)->getCursorID() << " (" << (*i)->getMissCounter() << ")" << std::endl;

		if ((*i)->getMissCounter() > STICKY_FRAMES) {
			std::cout << "Remove cursor " << (*i)->getCursorID() << std::endl;
			m_tuioServer->removeTuioCursor(*i);
			i = m_TUIOCursorMap.erase(i);
			continue;
		} else {
			++i;
		}
	}

	// add new cursors
	for (auto i = tempCollapsed.begin(); i != tempCollapsed.end(); ) {
		i->incrementHitCounter();
		if (i->getHitCounter() > FRAME_THRESHOLD) {
			m_TUIOCursorMap.insert(m_tuioServer->addTuioCursor(i->getX(), i->getY()));
			i = tempCollapsed.erase(i);
		} else {
			++i;
		}
	}

	m_temp = tempCollapsed;

	/*

	bool updated = false;

	for (auto i = m_temp.begin(); i != m_temp.end(); ++i)
	{
		if(i->getDistance(x,y) < COLLAPSE_THRESHOLD)
		{
			i->incrementHitCounter();
			updated=true;

			if(i->getHitCounter() > FRAME_THRESHOLD)
			{
				std::cout << "Adding cursors (counter: " << i->getHitCounter() << ")" << std::endl;
				m_newTuioPoints.push_back(std::make_pair(x,y));
				// TODO: delete point from m_temp
			}
			break;
		}
	}

	if(!updated)
	{
		temp.incrementHitCounter();
		m_temp.push_back(temp);
	}

	//std::cout << "Num new points: " << m_newTuioPoints.size() << std::endl;
	for (auto & i : m_newTuioPoints)
	{

		//std::cout << "[" << i.first << ", " << i.second << "]" << std::end;
		//std::cout << "position: [" << i.first << ", " << i.second << "]" << std::endl;
		bool update = false;

		for (auto & cursor : m_TUIOCursorMap)
		{
			if (cursor->getDistance(i.first, i.second) < COLLAPSE_THRESHOLD * cursor->getHitCounter())
			{
				std::cout << "Update point with ID: " << cursor->getCursorID() << std::endl;
				m_tuioServer->updateTuioCursor(cursor, i.first, i.second);
				update = true;
				cursor->resetMissCounter();
				break;
			}

		}
		if (!update) {
			m_TUIOCursorMap.insert(m_tuioServer->addTuioCursor(i.first, i.second));
			//std::cout << "Add point with ID: " << m_TUIOCursorMap[id]->getCursorID() << std::endl;
		}
		//std::cout << "Num current points: " << m_TUIOCursorMap.size() << std::endl;
	}

	std::vector<std::list<TuioCursor>::iterator> deleteCandidates;

	for (auto i  = m_temp.begin(); i != m_temp.end(); ++i) {
		for (auto const& j : m_TUIOCursorMap) {
			if (j->getDistance(i->getX(), i->getY()) > COLLAPSE_THRESHOLD) {
				m_tuioServer->updateTuioCursor(j, i->getX(), i->getY());
				deleteCandidates.push_back(i);
			}
		}
	}

	for (auto i : deleteCandidates) {
		std::cout << "m_temp.size(): " << m_temp.size() << std::endl;
		//m_temp.erase(i);
	}

	m_newTuioPoints.clear();*/
}

void TUIOSender::cleanCursors()
{
	/*std::vector<std::vector<TuioCursor>::iterator> deleteCandidates;

	for (auto i = m_temp.begin(); i != m_temp.end(); ++i)
	{
		i->incrementFrameCounter();
		if(i->getFrameCounter() - i->getHitCounter() >= DELETE_THRESHOLD)
		{
			std::cout << i->getFrameCounter() << " " << m_temp.size() << std::endl;
			deleteCandidates.push_back(i);
		} 
	}

	for (auto const& i : deleteCandidates) {
		m_temp.erase(i);
	}


	for (auto i = m_TUIOCursorMap.begin(); i != m_TUIOCursorMap.end(); ++i) 
	{
		(*i)->incrementMissCounter();
		if ((*i)->getMissCounter() >= STICKY_FRAMES)
		{
			//std::cout << "Remove point with ID " << i->second->getCursorID() << std::endl;
			m_tuioServer->removeTuioCursor(*i);
			m_TUIOCursorMap.erase(i);
			//std::cout << "Num current points: " << m_TUIOCursorMap.size() << std::endl;
		}
	}
	//std::cout << "foundPoints: " << m_TUIOCursorMap.size() << std::endl;*/
}


void TUIOSender::sendCursors() {
	m_tuioServer->stopUntouchedMovingCursors();
	//m_tuioServer->sendFullMessages();
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