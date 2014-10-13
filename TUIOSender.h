/*
 TUIO Demo - part of the reacTIVision project
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

#ifndef INCLUDED_TUIOSender_H
#define INCLUDED_TUIOSender_H

#include "tuio/TuioServer.h"
#include "TuioCursor.h"
#include <list>
#include <deque>
#include <set>
#include <math.h>
#include <opencv2/opencv.hpp>
 
using namespace TUIO;

class TUIOSender { 
	
public:
	TUIOSender();
	TUIOSender(const char *host, int port); 
	~TUIOSender() {
		delete m_tuioServer;
	};

	void drawEllipses(cv::Mat& img, int dimX, int dimY) const;
	
	void initFrame();
	void addTuioCursor(float const& x, float const& y);
	void cleanCursors();
	void updateCursors();
	void sendCursors();
	
	TuioTime m_currentTime;
	TuioServer *m_tuioServer;

	std::deque<std::pair<float, float>> m_newTuioPoints;
	std::vector<TuioCursor> m_temp;

	std::set<TuioCursor*> m_TUIOCursorMap;

private:
	const float COLLAPSE_THRESHOLD = .2; //0.2
	const float COLLAPSE_THRESHOLD_EXISTING = .2; //.2
	const int STICKY_FRAMES        = 2; //2
	const int FRAME_THRESHOLD      = 6; //6
};

#endif /* INCLUDED_TUIOSender_H */
