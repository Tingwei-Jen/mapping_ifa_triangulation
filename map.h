#pragma once
#ifndef MAP_H
#define MAP_H
#include "frame.h"
#include "sign.h"

class Map
{
public:
	Map();
	void AddFrame(const Frame& frame);
	void AddSign(const Sign& sign);
	std::vector<Frame> GetAllFrames(){ return mFrames; }
	std::vector<Sign> GetAllSigns(){ return mSigns; }

private:
	std::vector<Frame> mFrames;
	std::vector<Sign> mSigns;

};

#endif //MAP_H