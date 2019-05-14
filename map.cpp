#include "map.h"

Map::Map()
{
}

void Map::AddFrame(const Frame& frame)
{
	this->mFrames.push_back(frame);
}

void Map::AddSign(const Sign& sign)
{
	this->mSigns.push_back(sign);
}
