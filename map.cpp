#include "map.h"

Map::Map()
{
}

void Map::AddSign(Sign* sign)
{
	this->mvpSigns.push_back(sign);
}
