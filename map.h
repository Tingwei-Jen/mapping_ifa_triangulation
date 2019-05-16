#pragma once
#ifndef MAP_H
#define MAP_H
#include "sign.h"

class Map
{
public:
	Map();
	void AddSign(Sign* sign);
	std::vector<Sign*> GetAllSigns(){ return mvpSigns; }

private:
	std::vector<Sign*> mvpSigns;

};

#endif //MAP_H