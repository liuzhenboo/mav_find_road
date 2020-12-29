#include <iostream>
#include <Cell.h>

Cell::Cell()
{
}
Cell::Cell(uint8_t class_state, float current_height)
{
	state = 3;
	seen_times = 0;
	obs_seentimes = 0;
	ground_seentimes = 0;
	obs_height = 0;
	ground_height = 0;
	Update(class_state, current_height);
}
Cell::~Cell()
{
}

uint8_t Cell::Update(uint8_t class_state, float current_height)
{
	uint8_t fusion_id = class_state;
	// 更新格子状态
	if (class_state == 1)
	{
		ground_height = (current_height + ground_height * ground_seentimes) / (1.0 * (++ground_seentimes));
	}
	else if (class_state == 2)
	{
		obs_height = (current_height + obs_height * obs_seentimes) / (1.0 * (++obs_seentimes));
	}
	if (obs_seentimes >= 5)
	{
		fusion_id = 2;
	}
	else if (ground_seentimes >= 1)
	{
		fusion_id = 1;
	}
	else
	{
		fusion_id = 3;
	}
	state = fusion_id;
	return fusion_id;
}
float Cell::GetZ(int id)
{
	if (state == 1)
	{
		return ground_height;
	}
	else if (state == 2)
	{
		return obs_height;
	}

	return 0;
}
