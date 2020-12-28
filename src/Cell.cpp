#include <iostream>
#include <Cell.h>

Cell::Cell()
{
}
Cell::Cell(uint8_t class_state, float current_height)
{
	state = 1;
	height = current_height;
	seen_times = 1;
}
Cell::~Cell()
{
}

void Cell::Update(uint8_t class_state, float current_height)
{
	state = 1;
	height = (current_height + height * seen_times) / (1.0 * (++seen_times));
}
float Cell::GetZ(int id)
{
	return height;
}
