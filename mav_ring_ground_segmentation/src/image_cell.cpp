#include "image_cell.h"

cell::cell()
{
	lable_ = "EDGE";
	ave_depth_ = 0.0;
	status_ = "no_occupied";
	ave_x_ = 0;
	ave_y_ = 0;
	ave_z_ = 0;
}
cell::cell(unsigned int x_index, unsigned int y_index)
{
	lable_ = "EDGE";
	ave_depth_ = 0.0;
	status_ = "no_occupied";
	x_index_ = x_index;
	y_index_ = y_index;
}

cell::~cell()
{
}