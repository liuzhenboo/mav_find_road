#include "ring_cell.h"

ring_cell::ring_cell()
{
}
ring_cell::ring_cell(unsigned int index_x, unsigned int index_y, string status)
{
	index_x_ = index_x;
	index_y_ = index_y;
	status_ = status;
}

ring_cell::~ring_cell()
{
}