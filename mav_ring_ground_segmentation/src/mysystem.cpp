
#include "mysystem.h"
void mysystem::resize_cellmap(unsigned int col, unsigned int row, unsigned int col_percell, unsigned int row_percell)
{
	// 初始化ringmap和imagemap
	camera_factor_ = 1000;
	camera_cx_ = 321.798;
	camera_cy_ = 239.607;
	camera_fx_ = 615.899;
	camera_fy_ = 616.468;
	camera_f_ = 615.899;

	ring_r_ = 0.5;
	ring_R_ = 6.0;
	ring_theta_ = 2.0;
	ring_row_ = 10;
	ring_col_ = 10;
	min_obs_ = 0.2;
	max_between_cell_high_ = 0.2;
	edge_dilate_size_ = 3;
	kenel_size_ = 5;
	voxfile_ = 0.1f;

	cv::FileStorage fsSettings("../src/config.yaml", cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong config path at : " << endl;
		exit(-1);
	}
	camera_factor_ = fsSettings["camera_factor"];
	camera_cx_ = fsSettings["camera_cx"];
	camera_cy_ = fsSettings["camera_cy"];
	camera_fx_ = fsSettings["camera_fx"];
	camera_fy_ = fsSettings["camera_fy"];
	camera_f_ = fsSettings["camera_f"];

	// std::cout << camera_factor_ << std::endl;
	ring_r_ = fsSettings["ring_r"];
	ring_R_ = fsSettings["ring_R"];
	ring_theta_ = fsSettings["ring_theta"];
	ring_row_ = fsSettings["ring_row"];
	ring_col_ = fsSettings["ring_col"];
	min_obs_ = fsSettings["min_obs"];
	max_between_cell_high_ = fsSettings["max_between_cell_high"];
	edge_dilate_size_ = fsSettings["edge_dilate_size"];
	kenel_size_ = fsSettings["kenel_size"];
	voxfile_ = fsSettings["voxfile"];

	col_ = col;
	row_ = row;
	col_percell_ = col_percell;
	row_percell_ = row_percell;
	col_cellsum_ = col / col_percell;
	row_cellsum_ = row / row_percell;
	pix_sum_percell_ = col_percell * row_percell_;
	cellmap_.resize(col_cellsum_);

	// 实例化
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		cellmap_[i].resize(row_cellsum_);
	}

	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			cell temp(i, j);
			cellmap_[i][j] = temp;
		}
	}
	// 实例化
	true_cellmap_.resize(col_cellsum_);
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		true_cellmap_[i].resize(row_cellsum_);
	}
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			cell temp(i, j);
			true_cellmap_[i][j] = temp;
		}
	}
	ring_cellmap_.resize(ring_col_);
	for (unsigned int i = 0; i < ring_col_; i++)
	{
		ring_cellmap_[i].resize(ring_row_);
	}

	// 实例化
	for (unsigned int i = 0; i < ring_col_; i++)
	{
		for (unsigned int j = 0; j < ring_row_; j++)
		{
			ring_cell *ring_cell_temp = new ring_cell(i, j, "just_init");
			ring_cellmap_[i][j] = ring_cell_temp;
		}
	}
}
void mysystem::getdata(cv::Mat idepth, cv::Mat itrue)
{
	depth_img_ = idepth;
	rgb_img_ = itrue;
}

void mysystem::update_true_cell(unsigned int i_index, unsigned int j_index)
{
	true_cellmap_[i_index][j_index].x_index_ = i_index;
	true_cellmap_[i_index][j_index].y_index_ = j_index;
	unsigned int start_u = i_index * col_percell_;
	unsigned int start_v = j_index * row_percell_;
	unsigned int ground_point = 0, obs_point = 0, unsure = 0;
	for (unsigned int u = 0; u < col_percell_; u++)
	{
		for (unsigned int v = 0; v < row_percell_; v++)
		{
			unsigned int b = true_img_.at<Vec3b>(start_v + v, start_u + u)[0];
			unsigned int g = true_img_.at<Vec3b>(start_v + v, start_u + u)[1];
			unsigned int r = true_img_.at<Vec3b>(start_v + v, start_u + u)[2];

			// 真实分割只有两种颜色128的红和黑色.
			if (r == 128 && g == 0 && b == 0)
			{
				ground_point++;
			}
			else
			{
				obs_point++;
			}
		}
	}
	std::string temp = to_string(i_index) + ",";
	temp = temp + to_string(j_index);

	if (ground_point == pix_sum_percell_)
	{
		true_cellmap_[i_index][j_index].lable_ = "GROUND";
	}
	else if (obs_point == pix_sum_percell_)
	{
		true_cellmap_[i_index][j_index].lable_ = "OBS";
	}
	else
	{
		true_cellmap_[i_index][j_index].lable_ = "UNSURE";
	}
}
void mysystem::debug_mypoint(string name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imgcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			for (auto it : cellmap_[i][j].image_ponts_)
			{
				pcl::PointXYZRGB p;
				// 计算这个点的空间坐标
				p.z = it->z_;
				p.x = it->x_;
				p.y = it->y_;

				if (it->is_ground_ == 11)
				{
					p.b = 0;
					p.g = 255;
					p.r = 0;
				}
				else if (it->is_ground_ == 22)
				{
					p.b = 0;
					p.g = 0;
					p.r = 255;
				}
				else
				{
					p.b = 255;
					p.g = 0;
					p.r = 0;
				}
				imgcloud->points.push_back(p);
			}
		}
	}
	imgcloud->height = 1;
	imgcloud->width = imgcloud->points.size();
	std::cout << "mypointcloud" << imgcloud->width << std::endl;
	pcl::io::savePCDFileASCII(name, *imgcloud);
}
void mysystem::debug_mypoint_true_labelme()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imgcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			for (auto it : cellmap_[i][j].image_ponts_)
			{
				pcl::PointXYZRGB p;
				// 计算这个点的空间坐标
				p.z = it->z_;
				p.x = it->x_;
				p.y = it->y_;

				if (true_cellmap_[i][j].status_ == "ground")
				{
					p.b = 0;
					p.g = 255;
					p.r = 0;
				}
				else if (true_cellmap_[i][j].status_ == "obs")
				{
					p.b = 0;
					p.g = 0;
					p.r = 255;
				}
				else
				{
					p.b = 255;
					p.g = 0;
					p.r = 0;
				}
				imgcloud->points.push_back(p);
			}
		}
	}
	imgcloud->height = 1;
	imgcloud->width = imgcloud->points.size();
	std::cout << "mypointcloud" << imgcloud->width << std::endl;
	pcl::io::savePCDFileASCII("test_mypoint_true_lable_pcd.pcd", *imgcloud);
}

void mysystem::get_pcl_pointcloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// 遍历深度图
	for (int m = 0; m < depth_img_.rows; m++)
	{
		for (int n = 0; n < depth_img_.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			//float d = depth_img_.ptr<float>(m)[n];
			float d = depth_img_.ptr<ushort>(m)[n];

			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			pcl::PointXYZRGB p;

			// 计算这个点的空间坐标
			p.z = double(d) / camera_factor_;
			p.x = (n - camera_cx_) * p.z / camera_fx_;
			p.y = (m - camera_cy_) * p.z / camera_fy_;
			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			p.b = rgb_img_.ptr<uchar>(m)[n * 3];
			p.g = rgb_img_.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb_img_.ptr<uchar>(m)[n * 3 + 2];
			// 把p加入到点云中
			if (p.z > ring_R_)
			{
				continue;
			}
			cloud->points.push_back(p);
		}
	}

	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	std::cout << "原始点云大小：" << cloud->width << std::endl;
	cloud->is_dense = false; // 转换点云的数据类型并存储成pcd文件

	pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;			 //直通滤波对象定义
	voxelGrid.setInputCloud(cloud);						 //设置输入点云
	voxelGrid.setLeafSize(voxfile_, voxfile_, voxfile_); //滤波网格大小，实际空间尺寸，和点间距有关，单位是m
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	voxelGrid.filter(*voxelCloud); //执行滤波
								   // pcl::io::savePCDFileASCII("test_pcd.pcd", *voxelCloud);
	std::cout << "体素滤波后原始点云大小：" << voxelCloud->width << std::endl;

	for (int i = 0; i < voxelCloud->points.size(); i++)
	{
		pcl::PointXYZRGB p1 = voxelCloud->points[i];
		//std::cout << 11 << std::endl;
		// 找到p1对应的图像地图位置，将两者相互关联起来
		int u = p1.x * camera_fx_ / p1.z + camera_cx_;
		int v = p1.y * camera_fy_ / p1.z + camera_cy_;
		int col_id = u / col_percell_;
		int row_id = v / row_percell_;

		// std::cout << 22 << std::endl;

		// 找到p1对应的环形地图位置，将两者相互关联起来
		int ring_col_id = ring_col_ * (0.5 * ring_theta_ - atan2(p1.x, p1.z)) / ring_theta_;
		float ring_length = std::sqrt(p1.x * p1.x + p1.z * p1.z);
		int ring_row_id = ring_row_ * (ring_length - ring_r_) / (ring_R_ - ring_r_);

		// std::cout << 33 << std::endl;

		if (col_id >= 0 && col_id < col_cellsum_ && row_id >= 0 && row_id < row_cellsum_ && ring_col_id >= 0 && ring_col_id < ring_col_ && ring_row_id >= 0 && ring_row_id < ring_row_)
		{
			mypoint *temp_mp = new mypoint();
			temp_mp->x_ = p1.x;
			temp_mp->y_ = p1.y;
			temp_mp->z_ = p1.z;
			temp_mp->r_ = p1.r;
			temp_mp->g_ = p1.g;
			temp_mp->b_ = p1.b;
			temp_mp->cell_ = &cellmap_[col_id][row_id];
			cellmap_[col_id][row_id].image_ponts_.push_back(temp_mp);
			temp_mp->ring_cell_ = ring_cellmap_[ring_col_id][ring_row_id];
			// 为了方便，将其排序乘以-1
			ring_cellmap_[ring_col_id][ring_row_id]->ring_ponts_.insert(pair<float, mypoint *>(-1.0 * p1.y, temp_mp));
			ponts_db_.insert(pair<float, mypoint *>(-1.0 * p1.y, temp_mp));
		}
		else
		{
			continue;
		}
	}

	// test debug ringmap ring_cellmap_.size()
#ifdef debug_point_to_ringmap_project
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ringcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int i = 0; i < ring_cellmap_.size(); i++)
	{
		for (unsigned int j = 0; j < ring_cellmap_[i].size(); j++)
		{
			ring_cell *ring = ring_cellmap_[i][j];
			for (auto it : ring->ring_ponts_)
			{
				pcl::PointXYZRGB p;
				// 计算这个点的空间坐标
				p.z = it.second->z_;
				p.x = it.second->x_;
				p.y = it.second->y_;
				p.b = it.second->b_;
				p.g = it.second->g_;
				p.r = it.second->r_;
				ringcloud->points.push_back(p);
			}
		}
	}
	ringcloud->height = 1;
	ringcloud->width = ringcloud->points.size();
	std::cout << "ringcloud点云大小：" << ringcloud->width << std::endl;
	pcl::io::savePCDFileASCII("test_ring_map_pcd.pcd", *ringcloud);
#endif // DEBUG

// debug imagemap
#ifdef debug_point_to_imgmap_project
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imgcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			for (auto it : cellmap_[i][j].image_ponts_)
			{
				pcl::PointXYZRGB p;
				// 计算这个点的空间坐标
				p.z = it->z_;
				p.x = it->x_;
				p.y = it->y_;
				p.b = it->b_;
				p.g = it->g_;
				p.r = it->r_;
				imgcloud->points.push_back(p);
			}
		}
	}
	imgcloud->height = 1;
	imgcloud->width = imgcloud->points.size();
	std::cout << "imgcloud" << imgcloud->width << std::endl;
	pcl::io::savePCDFileASCII("test_img_map_pcd.pcd", *imgcloud);
#endif // DEBUG
}
void mysystem::segment_based_height()
{
	// segment_based_height
	for (unsigned int i = 0; i < ring_cellmap_.size(); i++)
	{
		for (unsigned int j = 0; j < ring_cellmap_[i].size(); j++)
		{
			//std::multimap<float, mypoint *> ring_ponts_
			if (ring_cellmap_[i][j]->ring_ponts_.size() != 0)
			{
				auto iter = ring_cellmap_[i][j]->ring_ponts_.begin();
				iter->second->is_ground_ = 11;
				float min_y = iter->first;
				for (auto it : ring_cellmap_[i][j]->ring_ponts_)
				{
					// 判断障碍物，没有考虑地势
					if (-1.0 * it.second->y_ > (min_y + min_obs_))
						it.second->is_ground_ = 22;
					else
						it.second->is_ground_ = 11;
				}
			}
		}
	}
}
void mysystem::update_mypoints_status(ring_cell *last, ring_cell *current)
{
	// float last_min = 0;
	// float nums = 0;
	// for (auto it : last->ring_ponts_)
	// {
	// 	last_min = last_min - 1.0 * it.second->y_;
	// 	nums++;
	// 	if (nums >= 10)
	// 		break;
	// }
	// last_min /= nums;

	// float current_min = 0;
	// float nums1 = 0;
	// for (auto it : current->ring_ponts_)
	// {
	// 	current_min = current_min - 1.0 * it.second->y_;
	// 	nums1++;
	// 	if (nums1 >= 10)
	// 		break;
	// }
	// current_min /= nums1;
	float last_min = -1 * last->ring_ponts_.begin()->second->y_;
	float current_min = -1 * current->ring_ponts_.begin()->second->y_;

	current->status_ = "ground";

	double m = (ring_R_ - ring_r_) / ((double)ring_row_);
	if (last->index_x_ == current->index_x_)
	{
		//径向
		double lxy = m;
		if ((current_min - last_min) / lxy > max_between_cell_high_)
			current->status_ = "all_obs";
	}
	else
	{
		//弧向
		double lxy = (ring_r_ + m * current->index_y_) * ring_theta_ / ((double)ring_col_);
		if ((current_min - last_min) / lxy > max_between_cell_high_)
			current->status_ = "all_obs";
	}
	// if ((current_min - last_min) > max_between_cell_high_)
	// {
	// 	current->status_ = "all_obs";
	// }

	for (auto it : current->ring_ponts_)
	{
		if (current->status_ == "all_obs")
		{
			it.second->is_ground_ = 22;
			continue;
		}
		else
			// {
			// 	it.second->is_ground_ = 11;
			// 	continue;
			// }
			// 判断障碍物，没有考虑地势
			if (-1.0 * it.second->y_ > (current_min + min_obs_))
		{
			it.second->is_ground_ = 22;
			current->status_ = "obs_and_ground";
		}
		else
			it.second->is_ground_ = 11;
	}
}
void mysystem::segment_based_wave()
{
	// 找到种子点，取ponts_db_中前面稳定收敛的最低点为种子点
	mypoint *lowest_point = ponts_db_.begin()->second;
	unsigned int x = lowest_point->ring_cell_->index_x_;
	unsigned int y = lowest_point->ring_cell_->index_y_;
	std::string id = to_string(x) + ",";
	id = id + to_string(y);
	edge_ring_cells[id] = lowest_point->ring_cell_;
	edge_ring_cells[id]->status_ = "all_ground";
	for (auto it : lowest_point->ring_cell_->ring_ponts_)
	{
		// 判断障碍物，没有考虑地势
		if (-1.0 * it.second->y_ > (ponts_db_.begin()->first + min_obs_))
		{
			it.second->is_ground_ = 22;
			edge_ring_cells[id]->status_ = "obs_and_ground";
		}
		else
			it.second->is_ground_ = 11;
	}

	//map<string, ring_cell *> explored_ring_cells;
	while (edge_ring_cells.size() != 0)
	{
		map<string, ring_cell *> edge_ring_cells_temp;
		for (auto it : edge_ring_cells)
		{
			unsigned int x = it.second->index_x_;
			unsigned int y = it.second->index_y_;

			std::string id1 = to_string(x) + ",";
			id1 = id1 + to_string(y - 1);
			if (explored_ring_cells.find(id1) == explored_ring_cells.end() && y >= 1)
			{
				update_mypoints_status(ring_cellmap_[x][y], ring_cellmap_[x][y - 1]);
				if (ring_cellmap_[x][y - 1]->status_ != "all_obs" && ring_cellmap_[x][y - 1]->status_ != "just_init")
				{
					edge_ring_cells_temp[id1] = ring_cellmap_[x][y - 1];
					explored_ring_cells[id1] = ring_cellmap_[x][y - 1];
				}
			}

			std::string id2 = to_string(x - 1) + ",";
			id2 = id2 + to_string(y);
			if (explored_ring_cells.find(id2) == explored_ring_cells.end() && x >= 1)
			{
				update_mypoints_status(ring_cellmap_[x][y], ring_cellmap_[x - 1][y]);
				if (ring_cellmap_[x - 1][y]->status_ != "all_obs" && ring_cellmap_[x - 1][y]->status_ != "just_init")
				{
					edge_ring_cells_temp[id2] = ring_cellmap_[x - 1][y];
					explored_ring_cells[id2] = ring_cellmap_[x - 1][y];
				}
			}

			std::string id3 = to_string(x + 1) + ",";
			id3 = id3 + to_string(y);
			if (explored_ring_cells.find(id3) == explored_ring_cells.end() && x + 1 < ring_col_)
			{
				update_mypoints_status(ring_cellmap_[x][y], ring_cellmap_[x + 1][y]);
				if (ring_cellmap_[x + 1][y]->status_ != "all_obs" && ring_cellmap_[x + 1][y]->status_ != "just_init")
				{
					edge_ring_cells_temp[id3] = ring_cellmap_[x + 1][y];
					explored_ring_cells[id3] = ring_cellmap_[x + 1][y];
				}
			}

			std::string id4 = to_string(x) + ",";
			id4 = id4 + to_string(y + 1);
			if (explored_ring_cells.find(id4) == explored_ring_cells.end() && y + 1 < ring_row_)
			{
				update_mypoints_status(ring_cellmap_[x][y], ring_cellmap_[x][y + 1]);
				if (ring_cellmap_[x][y + 1]->status_ != "all_obs" && ring_cellmap_[x][y + 1]->status_ != "just_init")
				{
					edge_ring_cells_temp[id4] = ring_cellmap_[x][y + 1];
					explored_ring_cells[id4] = ring_cellmap_[x][y + 1];
				}
			}
		}
		edge_ring_cells = edge_ring_cells_temp;
		std::cout << "边缘大小: " << edge_ring_cells.size() << std::endl;
	}
	for (unsigned int i = 0; i < ring_col_; i++)
	{
		for (unsigned int j = 0; j < ring_row_; j++)
		{
			if (ring_cellmap_[i][j]->status_ == "just_init")
			{
				for (auto it : ring_cellmap_[i][j]->ring_ponts_)
				{
					it.second->is_ground_ = 22;
				}
			}
		}
	}
}
void mysystem::update_image_map_lable()
{
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			int ground = 0, obs = 0;
			if (cellmap_[i][j].image_ponts_.size() == 0)
			{
				cellmap_[i][j].status_ = "no_occupied";
				continue;
			}
			double ave_high_x = 0.0;
			double ave_high_y = 0.0;
			double ave_high_z = 0.0;
			double depth = 0.0;

			for (auto it : cellmap_[i][j].image_ponts_)
			{
				ave_high_x += it->x_;
				ave_high_y += it->y_;
				ave_high_z += it->z_;
				depth += it->depth_;
				if (it->is_ground_ == 11)
					ground++;
				else if (it->is_ground_ == 22)
					obs++;
				else
					std::cout << "Wrong!! , 在更新图片地图时候出现没有初始化类别的mypoint !!" << std::endl;
			}
			ave_high_x /= (double)cellmap_[i][j].image_ponts_.size();
			ave_high_y /= (double)cellmap_[i][j].image_ponts_.size();
			ave_high_z /= (double)cellmap_[i][j].image_ponts_.size();
			depth /= (double)cellmap_[i][j].image_ponts_.size();

			cellmap_[i][j].ave_z_ = ave_high_z;
			cellmap_[i][j].ave_y_ = ave_high_y;
			cellmap_[i][j].ave_x_ = ave_high_x;
			cellmap_[i][j].ave_depth_ = depth;
			if (ground > obs)
				cellmap_[i][j].status_ = "ground";
			else
				cellmap_[i][j].status_ = "obs";
		}
	}
}
void mysystem::visual_image_map_true_lable()
{
	Mat temp_image(480, 640, rgb_img_.type(), Scalar(0, 0, 0));
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			Rect rec = cv::Rect(i * col_percell_, j * row_percell_, col_percell_, row_percell_);
			vector<Point> contour;
			contour.push_back(rec.tl());
			contour.push_back(Point(rec.tl().x + rec.width, rec.tl().y));
			contour.push_back(Point(rec.tl().x + rec.width, rec.tl().y + rec.height));
			contour.push_back(Point(rec.tl().x, rec.tl().y + rec.height));

			if (true_cellmap_[i][j].status_ == "obs")
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(0, 0, 255)); //fillPoly函数
			else if (true_cellmap_[i][j].status_ == "ground")
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(0, 255, 0)); //
			else if (true_cellmap_[i][j].status_ == "had_dilate")
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(255, 0, 0));
			else
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(0, 0, 0)); //
		}
	}
	string name = "visual_image_map_lable.png";
	cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	cv::imshow(name, temp_image);
	cv::waitKey();
	imwrite(name, temp_image);
}
void mysystem::visual_image_map_lable()
{
	Mat temp_image(480, 640, rgb_img_.type(), Scalar(0, 0, 0));
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			Rect rec = cv::Rect(i * col_percell_, j * row_percell_, col_percell_, row_percell_);
			vector<Point> contour;
			contour.push_back(rec.tl());
			contour.push_back(Point(rec.tl().x + rec.width, rec.tl().y));
			contour.push_back(Point(rec.tl().x + rec.width, rec.tl().y + rec.height));
			contour.push_back(Point(rec.tl().x, rec.tl().y + rec.height));

			if (cellmap_[i][j].status_ == "obs")
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(0, 0, 255)); //fillPoly函数
			else if (cellmap_[i][j].status_ == "ground")
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(0, 255, 0)); //
			else if (cellmap_[i][j].status_ == "had_dilate")
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(255, 0, 0));
			else
				cv::fillConvexPoly(temp_image, contour, cv::Scalar(0, 0, 0)); //
		}
	}
	string name = "visual_image_map_lable.png";
	cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	cv::imshow(name, temp_image);
	cv::waitKey();
	imwrite(name, temp_image);
}
void mysystem::optimize_segment()
{
	for (unsigned int k = 0; k < edge_map_.size(); k++)
	{
		for (unsigned int i = 0; i < kenel_size_; i++)
		{
			double score_obs = 0.0;
			double score_ground = 0.0;
			for (unsigned int j = 0; j < kenel_size_; j++)
			{
				unsigned int x_id = (edge_map_[k]->x_index_ - 0.5 * edge_dilate_size_ + i + 1);
				unsigned int y_id = (edge_map_[k]->y_index_ - 0.5 * edge_dilate_size_ + j + 1);
				if (x_id >= 0 && x_id < col_cellsum_ && y_id >= 0 && y_id < row_cellsum_ && (i != kenel_size_ / 2 || j != kenel_size_ / 2))
				{
					double dis_xyz = 0.0;
					double D = 0.0;
					double s = 1.0;
					if (cellmap_[x_id][y_id].status_ == "obs" || cellmap_[x_id][y_id].status_ == "ground")
					{
						dis_xyz = cellmap_[x_id][y_id].ave_x_ * cellmap_[x_id][y_id].ave_x_ + cellmap_[x_id][y_id].ave_y_ * cellmap_[x_id][y_id].ave_y_ + cellmap_[x_id][y_id].ave_z_ * cellmap_[x_id][y_id].ave_z_;
						double temp = sqrt(dis_xyz);
						if (temp > 10.0)
						{
							D = 0.0;
							continue;
						}
						D = exp(-1.0 * s * temp);

						if (cellmap_[x_id][y_id].status_ == "obs")
						{
							score_obs += D;
						}
						else
						{
							score_ground += D;
						}
					}
				}
				else
					continue;
			}
			if (score_obs > score_ground)
			{
				edge_map_[k]->status_ = "obs";
				for (size_t i = 0; i < edge_map_[k]->image_ponts_.size(); i++)
				{
					edge_map_[k]->image_ponts_[i]->is_ground_ = 22;
				}
			}

			else
			{
				edge_map_[k]->status_ = "ground";
				for (size_t i = 0; i < edge_map_[k]->image_ponts_.size(); i++)
				{
					edge_map_[k]->image_ponts_[i]->is_ground_ = 11;
				}
			}
		}
	}
}
bool mysystem::detect_dilate(unsigned int x, unsigned int y)
{
	for (unsigned int i = 0; i < edge_dilate_size_; i++)
	{
		for (unsigned int j = 0; j < edge_dilate_size_; j++)
		{
			unsigned int x_id = (x - 0.5 * edge_dilate_size_ + i + 1);
			unsigned int y_id = (y - 0.5 * edge_dilate_size_ + j + 1);
			if (x_id >= 0 && x_id < col_cellsum_ && y_id >= 0 && y_id < row_cellsum_)
			{
				if (cellmap_[x_id][y_id].status_ == "obs")
				{
					return true;
				}
				else
					continue;
			}
			else
				continue;
		}
	}
	return false;
}
void mysystem::get_dilate_edge()
{
	for (unsigned int i = 0; i < col_cellsum_; i++)
	{
		for (unsigned int j = 0; j < row_cellsum_; j++)
		{
			if (cellmap_[i][j].status_ == "ground" && detect_dilate(i, j)) //  && detect_dilate(i, j)
			{
				cellmap_[i][j].status_ = "had_dilate";
				edge_map_.push_back(&cellmap_[i][j]);
			}
		}
	}

	for (size_t i = 0; i < edge_map_.size(); i++)
	{
		for (size_t j = 0; j < edge_map_[i]->image_ponts_.size(); j++)
		{
			edge_map_[i]->image_ponts_[j]->is_ground_ = 99;
		}
	}
}
void mysystem::get_true_lable_image_map()
{
	// true map
	for (unsigned int i_index = 0; i_index < col_cellsum_; i_index++)
	{
		for (unsigned int j_index = 0; j_index < row_cellsum_; j_index++)
		{
			unsigned int start_u = i_index * col_percell_;
			unsigned int start_v = j_index * row_percell_;
			int ground_point = 0, obs_point = 0, unsure = 0;
			for (unsigned int u = 0; u < col_percell_; u++)
			{
				for (unsigned int v = 0; v < row_percell_; v++)
				{
					unsigned int b = rgb_img_.at<Vec3b>(start_v + v, start_u + u)[0];
					unsigned int g = rgb_img_.at<Vec3b>(start_v + v, start_u + u)[1];
					unsigned int r = rgb_img_.at<Vec3b>(start_v + v, start_u + u)[2];
					// 真实分割只有两种颜色128的红和黑色.
					if (r == 128 && g == 0 && b == 0)
					{
						ground_point++;
					}
					else
					{
						obs_point++;
					}
				}
			}
			if (ground_point == pix_sum_percell_)
			{
				true_cellmap_[i_index][j_index].status_ = "ground";
			}
			else if (obs_point == pix_sum_percell_)
			{
				true_cellmap_[i_index][j_index].status_ = "obs";
			}
			else
			{
				true_cellmap_[i_index][j_index].status_ = "no_occupied";
			}
		}
	}
}
void mysystem::calculate_normal()
{
	// Estimating surface normals with depth image gradients for fast and accurate registration 2015 International Conference on 3D Vision
	// 相邻元素叉乘
	// cellmap_[i][j]   cellmap_[i+1][j]
	// cellmap_[i][j+1]
	float cx_ = camera_cx_;
	float cy_ = camera_cy_;
	float f_ = camera_f_;
	Mat temp_image(480, 640, rgb_img_.type(), Scalar(0, 0, 0));

	for (unsigned int x = 0; x < col_cellsum_; x++)
	{
		for (unsigned int y = 0; y < row_cellsum_; y++)
		{
			if (y == row_cellsum_ - 1 || x == col_cellsum_ - 1)
			{
				vector<double> normal(3, 0);
				normal[0] = 0.0;
				normal[1] = 0.0;
				normal[2] = 0.0;
				cellmap_[x][y].normal_ = normal;
				continue;
			}
			double Zxy = cellmap_[x][y].ave_z_;
			double Zx1y = cellmap_[x + 1][y].ave_z_;
			double Zxy1 = cellmap_[x][y + 1].ave_z_;
			// double Zxy = cellmap_[x][y].ave_depth_;
			// double Zx1y = cellmap_[x + 1][y].ave_depth_;
			// double Zxy1 = cellmap_[x][y + 1].ave_depth_;
			if (Zxy == 0 || Zx1y == 0 || Zxy1 == 0)
			{
				vector<double> normal(3, 0);
				normal[0] = 0.0;
				normal[1] = 0.0;
				normal[2] = 0.0;
				cellmap_[x][y].normal_ = normal;
				continue;
			}

			//std::cout << Zxy << ", " << Zx1y << ", " << Zxy1 << endl;

			double Zdx = Zx1y - Zxy;
			double Zdy = Zxy1 - Zxy;

			vector<double> vx(3, 0), vy(3, 0);
			vector<double> normal(3, 0);

			double x_pix = x * col_percell_ - 0.5 * col_percell_;
			double y_pix = y * row_percell_ - 0.5 * row_percell_;

			vx[0] = Zxy / f_ + ((x_pix - cx_) / f_) * Zdx;
			vx[1] = ((y_pix - cy_) / f_) * Zdx;
			vx[2] = Zdx;

			vy[0] = ((x_pix - cx_) / f_) * Zdy;
			vy[1] = Zxy / f_ + ((y_pix - cy_) / f_) * Zdy;
			vy[2] = Zdy;
			normal[0] = vx[1] * vy[2] - vy[1] * vx[2];
			normal[1] = vx[2] * vy[0] - vy[2] * vx[0];
			normal[2] = vx[0] * vy[1] - vy[0] * vx[1];
			//	std::cout << vx[0] << ", " << vx[1] << ", " << vx[2] << endl;
			//std::cout << vy[0] << ", " << vy[1] << ", " << vy[2] << endl;

			//std::cout << normal[0] << ", " << normal[1] << ", " << normal[2] << endl;
			double norm = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
			norm = sqrt(norm);
			normal[0] = normal[0] / norm;
			normal[1] = normal[1] / norm;
			normal[2] = normal[2] / norm;
			cellmap_[x][y].normal_ = normal;

			Rect rec = cv::Rect(x * col_percell_, y * row_percell_, col_percell_, row_percell_);
			vector<Point> contour;
			contour.push_back(rec.tl());
			contour.push_back(Point(rec.tl().x + rec.width, rec.tl().y));
			contour.push_back(Point(rec.tl().x + rec.width, rec.tl().y + rec.height));
			contour.push_back(Point(rec.tl().x, rec.tl().y + rec.height));

			cv::fillConvexPoly(temp_image, contour, cv::Scalar(200.0 * normal[2], 200.0 * normal[1], 200.0 * normal[0])); //fillPoly函数的第二个参数是二维数组！！
		}
	}
	cv::namedWindow("normal_map", cv::WINDOW_AUTOSIZE);
	cv::imshow("normal_map", temp_image);
	cv::waitKey();
	imwrite("normal.jpg", temp_image);
}
void mysystem::run()
{
	get_true_lable_image_map();
	visual_image_map_true_lable();
	// (1)生成点云几何
	//update_map();
	get_pcl_pointcloud();

	debug_mypoint_true_labelme();

	clock_t start = clock();
	segment_based_wave();
	//segment_based_height();
	clock_t ends = clock();
	cout << "Running Time : " << (double)(ends - start) / CLOCKS_PER_SEC << endl;

	debug_mypoint("ring.pcd");
	//更新status_
	update_image_map_lable();
	visual_image_map_lable();

	clock_t start_normal = clock();
	calculate_normal();
	clock_t ends_normal = clock();
	cout << "cal normal Time : " << (double)(ends_normal - start_normal) / CLOCKS_PER_SEC << endl;

	get_dilate_edge();
	visual_image_map_lable();
	debug_mypoint("dilate.pcd");

	clock_t start_optimize = clock();
	optimize_segment();
	clock_t ends_optimize = clock();
	cout << "optimize Time : " << (double)(ends_optimize - start_optimize) / CLOCKS_PER_SEC << endl;

	visual_image_map_lable();
	evaluate();
	debug_mypoint("optimaze.pcd");
}
void mysystem::evaluate()
{
	// IOU_GROUND
	double S = 0;
	double TP = 0.0, TN = 0.0, FP = 0.0, FN = 0.0;

	for (unsigned i = 0; i < col_cellsum_; i++)
	{
		for (unsigned j = 0; j < row_cellsum_; j++)
		{
			if (true_cellmap_[i][j].status_ != "no_occupied" && cellmap_[i][j].status_ != "no_occupied") // 有效范围
			{
				S += 1.0;
				if (true_cellmap_[i][j].status_ == "ground")
				{
					if (cellmap_[i][j].status_ == "ground")
						TP += 1.0;
					else if (cellmap_[i][j].status_ == "obs")
						FN += 1.0;
					else
						cout << "maybe have bug1!" << endl;
				}
				else if (true_cellmap_[i][j].status_ == "obs")
				{
					if (cellmap_[i][j].status_ == "ground")
						FP += 1;
					else if (cellmap_[i][j].status_ == "obs")
					{
						TN += 1.0;
					}
					else
						cout << "maybe have bug2!" << endl;
				}
				else
				{
					cout << "maybe have bug3!" << endl;
				}
			}
		}
	}
	std::cout << "TP:  " << (TP) << std::endl;
	std::cout << "FP:  " << (FP) << std::endl;
	std::cout << "FN:  " << (FN) << std::endl;
	std::cout << "TN:  " << (TN) << std::endl;

	std::cout << "IOU_GROUND:  " << (TP / (TP + FP + FN)) << std::endl;
	std::cout << "Recall_GROUND:  " << (TP / (TP + FN)) << std::endl;
	std::cout << "Recall_OBS:  " << (TN / (TN + FN)) << std::endl;
}
