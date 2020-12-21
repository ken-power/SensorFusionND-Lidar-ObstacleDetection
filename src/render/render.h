#ifndef RENDER_H
#define RENDER_H

#include <iostream>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>

#include "box.h"

struct Color
{
	float r, g, b;

	Color(float r_value, float g_value, float b_value)
		: r(r_value), g(g_value), b(b_value)
	{}
};

enum CameraAngle
{
	XY, TopDown, Side, FPS
};

void RenderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      std::string name,
                      Color color = Color(-1, -1, -1));

void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer,
               Box box,
               int id,
               Color color = Color(1, 0, 0),
               float opacity = 1);

void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer,
               BoxQ box,
               int id,
               Color color = Color(1, 0, 0),
               float opacity = 1);

#endif
