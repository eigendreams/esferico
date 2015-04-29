#include "structs.h"

#ifndef ROBOTSPACE_IO_UTILS
#define ROBOTSPACE_IO_UTILS

namespace RobotSpace
{
// Funciones ayudantes de conversión entre velocidades en términos de las ruedas (w_w), o en términos
// de velocidad angular y velocidad tangencial lineal (v_w)

w_w_form ww_from_vw(v_w_form inputIs, robot_data robotData)
{
	w_w_form outputIs;

	outputIs.w_r = (inputIs.v + (inputIs.w * robotData.wheel_separation) / 2) / robotData.wheel_radii;
	outputIs.w_l = (inputIs.v - (inputIs.w * robotData.wheel_separation) / 2) / robotData.wheel_radii;

	return outputIs;
}

v_w_form vw_from_ww(w_w_form inputIs, robot_data robotData)
{
	v_w_form outputIs;

	outputIs.v = robotData.wheel_radii * 0.5 * (inputIs.w_r + inputIs.w_l);
	outputIs.w = robotData.wheel_radii * (inputIs.w_r - inputIs.w_l) / robotData.wheel_separation;

	return outputIs;
}
}
#endif
