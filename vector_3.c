/*
  vector_3.cpp - Vector library for bed leveling
  Copyright (c) 2012 Lars Brubaker.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <math.h>
#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"

vector_3* vector_3_init_3(float x, float y, float z)
{
  vector_3 *v = (vector_3 *)malloc(sizeof(vector_3));
  v->x = x;
  v->y = y;
  v->z = z;
  return v;
}

void vector_3_apply_rotation(vector_3 *v, matrix_3x3 matrix)
{
	float resultX = v->x * matrix.matrix[3*0+0] + v->y * matrix.matrix[3*1+0] + v->z * matrix.matrix[3*2+0];
	float resultY = v->x * matrix.matrix[3*0+1] + v->y * matrix.matrix[3*1+1] + v->z * matrix.matrix[3*2+1];
	float resultZ = v->x * matrix.matrix[3*0+2] + v->y * matrix.matrix[3*1+2] + v->z * matrix.matrix[3*2+2];

	v->x = resultX;
	v->y = resultY;
	v->z = resultZ;
}

void matrix_3x3_set_to_identity(matrix_3x3 *m)
{
  m->matrix[0] = 1; m->matrix[1] = 0; m->matrix[2] = 0;
  m->matrix[3] = 0; m->matrix[4] = 1; m->matrix[5] = 0;
  m->matrix[6] = 0; m->matrix[7] = 0; m->matrix[8] = 1;
}

void apply_rotation_xyz(matrix_3x3 matrix, float *x, float *y, float *z)
{
	vector_3 *vector = vector_3_init_3(*x, *y, *z);
  vector_3_apply_rotation(vector, matrix);
	*x = vector->x;
	*y = vector->y;
	*z = vector->z;
}

matrix_3x3 matrix_3x3_transpose(matrix_3x3 original)
{
  matrix_3x3 new_matrix;
  new_matrix.matrix[0] = original.matrix[0]; new_matrix.matrix[1] = original.matrix[3]; new_matrix.matrix[2] = original.matrix[6]; 
  new_matrix.matrix[3] = original.matrix[1]; new_matrix.matrix[4] = original.matrix[4]; new_matrix.matrix[5] = original.matrix[7]; 
  new_matrix.matrix[6] = original.matrix[2]; new_matrix.matrix[7] = original.matrix[5]; new_matrix.matrix[8] = original.matrix[8];
  return new_matrix;
}


#endif // #ifdef ENABLE_AUTO_BED_LEVELING

/* vi: set et sw=2 sts=2: */
