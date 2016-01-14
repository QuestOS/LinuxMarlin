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
#ifndef VECTOR_3_H
#define VECTOR_3_H

#ifdef ENABLE_AUTO_BED_LEVELING
typedef struct _matrix_3x3
{
  float matrix[9];
} matrix_3x3;

typedef struct _vector_3
{
  float x, y, z;
} vector_3;

vector_3 *vector_3_init_3(float x, float y, float z);
void vector_3_apply_rotation(vector_3 *v, matrix_3x3 matrix);
void matrix_3x3_set_to_identity(matrix_3x3 *m);
void apply_rotation_xyz(matrix_3x3 rotationMatrix, float *x, float *y, float *z);
matrix_3x3 matrix_3x3_transpose(matrix_3x3 original);
#endif

#endif

/* vi: set et sw=2 sts=2: */
