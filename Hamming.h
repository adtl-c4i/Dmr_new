/*
 *   Copyright (C) 2015,2016 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *   ---
 *   Converted from C++ to C. The `CHamming` class had only static member
 *   functions and no state, so it becomes a set of plain C functions with
 *   a `CHamming_` prefix in place of the `CHamming::` scope.
 */

#ifndef	Hamming_H
#define	Hamming_H

#include <stdbool.h>

void CHamming_encode15113_1(bool* d);
bool CHamming_decode15113_1(bool* d);

void CHamming_encode15113_2(bool* d);
bool CHamming_decode15113_2(bool* d);

void CHamming_encode1393(bool* d);
bool CHamming_decode1393(bool* d);

void CHamming_encode1063(bool* d);
bool CHamming_decode1063(bool* d);

void CHamming_encode16114(bool* d);
bool CHamming_decode16114(bool* d);

void CHamming_encode17123(bool* d);
bool CHamming_decode17123(bool* d);

#endif
