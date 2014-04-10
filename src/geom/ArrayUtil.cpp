/*
 * gridUtil.cpp
 *
 *  Created on: May 20, 2013
 *      Author: daikiyamanaka
 */


#include "ArrayUtil.h"

template void allocate3D(int ***p, int sx, int sy, int sz, int value);
template void allocate3D(float ***p, int sx, int sy, int sz, float value);
template void allocate3D(double ***p, int sx, int sy, int sz, double value);

template void save3D(float ***p, int sx, int sy, int sz, std::string filename);
template void save3D(double ***p, int sx, int sy, int sz, std::string filename);
template void save3D(int ***p, int sx, int sy, int sz, std::string filename);

template <class T> void save3D(T ***p, int sx, int sy, int sz, std::string filename){
	// --- output ---
	std::ofstream ofs(filename.c_str(), std::ios::out| std::ios::binary);
	T *buff = new T[sx];
	for(int z=0; z<sz; z++){
		for(int y=0; y<sy; y++){
			for(int x=0; x<sx; x++){
				buff[x] = p[x][y][z];
			}
			ofs.write((char *)buff, sx*sizeof(T));
		}
	}
	ofs.close();

	delete(buff);
}

template <class X> void allocate3D(X ***p, int sx, int sy, int sz, X value){
	for (int i = 0; i < sx; i++) {
			p[i] = p[0] + i * sy;
			for (int j = 0; j < sy; j++)
				p[i][j] = p[0][0] + i * sy * sz + j * sz;
		}

 	for(int i=0; i<sx; i++){
 		for(int j=0; j<sy; j++){
 			for(int k=0; k<sz; k++){
 				p[i][j][k] = value;
 			}
 		}
 	}
}

