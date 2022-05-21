#ifndef __LADYBUGGEOMTRY_H____
#define __LADYBUGGEOMTRY_H____
#include <math.h>
class LadybugGeom
{
public:
	static const int CAMERA_COUNT = 6;
	LadybugGeom()
	{
		isMatricesComputed = false;
	}
	
	
	void setCameraParams(int cameraNo, double focal, double centerX, double centerY, double _rotX, double _rotY, double _rotZ,
			double _transX, double _transY, double _transZ)
	{
		dFocalLen[cameraNo]=focal;
		dCameraCenterX[cameraNo] = centerX;
		dCameraCenterY[cameraNo] = centerY;

		rotX[cameraNo] = _rotX;
		rotY[cameraNo] = _rotY;
		rotZ[cameraNo] = _rotZ;

		transX[cameraNo] = _transX;
		transY[cameraNo] = _transY;
		transZ[cameraNo] = _transZ;
	}
	void setParamsGloabalRotCamera0(double x, double y, double z)
	{
		dRx = x;
		dRy = y;
		dRz = z;

	}
	void myladybugXYZtoRC(double x, double y, double z, int camera, double *u, double *v)
	{
		
		double toGlobalCoords[4][4]; // Craig's Matrix
		makeTransformation(rotX[camera], rotY[camera], rotZ[camera], transX[camera], transY[camera], transZ[camera], toGlobalCoords);
		invertMatrix(toGlobalCoords, toLocalCoords);
		makeTransformation(dRx, dRy, dRz, 0, 0, 0, cam0rot);
		invertMatrix(cam0rot, cam0rot);


		
		// transform to local
		double dLocalX = x;
		double dLocalY = y;
		double dLocalZ = z;

		applyTransformation(cam0rot, dLocalX, dLocalY, dLocalZ);
		applyTransformation(toLocalCoords, dLocalX, dLocalY, dLocalZ);
		//printf("dLocal : %f,%f,%f \n", dLocalX, dLocalY, dLocalZ);
		if (dLocalZ > 0)
		{
			*v = (dLocalX / dLocalZ) * dFocalLen[camera] + dCameraCenterX[camera];
			*u = (dLocalY / dLocalZ) * dFocalLen[camera] + dCameraCenterY[camera];
		}
		else
		{
			*v = -1;
			*u = -1;
		}
			
	}
	// pinhole model params
	double dFocalLen[CAMERA_COUNT];
	double dCameraCenterX[CAMERA_COUNT];
	double dCameraCenterY[CAMERA_COUNT];
	double rotX[CAMERA_COUNT];
	double rotY[CAMERA_COUNT];
	double rotZ[CAMERA_COUNT];
	
	double transX[CAMERA_COUNT];
	double transY[CAMERA_COUNT];
	double transZ[CAMERA_COUNT];
	double dRx;
	double dRy;
	double dRz;

private:
	double toLocalCoords[4][4];
	double cam0rot[4][4];
	bool isMatricesComputed;	
	void makeTransformation( const double rotX, const double rotY, const double rotZ, const double transX, const double transY, const double transZ,
								 double matrix[4][4] /*out*/ )
		{
			double cosX, sinX, cosY, sinY, cosZ, sinZ;

			cosX = cos( rotX );		sinX = sin( rotX );
			cosY = cos( rotY );		sinY = sin( rotY );
			cosZ = cos( rotZ );		sinZ = sin( rotZ );

			// translation portion of transform
			matrix[0][3] = transX;
			matrix[1][3] = transY;
			matrix[2][3] = transZ;

			// cz*cy;
			matrix[0][0] = cosZ * cosY; 
			// cz*sy*sx - sz*cx;
			matrix[0][1] = cosZ * sinY * sinX - sinZ * cosX; 
			// cz*sy*cx + sz*sx;
			matrix[0][2] = cosZ * sinY * cosX + sinZ * sinX; 

			// sz*cy;
			matrix[1][0] = sinZ * cosY; 
			// sz*sy*sx + cz*cx;
			matrix[1][1] = sinZ * sinY * sinX + cosZ * cosX; 
			// sz*sy*cx - cz*sx;
			matrix[1][2] = sinZ * sinY * cosX - cosZ * sinX; 

			//-sy;
			matrix[2][0] = -sinY; 
			//cy*sx;
			matrix[2][1] = cosY * sinX; 
			//cy*cx;
			matrix[2][2] = cosY * cosX; 

			// bottom row, always the same
			matrix[3][0] = 0.0;
			matrix[3][1] = 0.0;
			matrix[3][2] = 0.0;
			matrix[3][3] = 1.0;
		}

		void applyTransformation(const double matrix[4][4], double & x, double & y, double & z)
		{
			double outPoint[3];
			for ( int r = 0; r < 3; r++ )
			{
				outPoint[r] = matrix[r][3] 
				            + matrix[r][0] * x
				            + matrix[r][1] * y
				            + matrix[r][2] * z ;
			}
			x = outPoint[0];
			y = outPoint[1];
			z = outPoint[2];
		}
		
	void invertMatrix(double in[4][4], double out[4][4])
	{
		double inGL[16] = { 0 };
		double outGL[16] = { 0 };

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				inGL[j * 4 + i] = in[i][j];
			}
		}
		_gluInvertMatrix(inGL, outGL);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				out[i][j] = outGL[j * 4 + i];
			}
		}
	}
	
	bool _gluInvertMatrix(const double m[16], double invOut[16])
	{
		double inv[16], det;
		int i;

		inv[0] = m[5] * m[10] * m[15] -
			m[5] * m[11] * m[14] -
			m[9] * m[6] * m[15] +
			m[9] * m[7] * m[14] +
			m[13] * m[6] * m[11] -
			m[13] * m[7] * m[10];

		inv[4] = -m[4] * m[10] * m[15] +
			m[4] * m[11] * m[14] +
			m[8] * m[6] * m[15] -
			m[8] * m[7] * m[14] -
			m[12] * m[6] * m[11] +
			m[12] * m[7] * m[10];

		inv[8] = m[4] * m[9] * m[15] -
			m[4] * m[11] * m[13] -
			m[8] * m[5] * m[15] +
			m[8] * m[7] * m[13] +
			m[12] * m[5] * m[11] -
			m[12] * m[7] * m[9];

		inv[12] = -m[4] * m[9] * m[14] +
			m[4] * m[10] * m[13] +
			m[8] * m[5] * m[14] -
			m[8] * m[6] * m[13] -
			m[12] * m[5] * m[10] +
			m[12] * m[6] * m[9];

		inv[1] = -m[1] * m[10] * m[15] +
			m[1] * m[11] * m[14] +
			m[9] * m[2] * m[15] -
			m[9] * m[3] * m[14] -
			m[13] * m[2] * m[11] +
			m[13] * m[3] * m[10];

		inv[5] = m[0] * m[10] * m[15] -
			m[0] * m[11] * m[14] -
			m[8] * m[2] * m[15] +
			m[8] * m[3] * m[14] +
			m[12] * m[2] * m[11] -
			m[12] * m[3] * m[10];

		inv[9] = -m[0] * m[9] * m[15] +
			m[0] * m[11] * m[13] +
			m[8] * m[1] * m[15] -
			m[8] * m[3] * m[13] -
			m[12] * m[1] * m[11] +
			m[12] * m[3] * m[9];

		inv[13] = m[0] * m[9] * m[14] -
			m[0] * m[10] * m[13] -
			m[8] * m[1] * m[14] +
			m[8] * m[2] * m[13] +
			m[12] * m[1] * m[10] -
			m[12] * m[2] * m[9];

		inv[2] = m[1] * m[6] * m[15] -
			m[1] * m[7] * m[14] -
			m[5] * m[2] * m[15] +
			m[5] * m[3] * m[14] +
			m[13] * m[2] * m[7] -
			m[13] * m[3] * m[6];

		inv[6] = -m[0] * m[6] * m[15] +
			m[0] * m[7] * m[14] +
			m[4] * m[2] * m[15] -
			m[4] * m[3] * m[14] -
			m[12] * m[2] * m[7] +
			m[12] * m[3] * m[6];

		inv[10] = m[0] * m[5] * m[15] -
			m[0] * m[7] * m[13] -
			m[4] * m[1] * m[15] +
			m[4] * m[3] * m[13] +
			m[12] * m[1] * m[7] -
			m[12] * m[3] * m[5];

		inv[14] = -m[0] * m[5] * m[14] +
			m[0] * m[6] * m[13] +
			m[4] * m[1] * m[14] -
			m[4] * m[2] * m[13] -
			m[12] * m[1] * m[6] +
			m[12] * m[2] * m[5];

		inv[3] = -m[1] * m[6] * m[11] +
			m[1] * m[7] * m[10] +
			m[5] * m[2] * m[11] -
			m[5] * m[3] * m[10] -
			m[9] * m[2] * m[7] +
			m[9] * m[3] * m[6];

		inv[7] = m[0] * m[6] * m[11] -
			m[0] * m[7] * m[10] -
			m[4] * m[2] * m[11] +
			m[4] * m[3] * m[10] +
			m[8] * m[2] * m[7] -
			m[8] * m[3] * m[6];

		inv[11] = -m[0] * m[5] * m[11] +
			m[0] * m[7] * m[9] +
			m[4] * m[1] * m[11] -
			m[4] * m[3] * m[9] -
			m[8] * m[1] * m[7] +
			m[8] * m[3] * m[5];

		inv[15] = m[0] * m[5] * m[10] -
			m[0] * m[6] * m[9] -
			m[4] * m[1] * m[10] +
			m[4] * m[2] * m[9] +
			m[8] * m[1] * m[6] -
			m[8] * m[2] * m[5];

		det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

		if (det == 0)
			return false;

		det = 1.0 / det;

		for (i = 0; i < 16; i++)
			invOut[i] = inv[i] * det;

		return true;
	}
};


#endif
