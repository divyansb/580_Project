/* CS580 Homework 3 */

#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"

#define PI (float) 3.14159265358979323846

//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
// SMALL HELPERS	||
//------------------//

#define KA				GZ_AMBIENT_COEFFICIENT	
#define KD				GZ_DIFFUSE_COEFFICIENT	
#define KS				GZ_SPECULAR_COEFFICIENT	
#define SPECULAR_POWER  GZ_DISTRIBUTION_COEFFICIENT 

#define max(a, b)		((a > b) ? a : b)
#define min(a, b)		((a < b) ? a : b)

#define COPY_COORD(A, B)	A[0] = B[0];\
							A[1] = B[1];\
							A[2] = B[2];

const float degToRad = PI / 180;

const GzMatrix I_4X4 = { 1, 0, 0, 0,
0, 1, 0, 0,
0, 0, 1, 0,
0, 0, 0, 1 };

template< typename type >
inline type clampvalue(type val, type min, type max)
{
	return (val > max) ? max : (val < min ? min : val);
}

void CrossProduct(float a[3], float b[3], float productVector[3])
{
	// productVector will have the components of aXb
	productVector[0] = a[1] * b[2] - a[2] * b[1];
	productVector[1] = a[2] * b[0] - a[0] * b[2];
	productVector[2] = a[0] * b[1] - a[1] * b[0];
}

void NormalizeVector(float vec[3])
{
	float sumOfSquares = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
	sumOfSquares = sqrt(sumOfSquares);
	vec[0] = vec[0] / sumOfSquares;
	vec[1] = vec[1] / sumOfSquares;
	vec[2] = vec[2] / sumOfSquares;
}

void MatrixMultiply(GzMatrix matA, GzMatrix matB, GzMatrix matRes)
{
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			matRes[row][col] = 0;
			for (int it = 0; it < 4; ++it)
			{
				matRes[row][col] += (matA[row][it] * matB[it][col]);
			}
		}
	}
}

void MatrixMulScalar(GzMatrix mat, float scalar)
{
	for (int row = 0; row < 4; ++row)
		for (int col = 0; col < 4; ++col)
			mat[row][col] = mat[row][col] * scalar;
}

inline float Vec4Dot(float a[4], float b[4])
{
	float dot = 0;
	for (int it = 0; it < 4; ++it)
	{
		dot += a[it] * b[it];
	}
	return dot;
}

inline float Vec3Dot(float a[3], float b[3])
{
	float dot = 0;
	for (int it = 0; it < 3; ++it)
	{
		dot += a[it] * b[it];
	}
	return dot;
}

void TransformCoord(GzCoord &positionVector, GzMatrix transformationMatrix)
{
	float vecHomogenous[4] = { positionVector[0], positionVector[1], positionVector[2], 1 };
	float w = Vec4Dot(vecHomogenous, transformationMatrix[3]);
	positionVector[0] = Vec4Dot(vecHomogenous, transformationMatrix[0]) / w;
	positionVector[1] = Vec4Dot(vecHomogenous, transformationMatrix[1]) / w;
	positionVector[2] = Vec4Dot(vecHomogenous, transformationMatrix[2]) / w;
}

void CopyMatrix(const GzMatrix from, GzMatrix to)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			to[i][j] = from[i][j];
}

void CoordScalarMul(GzCoord coord, float scalar)
{
	for (int i = 0; i < 3; ++i)
		coord[i] *= scalar;
}

void CoordDiff(GzCoord coord1, GzCoord coord2)
{
	for (int i = 0; i < 3; ++i)
		coord1[i] = coord1[i] - coord2[i];
}

void ConvertToPureRotationMatrix(GzMatrix mat)
{
	// assuming uniform scale
	// remove scale
	float scale = sqrt(mat[0][0] * mat[0][0] + mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2]);
	if (scale != 1)
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				mat[i][j] = mat[i][j] / scale;

	// remove translation
	mat[0][3] = 0;
	mat[1][3] = 0;
	mat[2][3] = 0;
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
// HOMEWORK FUNCTIONS, TRANSFORMATION MATRICES	||
//----------------------------------------------//
int GzRender::GzRotXMat(float degree, GzMatrix mat)
{
/* HW 3.1
// Create rotate matrix : rotate along x axis
// Pass back the matrix using mat value
*/
	float radians = degree * degToRad;
	float tempArray[4][4] = {
		1,		0,				0,						0,
		0,		cos(radians),	(-1)*(sin(radians)),	0,
		0,		sin(radians),	cos(radians),			0,
		0,		0,				0,						1
	};

	CopyMatrix(tempArray, mat);

	return GZ_SUCCESS;
}

int GzRender::GzRotYMat(float degree, GzMatrix mat)
{
/* HW 3.2
// Create rotate matrix : rotate along y axis
// Pass back the matrix using mat value
*/
	float radians = degree * degToRad;
	float tempArray[4][4] = {
		cos(radians),		0,				sin(radians),			0,
		0,					1,				0,						0,
		(-1)*sin(radians),	0,				cos(radians),			0,
		0,					0,				0,						1
	};

	CopyMatrix(tempArray, mat);
	return GZ_SUCCESS;
}

int GzRender::GzRotZMat(float degree, GzMatrix mat)
{
/* HW 3.3
// Create rotate matrix : rotate along z axis
// Pass back the matrix using mat value
*/
	float radians = degree * degToRad;
	float tempArray[4][4] = {
		cos(radians),		(-1)*sin(radians),			0,	0,
		sin(radians),		cos(radians),				0,	0,
		0,					0,							1,	0,
		0,					0,							0,	1
	};

	CopyMatrix(tempArray, mat);
	return GZ_SUCCESS;
}

int GzRender::GzTrxMat(GzCoord translate, GzMatrix mat)
{
/* HW 3.4
// Create translation matrix
// Pass back the matrix using mat value
*/
	float tempArray[4][4] = {
		1,		0,		0,		translate[0],
		0,		1,		0,		translate[1],
		0,		0,		1,		translate[2],
		1,		0,		0,		1
	};

	CopyMatrix(tempArray, mat);
	return GZ_SUCCESS;
}


int GzRender::GzScaleMat(GzCoord scale, GzMatrix mat)
{
/* HW 3.5
// Create scaling matrix
// Pass back the matrix using mat value
*/
	float tempArray[4][4] = {
		scale[0],		0,				0,				0,
		0,				scale[1],		0,				0,
		0,				0,				scale[2],		0,
		1,				0,				0,				1
	};

	CopyMatrix(tempArray, mat);

	return GZ_SUCCESS;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
// IMAGE SPACE MATRIX COMPUTATION FUNCTIONS, XPI AND XIW	||
//----------------------------------------------------------//
void ComputeXpiForCamera(GzCamera &camera)
{
	// Xpi
	GzMatrix tempMat = {
		1,				0,				0,						0,
		0,				1,				0,						0,
		0,				0,				tan((camera.FOV / 2) * degToRad),	0,
		1,				0,				tan((camera.FOV / 2) * degToRad),	1
	};
	CopyMatrix(tempMat, camera.Xpi);
}

void ComputeXiwForCamera(GzCamera &camera)
{
	// Xiw
	GzCoord camZ = {
		camera.lookat[0] - camera.position[0],
		camera.lookat[1] - camera.position[1],
		camera.lookat[2] - camera.position[2]
	};
	NormalizeVector(camZ);

	GzCoord camY;
	float upDotZ = Vec3Dot(camera.worldup, camZ);
	camY[0] = camera.worldup[0] - upDotZ * camZ[0];
	camY[1] = camera.worldup[1] - upDotZ * camZ[1];
	camY[2] = camera.worldup[2] - upDotZ * camZ[2];
	NormalizeVector(camY);

	GzCoord camX;
	CrossProduct(camY, camZ, camX);

	GzMatrix tempMat = {
		camX[0],		camX[1],		camX[2],		(-1)*(Vec3Dot(camX, camera.position)),
		camY[0],		camY[1],		camY[2],		(-1)*(Vec3Dot(camY, camera.position)),
		camZ[0],		camZ[1],		camZ[2],		(-1)*(Vec3Dot(camZ, camera.position)),
		0,				0,				0,				1
	};
	CopyMatrix(tempMat, camera.Xiw);
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
// HOMEWORK FUNCTIONS, RENDERING GENERAL	||
//------------------------------------------//
GzRender::GzRender(int xRes, int yRes)
{
	xRes = clampvalue<int>(xRes, 0, MAXXRES);
	yRes = clampvalue<int>(yRes, 0, MAXYRES);

	this->xres = xRes;
	this->yres = yRes;
	this->pixelbuffer = new GzPixel[3 * xRes * yRes];
	this->framebuffer = new char[3 * xRes * yRes];

	/* HW 3.6
	- setup Xsp and anything only done once
	- init default camera
	*/
	// Xsp
	float tempArray[4][4] = {
		this->xres / 2,	0,						0,			this->xres / 2,
		0,				(-1)*(this->yres / 2),	0,			this->yres / 2,
		0,				0,						MAXINT,		0,
		1,				0,						0,			1
	};
	CopyMatrix(tempArray, Xsp);

	// Init default camera
	this->m_camera.FOV = DEFAULT_FOV;
	this->m_camera.lookat[0] = 0;					this->m_camera.lookat[1] = 0;						this->m_camera.lookat[2] = 0;
	this->m_camera.position[0] = DEFAULT_IM_X;		this->m_camera.position[1] = DEFAULT_IM_Y;			this->m_camera.position[2] = DEFAULT_IM_Z;
	this->m_camera.worldup[0] = 0;					this->m_camera.worldup[1] = 1;						this->m_camera.worldup[2] = 0;

	ComputeXpiForCamera(m_camera);
	ComputeXiwForCamera(m_camera);

	// Init number of lights 
	this->numlights = 0;
}

GzRender::~GzRender()
{
	/* HW1.2 clean up, free buffer memory */
	if (pixelbuffer)
		delete pixelbuffer;
	if (framebuffer)
		delete framebuffer;
}

int GzRender::GzDefault()
{
/* HW1.3 set pixel buffer to some default values - start a new frame */
	ASSERT(pixelbuffer);

	for (int pixelIndex_X = 0; pixelIndex_X < this->xres; pixelIndex_X++)
		for (int pixelIndex_Y = 0; pixelIndex_Y < this->xres; pixelIndex_Y++)
		{
			pixelbuffer[ARRAY(pixelIndex_X, pixelIndex_Y)].red = 3000;
			pixelbuffer[ARRAY(pixelIndex_X, pixelIndex_Y)].green = 3000;
			pixelbuffer[ARRAY(pixelIndex_X, pixelIndex_Y)].blue = 3500;
			pixelbuffer[ARRAY(pixelIndex_X, pixelIndex_Y)].alpha = 1;
			pixelbuffer[ARRAY(pixelIndex_X, pixelIndex_Y)].z = MAXINT;
		}

	return GZ_SUCCESS;
}

int GzRender::GzBeginRender()
{
	/* HW 3.7
	- setup for start of each frame - init frame buffer color,alpha,z
	- compute Xiw and projection xform Xpi from camera definition
	- init Ximage - put Xsp at base of stack, push on Xpi and Xiw
	- now stack contains Xsw and app can push model Xforms when needed
	*/

	// init frame buffer color, alpha, z ?? how to do that ?? 

	// Xpi
	ComputeXpiForCamera(m_camera);

	// Xiw
	ComputeXiwForCamera(m_camera);

	// Push transformation matrices on stack
	this->matlevel = -1;
	GzPushMatrix(Xsp);
	GzPushMatrix(m_camera.Xpi);
	GzPushMatrix(m_camera.Xiw);

	return GZ_SUCCESS;
}

int GzRender::GzPutCamera(GzCamera camera)
{
	/* HW 3.8
	/*- overwrite renderer camera structure with new camera definition
	*/
	// fov
	m_camera.FOV = camera.FOV;

	// position
	COPY_COORD(m_camera.position, camera.position);

	// lookat
	COPY_COORD(m_camera.lookat, camera.lookat);

	// worldup
	COPY_COORD(m_camera.worldup, camera.worldup);

	// Xpi
	ComputeXpiForCamera(m_camera);

	// Xiw
	ComputeXiwForCamera(m_camera);

	return GZ_SUCCESS;
}

int GzRender::GzPushMatrix(GzMatrix	matrix)
{
	/* HW 3.9
	- push a matrix onto the Ximage stack
	- check for stack overflow
	*/
	// matlevel is top-of-stack index
	if (matlevel >= MATLEVELS)	// overflow
		return GZ_SUCCESS;

	bool pushIdentityToNormalStack = false;
	if (matrix[3][0] != 0)
	{
		pushIdentityToNormalStack = true;
		matrix[3][0] = 0;
	}

	if (matlevel < 0)	// stack empty
	{
		// push to image stack
		CopyMatrix(matrix, Ximage[++matlevel]);

		// push to normal stack
		if (pushIdentityToNormalStack)
		{
			CopyMatrix(I_4X4, Xnorm[matlevel]);
		}
		else
		{
			ConvertToPureRotationMatrix(matrix);
			CopyMatrix(matrix, Xnorm[matlevel]);
		}
	}
	else
	{
		// push to image stack
		GzMatrix tempResMat;
		MatrixMultiply(Ximage[matlevel], matrix, tempResMat);
		CopyMatrix(tempResMat, Ximage[++matlevel]);

		// push to normal stack
		if (pushIdentityToNormalStack)
		{
			CopyMatrix(Xnorm[matlevel - 1], Xnorm[matlevel]);	// matlevel has been incremented by ximage stack
		}
		else
		{
			ConvertToPureRotationMatrix(matrix);
			MatrixMultiply(Xnorm[matlevel - 1], matrix, tempResMat);
			CopyMatrix(tempResMat, Xnorm[matlevel]);
		}
	}

	return GZ_SUCCESS;
}

int GzRender::GzPopMatrix()
{
	/* HW 3.10
	- pop a matrix off the Ximage stack
	- check for stack underflow
	*/
	if (matlevel < 0)	// underflow
		return GZ_SUCCESS;

	--matlevel;

	return GZ_SUCCESS;
}

int GzRender::GzPut(int i, int j, GzIntensity r, GzIntensity g, GzIntensity b, GzIntensity a, GzDepth z)
{
	/* HW1.4 write pixel values into the buffer */
	ASSERT(pixelbuffer);

	if (i >= 0 && j >= 0 && i < this->xres && j < this->yres)	// ignore values out of the current resolution
	{
		// clamp rgba values
		r = clampvalue<int>(r, 0, 4055);
		g = clampvalue<int>(g, 0, 4055);
		b = clampvalue<int>(b, 0, 4055);
		a = clampvalue<int>(a, 0, 4055);

		pixelbuffer[ARRAY(i, j)].red = r;
		pixelbuffer[ARRAY(i, j)].green = g;
		pixelbuffer[ARRAY(i, j)].blue = b;
		pixelbuffer[ARRAY(i, j)].alpha = a;
		pixelbuffer[ARRAY(i, j)].z = z;
	}

	return GZ_SUCCESS;
}

int GzRender::GzGet(int i, int j, GzIntensity *r, GzIntensity *g, GzIntensity *b, GzIntensity *a, GzDepth *z)
{
	/* HW1.5 retrieve a pixel information from the pixel buffer */
	ASSERT(pixelbuffer);
	if (!(i >= 0 && j >= 0 && i <= this->xres && j <= this->yres))
		return GZ_FAILURE;	// pixel fetched should be on the screen in the rendered resolution

	*r = pixelbuffer[ARRAY(i, j)].red;
	*g = pixelbuffer[ARRAY(i, j)].green;
	*b = pixelbuffer[ARRAY(i, j)].blue;
	*a = pixelbuffer[ARRAY(i, j)].alpha;
	*z = pixelbuffer[ARRAY(i, j)].z;

	return GZ_SUCCESS;
}

int GzRender::GzFlushDisplay2File(FILE* outfile)
{
	/* HW1.6 write image to ppm file -- "P6 %d %d 255\r" */
	ASSERT(pixelbuffer);

	fprintf(outfile, "P6 %d %d 255\r", this->xres, this->yres);
	for (int pixelIndex_y = 0; pixelIndex_y < this->yres; ++pixelIndex_y)
		for (int pixelIndex_x = 0; pixelIndex_x < this->xres; ++pixelIndex_x)
			fprintf(outfile, "%c%c%c", (char)(pixelbuffer[ARRAY(pixelIndex_x, pixelIndex_y)].red >> 4), (char)(pixelbuffer[ARRAY(pixelIndex_x, pixelIndex_y)].green >> 4), (char)(pixelbuffer[ARRAY(pixelIndex_x, pixelIndex_y)].blue >> 4));

	return GZ_SUCCESS;
}

int GzRender::GzFlushDisplay2FrameBuffer()
{
	/* HW1.7 write pixels to framebuffer:
	- put the pixels into the frame buffer
	- CAUTION: when storing the pixels into the frame buffer, the order is blue, green, and red
	- NOT red, green, and blue !!!
	*/
	ASSERT(pixelbuffer);
	ASSERT(framebuffer);

	for (int pixelIndex_y = 0; pixelIndex_y < this->yres; ++pixelIndex_y)
		for (int pixelIndex_x = 0; pixelIndex_x < this->xres; ++pixelIndex_x)
		{
			framebuffer[ARRAY(pixelIndex_x, pixelIndex_y) * 3] = (char)(pixelbuffer[ARRAY(pixelIndex_x, pixelIndex_y)].blue >> 4);
			framebuffer[ARRAY(pixelIndex_x, pixelIndex_y) * 3 + 1] = (char)(pixelbuffer[ARRAY(pixelIndex_x, pixelIndex_y)].green >> 4);
			framebuffer[ARRAY(pixelIndex_x, pixelIndex_y) * 3 + 2] = (char)(pixelbuffer[ARRAY(pixelIndex_x, pixelIndex_y)].red >> 4);
		}

	return GZ_SUCCESS;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
// TRIANGLE RASTERIZATION HELPERS	||
//----------------------------------//

inline void SwapCoords(GzCoord *firstCoord, GzCoord *secondCoord)
{
	for (int coordIndex = 0; coordIndex < 3; ++coordIndex)
	{
		float temp = (*firstCoord)[coordIndex];
		(*firstCoord)[coordIndex] = (*secondCoord)[coordIndex];
		(*secondCoord)[coordIndex] = temp;
	}
}

inline void SwapTexCoords(GzTextureIndex *first, GzTextureIndex *second)
{
	for (int coordIndex = 0; coordIndex < 2; ++coordIndex)
	{
		float temp = (*first)[coordIndex];
		(*first)[coordIndex] = (*second)[coordIndex];
		(*second)[coordIndex] = temp;
	}
}

inline void GenerateLineEquationParameters(GzCoord lineStart, GzCoord lineEnd, float parameters[3])
{
	// parameters will have values A, B, C
	parameters[0] = lineEnd[1] - lineStart[1];												// dY
	parameters[1] = -1 * (lineEnd[0] - lineStart[0]);										// -dX
	parameters[2] = ((-1 * parameters[1]) * lineStart[1]) - (parameters[0] * lineStart[0]);	// dX * Yinit - dY * Xinit
}

enum class VertSortOrder
{
	GZ_CLOCKWISE,
	GZ_ANTI_CLOCKWISE
};

void SortVertices(GzCoord vertices[3], VertSortOrder sortOrder, GzColor vertColors[3], GzCoord vertNorms[3], GzTextureIndex vertUV[3])
{
	// assuming clockwise
	// sort on y coordinates
	if (!(vertices[0][Y] < vertices[1][Y] && vertices[0][Y] < vertices[2][Y]))
	{
		if (vertices[1][Y] < vertices[2][Y])
		{
			SwapCoords(&vertices[0], &vertices[1]);
			SwapCoords(&vertColors[0], &vertColors[1]);
			SwapCoords(&vertNorms[0], &vertNorms[1]);
			SwapTexCoords(&vertUV[0], &vertUV[1]);
		}
		else
		{
			SwapCoords(&vertices[0], &vertices[2]);
			SwapCoords(&vertColors[0], &vertColors[2]);
			SwapCoords(&vertNorms[0], &vertNorms[2]);
			SwapTexCoords(&vertUV[0], &vertUV[2]);
		}
	}

	if (!(vertices[1][Y] < vertices[2][Y]))
	{
		SwapCoords(&vertices[1], &vertices[2]);
		SwapCoords(&vertColors[1], &vertColors[2]);
		SwapCoords(&vertNorms[1], &vertNorms[2]);
		SwapTexCoords(&vertUV[1], &vertUV[2]);
	}

	// if two verts have same y, special case. compare x values.
	if (vertices[0][Y] == vertices[1][Y])
	{
		if (vertices[0][X] > vertices[1][X])
		{
			SwapCoords(&vertices[0], &vertices[1]);
			SwapCoords(&vertColors[0], &vertColors[1]);
			SwapCoords(&vertNorms[0], &vertNorms[1]);
			SwapTexCoords(&vertUV[0], &vertUV[1]);
		}
	}
	else if (vertices[1][Y] == vertices[2][Y])
	{
		if (vertices[1][X] > vertices[2][X])
		{
			SwapCoords(&vertices[1], &vertices[2]);
			SwapCoords(&vertColors[1], &vertColors[2]);
			SwapCoords(&vertNorms[1], &vertNorms[2]);
			SwapTexCoords(&vertUV[1], &vertUV[2]);
		}
	}
	else
	{
		// generate equation of extreme y points( 0 and 2 )
		// Ax + By + C = 0
		float lineEquationParameters[3];
		GenerateLineEquationParameters(vertices[0], vertices[2], lineEquationParameters);
		float xp = 0, yp = vertices[1][Y];
		xp = -1 * (lineEquationParameters[2] + (lineEquationParameters[1] * yp)) / lineEquationParameters[0];
		if (xp > vertices[1][X])
		{
			SwapCoords(&vertices[1], &vertices[2]);
			SwapCoords(&vertColors[1], &vertColors[2]);
			SwapCoords(&vertNorms[1], &vertNorms[2]);
			SwapTexCoords(&vertUV[1], &vertUV[2]);
		}
	}
}

void GenerateInterPolationParameters(GzCoord vertices[3], float interpParameters[4])
{
	float planeNormal[3];
	float vec01[3];
	float vec12[3];
	for (int i = 0; i < 3; ++i)
	{
		vec01[i] = vertices[1][i] - vertices[0][i];
		vec12[i] = vertices[2][i] - vertices[1][i];
	}
	CrossProduct(vec01, vec12, planeNormal);
	NormalizeVector(planeNormal);
	float planeD = -1 * (planeNormal[0] * vertices[0][0] + planeNormal[1] * vertices[0][1] + planeNormal[2] * vertices[0][2]);	// -Ax - By - Cz

	interpParameters[0] = planeNormal[0];
	interpParameters[1] = planeNormal[1];
	interpParameters[2] = planeNormal[2];
	interpParameters[3] = planeD;
}

float GetInterpolatedValue(float x, float y, float interpParameters[4])
{
	float newValue = (-1 * (interpParameters[3] + interpParameters[0] * x + interpParameters[1] * y)) / interpParameters[2];
	return newValue;
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
// HOMEWORK FUNCTIONS, PUT ATTRIB AND TRIANGLE	||
//----------------------------------------------//
/***********************************************/
/* HW2 methods: implement from here */

int GzRender::GzPutAttribute(int numAttributes, GzToken	*nameList, GzPointer *valueList) 
{
/* HW 2.1
-- Set renderer attribute states (e.g.: GZ_RGB_COLOR default color)
-- In later homeworks set shaders, interpolaters, texture maps, and lights
*/

/*
- GzPutAttribute() must accept the following tokens/values:

- GZ_RGB_COLOR					GzColor		default flat-shade color
- GZ_INTERPOLATE				int			shader interpolation mode
- GZ_DIRECTIONAL_LIGHT			GzLight
- GZ_AMBIENT_LIGHT            	GzLight		(ignore direction)
- GZ_AMBIENT_COEFFICIENT		GzColor		Ka reflectance
- GZ_DIFFUSE_COEFFICIENT		GzColor		Kd reflectance
- GZ_SPECULAR_COEFFICIENT       GzColor		Ks coef's
- GZ_DISTRIBUTION_COEFFICIENT   float		spec power
*/
	int attributeIterator = 0;
	GzPointer currentValuePointer = valueList;
	for (attributeIterator; attributeIterator < numAttributes; ++attributeIterator)
	{
		switch (nameList[attributeIterator])
		{
		case GZ_NULL_TOKEN:
			break;
		case GZ_DIRECTIONAL_LIGHT:
		{
			GzLight *lightin = static_cast<GzLight*>(valueList[attributeIterator]);
			lights[numlights].direction[0] = lightin->direction[0];
			lights[numlights].direction[1] = lightin->direction[1];
			lights[numlights].direction[2] = lightin->direction[2];
			lights[numlights].color[0] = lightin->color[0];
			lights[numlights].color[1] = lightin->color[1];
			lights[numlights++].color[2] = lightin->color[2];

			break;
		}
		case GZ_AMBIENT_LIGHT:
		{
			GzLight *lightin = static_cast<GzLight*>(valueList[attributeIterator]);
			ambientlight.direction[0] = lightin->direction[0];
			ambientlight.direction[1] = lightin->direction[1];
			ambientlight.direction[2] = lightin->direction[2];
			ambientlight.color[0] = lightin->color[0];
			ambientlight.color[1] = lightin->color[1];
			ambientlight.color[2] = lightin->color[2];

			break;
		}
		case GZ_INTERPOLATE:
		{
			int *interpin = static_cast<int*>(valueList[attributeIterator]);
			interp_mode = *interpin;
			break;
		}
		case KA:
		{
			float *colorin = static_cast<float*>(valueList[attributeIterator]);
			this->Ka[0] = colorin[0];
			this->Ka[1] = colorin[1];
			this->Ka[2] = colorin[2];

			break;
		}
		case KD:
		{
			float *colorin = static_cast<float*>(valueList[attributeIterator]);

			this->Kd[0] = colorin[0];
			this->Kd[1] = colorin[1];
			this->Kd[2] = colorin[2];

			break;
		}
		case KS:
		{
			float *colorin = static_cast<float*>(valueList[attributeIterator]);

			this->Ks[0] = colorin[0];
			this->Ks[1] = colorin[1];
			this->Ks[2] = colorin[2];

			break;
		}
		case SPECULAR_POWER:
		{
			float *specin = static_cast<float*>(valueList[attributeIterator]);
			this->spec = *specin;
			break;
		}
		case GZ_AASHIFTX:
		{
			float *dx = static_cast<float*>(valueList[attributeIterator]);
			this->xOffset = *dx;
			break;
		}
		case GZ_AASHIFTY:
		{
			float *dy = static_cast<float*>(valueList[attributeIterator]);
			this->yOffset = *dy;
			break;
		}
		case GZ_RGB_COLOR:
		{
			// Read the color component values from the first vertex
			float *colorin = static_cast<float*>(valueList[attributeIterator]);

			flatcolor[0] = colorin[0];
			flatcolor[1] = colorin[1];
			flatcolor[2] = colorin[2];

			break;
		}
		case GZ_TEXTURE_MAP:
		{
			// Read in function pointer
			if (valueList[attributeIterator])
			{
				this->tex_fun = static_cast<GzTexture>(valueList[attributeIterator]);
			}
			else
			{
				tex_fun = nullptr;
			}
		}
		default:
			break;
		}
	}
	return GZ_SUCCESS;
}

int GzRender::GzPutTriangle(int numParts, GzToken *nameList, GzPointer *valueList)
/* numParts - how many names and values */
{
/* HW 2.2
-- Pass in a triangle description with tokens and values corresponding to
	  GZ_NULL_TOKEN:		do nothing - no values
	  GZ_POSITION:		3 vert positions in model space
-- Return error code
*/
/*
-- Xform positions of verts using matrix on top of stack 
-- Clip - just discard any triangle with any vert(s) behind view plane 
		- optional: test for triangles with all three verts off-screen (trivial frustum cull)
-- invoke triangle rasterizer  
*/
	GzCoord *vertexPositionPointer, *vertexNormalPointer;
	GzTextureIndex *vertexUVPointer;

	// Compute colors at triangle vertices using the lighting equations
	GzColor vertexColors[3];

#pragma region Read in the attributes

	// Read in attributes for the triangle to be rendered
	for (int attributeIterator = 0; attributeIterator < numParts; ++attributeIterator)
	{
		switch (nameList[attributeIterator])
		{
		case GZ_NULL_TOKEN:	break;

		case GZ_POSITION:
		{
			// Read in position values.
			vertexPositionPointer = static_cast<GzCoord*>(valueList[attributeIterator]);
			break;
		}

		case GZ_NORMAL:
		{
			// Read normal values
			vertexNormalPointer = static_cast<GzCoord*>(valueList[attributeIterator]);
			break;
		}

		case GZ_TEXTURE_INDEX:
		{
			// Read in texture coordinate values
			vertexUVPointer = static_cast<GzTextureIndex*>(valueList[attributeIterator]);
			break;
		}

		default:	break;
		}
	}

#pragma endregion

	flatcolor[RED] = vertexColors[0][RED];
	flatcolor[GREEN] = vertexColors[0][GREEN];
	flatcolor[BLUE] = vertexColors[0][BLUE];

	// Apply transformation to get the vertices in screen space
	TransformCoord(vertexPositionPointer[0], Ximage[matlevel]);
	TransformCoord(vertexPositionPointer[1], Ximage[matlevel]);
	TransformCoord(vertexPositionPointer[2], Ximage[matlevel]);

	// Apply AA offsets
	for (int vertIt = 0; vertIt < 3; vertIt++)
	{
		vertexPositionPointer[vertIt][X] -= xOffset;
		vertexPositionPointer[vertIt][Y] -= yOffset;
	}

	// Skip triangles that have a negative z value for a vertex
	if (vertexPositionPointer[0][2] < 0 || vertexPositionPointer[1][2] < 0 || vertexPositionPointer[2][2] < 0)
		return GZ_SUCCESS;


#pragma region Getting vertex uv coordinates into perspective correct UV space
	float vertParameterVzPrime[3] = { 0,0,0 };	// based on z value at each vertex
	GzIntensity zs[3] = { ctoi(vertexPositionPointer[0][Z]), ctoi(vertexPositionPointer[1][Z]), ctoi(vertexPositionPointer[2][Z]) };
	vertParameterVzPrime[0] = vertexPositionPointer[0][Z] / (MAXINT - vertexPositionPointer[0][Z]);
	vertParameterVzPrime[1] = vertexPositionPointer[1][Z] / (MAXINT - vertexPositionPointer[1][Z]);
	vertParameterVzPrime[2] = vertexPositionPointer[2][Z] / (MAXINT - vertexPositionPointer[2][Z]);

	//vertParameterVzPrime[0] = zs[0] / (MAXINT - zs[0]);
	//vertParameterVzPrime[1] = zs[1] / (MAXINT - zs[1]);
	//vertParameterVzPrime[2] = zs[2] / (MAXINT - zs[2]);

	vertexUVPointer[0][U] = vertexUVPointer[0][U] / (vertParameterVzPrime[0] + 1);
	vertexUVPointer[0][V] = vertexUVPointer[0][V] / (vertParameterVzPrime[0] + 1);

	vertexUVPointer[1][U] = vertexUVPointer[1][U] / (vertParameterVzPrime[1] + 1);
	vertexUVPointer[1][V] = vertexUVPointer[1][V] / (vertParameterVzPrime[1] + 1);

	vertexUVPointer[2][U] = vertexUVPointer[2][U] / (vertParameterVzPrime[2] + 1);
	vertexUVPointer[2][V] = vertexUVPointer[2][V] / (vertParameterVzPrime[2] + 1);

	// The vertex u,v parameters are now in perspective space and ready to be used to call on the texture function on,
	// based on the type of shading we are using, gouraud or phong

#pragma endregion

#pragma region Lighting, color computations at vertices

	// Get vectors N into image( camera ) space
	TransformCoord(vertexNormalPointer[0], Xnorm[matlevel]);
	TransformCoord(vertexNormalPointer[1], Xnorm[matlevel]);
	TransformCoord(vertexNormalPointer[2], Xnorm[matlevel]);
	GzCoord E = { 0, 0, -1 };

	// iterate through all the vertices of the triangle
	for (int vert = 0; vert < 3; ++vert)
	{
		GzCoord N = { vertexNormalPointer[vert][X], vertexNormalPointer[vert][Y], vertexNormalPointer[vert][Z] };
		GzColor specSum = { 0, 0, 0 }, diffSum = { 0, 0, 0 };
		// iterate through all the vertices
		for (int lt = 0; lt < numlights; ++lt)
		{
			GzColor le = { lights[lt].color[RED], lights[lt].color[GREEN], lights[lt].color[BLUE] };
			GzCoord L = { lights[lt].direction[X], lights[lt].direction[Y], lights[lt].direction[Z] };

			float NdotL = Vec3Dot(N, L);
			float NdotE = Vec3Dot(N, E);
			// N, E, R relative orientation checks
			if (NdotE < 0 && NdotL < 0)
			{
				// flip the normal, set to -N
				N[0] = (-1)*N[0];
				N[1] = (-1)*N[1];
				N[2] = (-1)*N[2];

				NdotL = Vec3Dot(N, L);
				NdotE = Vec3Dot(N, E);
			}
			else if (NdotL * NdotE < 0)
			{
				// Skip this light
				continue;
			}

			// R = 2(N.L)N - L
			GzCoord R = { 2 * NdotL*N[X], 2 * NdotL*N[Y], 2 * NdotL*N[Z] };
			CoordDiff(R, L);
			float RdotE = Vec3Dot(R, E);

			//Red
			specSum[RED] += (le[RED] * pow(RdotE, spec));
			diffSum[RED] += (le[RED] * NdotL);

			//Green
			specSum[GREEN] += (le[GREEN] * pow(RdotE, spec));
			diffSum[GREEN] += (le[GREEN] * NdotL);

			//Blue
			specSum[BLUE] += (le[BLUE] * pow(RdotE, spec));
			diffSum[BLUE] += (le[BLUE] * NdotL);

		}
		if (!tex_fun)
		{
			vertexColors[vert][RED] = (Ks[RED] * specSum[RED] + Kd[RED] * diffSum[RED] + Ka[RED] * ambientlight.color[RED]);
			vertexColors[vert][GREEN] = (Ks[GREEN] * specSum[GREEN] + Kd[GREEN] * diffSum[GREEN] + Ka[GREEN] * ambientlight.color[GREEN]);
			vertexColors[vert][BLUE] = (Ks[BLUE] * specSum[BLUE] + Kd[BLUE] * diffSum[BLUE] + Ka[BLUE] * ambientlight.color[BLUE]);
		}
		else
		{
			vertexColors[vert][RED] = (specSum[RED] + diffSum[RED] + ambientlight.color[RED]);
			vertexColors[vert][GREEN] = (specSum[GREEN] + diffSum[GREEN] + ambientlight.color[GREEN]);
			vertexColors[vert][BLUE] = (specSum[BLUE] + diffSum[BLUE] + ambientlight.color[BLUE]);
		}

		clampvalue<float>(vertexColors[vert][RED], 0, 1);
		clampvalue<float>(vertexColors[vert][GREEN], 0, 1);
		clampvalue<float>(vertexColors[vert][BLUE], 0, 1);
	}


#pragma endregion


	// Sort the vertices
	SortVertices(vertexPositionPointer, VertSortOrder::GZ_CLOCKWISE, vertexColors, vertexNormalPointer, vertexUVPointer);

	// Drawing the triangle
	// LEE: Linear Expression Evaluation
	// Generate bounding box for the triangle
	float xmin = min(vertexPositionPointer[0][0], min(vertexPositionPointer[1][0], vertexPositionPointer[2][0]));
	float xmax = max(vertexPositionPointer[0][0], max(vertexPositionPointer[1][0], vertexPositionPointer[2][0]));
	float ymin = min(vertexPositionPointer[0][1], min(vertexPositionPointer[1][1], vertexPositionPointer[2][1]));
	float ymax = max(vertexPositionPointer[0][1], max(vertexPositionPointer[1][1], vertexPositionPointer[2][1]));

	// Generate line equations
	float lineEquationParameters[3][3];
	GenerateLineEquationParameters(vertexPositionPointer[0], vertexPositionPointer[1], lineEquationParameters[0]);
	GenerateLineEquationParameters(vertexPositionPointer[1], vertexPositionPointer[2], lineEquationParameters[1]);
	GenerateLineEquationParameters(vertexPositionPointer[2], vertexPositionPointer[0], lineEquationParameters[2]);

	// Z interpolation parameters
	float zInterpParameters[4]; 
	GenerateInterPolationParameters(vertexPositionPointer, zInterpParameters);

#pragma region Generate UV interpolation parameters

	GzCoord uVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexUVPointer[0][U] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexUVPointer[1][U] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexUVPointer[2][U] },
	};
	float uInterpParameters[4];
	GenerateInterPolationParameters(uVerts, uInterpParameters);

	GzCoord vVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexUVPointer[0][V] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexUVPointer[1][V] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexUVPointer[2][V] },
	};
	float vInterpParameters[4];
	GenerateInterPolationParameters(vVerts, vInterpParameters);

#pragma endregion

#pragma region Generate color interpolation parameters

	GzCoord redVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexColors[0][RED] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexColors[1][RED] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexColors[2][RED] },
	};
	float redInterpolationParameters[4];
	GenerateInterPolationParameters(redVerts, redInterpolationParameters);

	GzCoord greenVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexColors[0][GREEN] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexColors[1][GREEN] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexColors[2][GREEN] },
	};
	float greenInterpolationParameters[4];
	GenerateInterPolationParameters(greenVerts, greenInterpolationParameters);

	GzCoord blueVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexColors[0][BLUE] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexColors[1][BLUE] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexColors[2][BLUE] },
	};
	float blueInterpolationParameters[4];
	GenerateInterPolationParameters(blueVerts, blueInterpolationParameters);

#pragma endregion

#pragma region Generate normal interpolation parameters

	GzCoord nxVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexNormalPointer[0][X] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexNormalPointer[1][X] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexNormalPointer[2][X] },
	};
	float nxInterpolationParameters[4];
	GenerateInterPolationParameters(nxVerts, nxInterpolationParameters);

	GzCoord nyVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexNormalPointer[0][Y] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexNormalPointer[1][Y] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexNormalPointer[2][Y] },
	};
	float nyInterpolationParameters[4];
	GenerateInterPolationParameters(nyVerts, nyInterpolationParameters);

	GzCoord nzVerts[3] =
	{
		{ vertexPositionPointer[0][X], vertexPositionPointer[0][Y], vertexNormalPointer[0][Z] },
		{ vertexPositionPointer[1][X], vertexPositionPointer[1][Y], vertexNormalPointer[1][Z] },
		{ vertexPositionPointer[2][X], vertexPositionPointer[2][Y], vertexNormalPointer[2][Z] },
	};
	float nzInterpolationParameters[4];
	GenerateInterPolationParameters(nzVerts, nzInterpolationParameters);


#pragma endregion

	// Traverse through the pixels in the bounding box
	for (int yiterator = floor(ymin); yiterator <= ceil(ymax); ++yiterator)
	{
		for (int xiterator = floor(xmin); xiterator <= ceil(xmax); ++xiterator)
		{
			// is pixel inside triangle
			bool isintri = false;
			float l1v = lineEquationParameters[0][0] * xiterator + lineEquationParameters[0][1] * yiterator + lineEquationParameters[0][2];
			float l2v = lineEquationParameters[1][0] * xiterator + lineEquationParameters[1][1] * yiterator + lineEquationParameters[1][2];
			float l3v = lineEquationParameters[2][0] * xiterator + lineEquationParameters[2][1] * yiterator + lineEquationParameters[2][2];
			if ((l1v >= 0 && l2v >= 0 && l3v > 0) || (l1v <= 0 && l2v <= 0 && l3v < 0))
				isintri = true;

			// compute z value at pixel, perspective z interpolation
			if (isintri)
			{
				float newZ = GetInterpolatedValue(xiterator, yiterator, zInterpParameters);
				GzColor pixelColor;

				GzTextureIndex UV;	// perspective interpolation
				UV[U] = GetInterpolatedValue(xiterator, yiterator, uInterpParameters);	
				UV[V] = GetInterpolatedValue(xiterator, yiterator, vInterpParameters);

				// Get uv back into affine space for use in lighting color computation
				float vertVzPrime = newZ / (MAXINT - newZ);
				UV[U] = UV[U] * (vertVzPrime + 1);
				UV[V] = UV[V] * (vertVzPrime + 1);

				// Obtained interpolated texture coordinates in affine space, use to do color lookup and then use in the lighting computation
				GzColor texColor;
				if(tex_fun)
					tex_fun(UV[U], UV[V], texColor);

				if (interp_mode == GZ_COLOR)
				{
					// compute interpolated color value because of the lights, no material ka parameters used
					pixelColor[RED] = GetInterpolatedValue(xiterator, yiterator, redInterpolationParameters);
					pixelColor[GREEN] = GetInterpolatedValue(xiterator, yiterator, greenInterpolationParameters);
					pixelColor[BLUE] = GetInterpolatedValue(xiterator, yiterator, blueInterpolationParameters);

					if (tex_fun)
					{
						pixelColor[RED] *= texColor[RED];
						pixelColor[GREEN] *= texColor[GREEN];
						pixelColor[BLUE] *= texColor[BLUE];
					}
				}
				else if (interp_mode == GZ_NORMAL)
				{
					// compute interpolated normal value
					GzCoord vertexNormal;
					vertexNormal[X] = GetInterpolatedValue(xiterator, yiterator, nxInterpolationParameters);
					vertexNormal[Y] = GetInterpolatedValue(xiterator, yiterator, nyInterpolationParameters);
					vertexNormal[Z] = GetInterpolatedValue(xiterator, yiterator, nzInterpolationParameters);
					NormalizeVector(vertexNormal);

#pragma region Compute color based on normal and lights
					GzCoord E = { 0, 0, -1 };
					GzColor specSum = { 0, 0, 0 }, diffSum = { 0, 0, 0 };
					// iterate through all the lights
					for (int lt = 0; lt < 3; ++lt)
					{
						GzColor le = { lights[lt].color[0], lights[lt].color[1], lights[lt].color[2] };
						GzCoord L = { lights[lt].direction[0], lights[lt].direction[1], lights[lt].direction[2] };

						float NdotL = Vec3Dot(vertexNormal, L);
						float NdotE = Vec3Dot(vertexNormal, E);

						// N, E, R relative orientation checks
						if (NdotE < 0 && NdotL < 0)
						{
							// flip the normal, set to -N
							vertexNormal[0] = (-1)*vertexNormal[0];
							vertexNormal[1] = (-1)*vertexNormal[1];
							vertexNormal[2] = (-1)*vertexNormal[2];

							NdotL = Vec3Dot(vertexNormal, L);
							NdotE = Vec3Dot(vertexNormal, E);
						}
						else if (NdotL * NdotE < 0)
						{
							// Skip this light
							continue;
						}

						// R = 2(N.L)N - L
						GzCoord R = { 2 * NdotL*vertexNormal[0], 2 * NdotL*vertexNormal[1], 2 * NdotL*vertexNormal[2] };
						CoordDiff(R, L);
						float RdotE = Vec3Dot(R, E);

						//Red
						specSum[RED] += (le[RED] * pow(RdotE, spec));
						diffSum[RED] += (le[RED] * NdotL);

						//Green
						specSum[GREEN] += (le[GREEN] * pow(RdotE, spec));
						diffSum[GREEN] += (le[GREEN] * NdotL);

						//Blue
						specSum[BLUE] += (le[BLUE] * pow(RdotE, spec));
						diffSum[BLUE] += (le[BLUE] * NdotL);

					}
					if (!tex_fun)
					{
						pixelColor[RED] = (Ks[RED] * specSum[RED] + Kd[RED] * diffSum[RED] + Ka[RED] * ambientlight.color[RED]);
						pixelColor[GREEN] = (Ks[GREEN] * specSum[GREEN] + Kd[GREEN] * diffSum[GREEN] + Ka[GREEN] * ambientlight.color[GREEN]);
						pixelColor[BLUE] = (Ks[BLUE] * specSum[BLUE] + Kd[BLUE] * diffSum[BLUE] + Ka[BLUE] * ambientlight.color[BLUE]);
					}
					else
					{
						pixelColor[RED] = (Ks[RED] * specSum[RED] + texColor[RED] * diffSum[RED] + texColor[RED] * ambientlight.color[RED]);
						pixelColor[GREEN] = (Ks[GREEN] * specSum[GREEN] + texColor[GREEN] * diffSum[GREEN] + texColor[GREEN] * ambientlight.color[GREEN]);
						pixelColor[BLUE] = (Ks[BLUE] * specSum[BLUE] + texColor[BLUE] * diffSum[BLUE] + texColor[BLUE] * ambientlight.color[BLUE]);

					}
					clampvalue<float>(pixelColor[RED], 0, 1);
					clampvalue<float>(pixelColor[GREEN], 0, 1);
					clampvalue<float>(pixelColor[BLUE], 0, 1);
#pragma endregion
				}
				else if (interp_mode == GZ_FLAT)
				{
					// flat shading
					pixelColor[RED] = flatcolor[RED];
					pixelColor[BLUE] = flatcolor[BLUE];
					pixelColor[GREEN] = flatcolor[GREEN];
				}
				else
				{
					return GZ_SUCCESS;
				}

				GzIntensity dontNeedRGB;
				GzDepth presentZvalue;
				GzGet(xiterator, yiterator, &dontNeedRGB, &dontNeedRGB, &dontNeedRGB, &dontNeedRGB, &presentZvalue);
				int maxint = MAXINT;
				if (newZ < presentZvalue)	// draw pixels closest to the screen
				{
					clampvalue<float>(pixelColor[RED], 0.0f, 1.0f);
					clampvalue<float>(pixelColor[GREEN], 0.0f, 1.0f);
					clampvalue<float>(pixelColor[BLUE], 0.0f, 1.0f);
					GzPut(xiterator, yiterator, ctoi(pixelColor[RED]), ctoi(pixelColor[GREEN]), ctoi(pixelColor[BLUE]), MAXINT, newZ);
				}
			}
		}
	}

	return GZ_SUCCESS;
}

