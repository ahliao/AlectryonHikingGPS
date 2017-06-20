// Program to map an image to lat and long

#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>

using namespace std;

struct GeoPoint 
{
	double latitude;
	double longitude;
	int imgX;
	int imgY;
};

int main() 
{
	const int WIDTH = 2560;
	const int HEIGHT = 1643;

	GeoPoint p1, p2;
	p2.latitude = 35.190435;
	p2.longitude = -106.496140;
	p2.imgX = 819;
	p2.imgY = 1422;

	p1.latitude = 35.086497;
	p1.longitude = -106.380553;
	p1.imgX = 2212;
	p1.imgY = 155;

	// Calculate the scales
	double diffX = abs(p2.imgX - p1.imgX);
	double diffY = abs(p2.imgY - p1.imgY);
	double diffLat = abs(p2.latitude - p1.latitude);
	double diffLong = abs(p2.longitude - p1.longitude);

	// Put the abs here
	float scaleX = diffLat / diffX; // Lat per pixel
	float scaleY = diffLong / diffY; // Long per pixel

	cout << "X Scale " << setprecision(10) << scaleX << endl;
	cout << "Y Scale " << setprecision(10) << scaleY << endl;

	// Give the (0,0) bottom-left pixel and the scale
	// NOTE: BMP is ordered as bottom to top
	float x0 = p1.latitude + p1.imgX * scaleX;
	float y0 = p1.longitude - (HEIGHT - p1.imgY) * scaleY;

	cout << "(0,0) = " << "(" << x0 << "," << y0 << ")" << endl;
	cout << "Size of double " << sizeof(float) << endl;

	// Each image should have the (0,0) point and the scales (4 floats)
	ofstream file;
	file.open("Sandia.trail", ios::out | ios::binary);
	file.write((char*)&x0, sizeof(x0));
	file.write((char*)&y0, sizeof(y0));
	file.write((char*)&scaleX, sizeof(scaleX));
	file.write((char*)&scaleY, sizeof(scaleY));

	file.close();

	// Could insert the 4 floats (16 bytes) into the BMP
	// These 16bit BMPs can't be read normally anyways...
	// But it will make editting with GIMP more annoying

	return 0;
}
