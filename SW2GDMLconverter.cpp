// SW2GDMLconverter.cpp : Console application that converts a SolidWorks design into GDML for Geant4
//
// Outstanding issues:
// Support for multiple holes in a part is missing
//
// Rotation of parts with angular size (like disks) may be off when the
// part is composed of two faces with different sizes because output
// uses the largest sizes/angle but the z rotation may be based on the
// smaller face



#include "stdafx.h"
//Import the SolidWorks type library
#import "sldworks.tlb" raw_interfaces_only, raw_native_types, no_namespace, named_guids 

//Import the SolidWorks constant type library    
#import "swconst.tlb"  raw_interfaces_only, raw_native_types, no_namespace, named_guids  

#import "swcommands.tlb"  


#include "atlbase.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace std;

double Units_mm = 1000;

double units = Units_mm;

enum surfTypeIDs {
	PLANE_ID = 4001, CYLINDER_ID, CONE_ID, SPHERE_ID, TORUS_ID, BSURF_ID, BLEND_ID,
	OFFSET_ID, EXTRUSION_ID, S_REVOLVE_ID, VOLUME_ID, POSITION_ID, ROTATION_ID, SUBTRACTION_ID, DISK_ID, BOARD_ID
};

const char *const surftypes[] = { "plane", "cylinder", "cone", "sphere", "torus", "bsurf", "blend", "offset", "extrusion",
"s-revolve", "vol", "pos", "rot", "subt", "disk", "board" };

static const double MIN_THICKNESS = 0.0005;
static const double MIN_ABS_DIFF = 0.0005;

typedef struct funcpair {
	HRESULT(*boolFunc)(VARIANT_BOOL *arg);
	HRESULT(*getParams)(VARIANT *arg);
} surfFuncs;

static bool approxEqual(double val1, double val2) {
	double diff = fabs(val1 - val2);
	double avg = (fabs(val1) + fabs(val2)) / 2.0;
	// Allow relative differences of 9%.
	return (diff < MIN_ABS_DIFF || (diff < 0.4 && (avg == 0 || diff / avg < 0.09)));
}

static double checkZero(double val, double precision = 0.0000001) {
	if (val != 0.0 && fabs(val) < precision)
		val = 0.0;
	return (val);
}

// Greater than & less than for doubles that are close
static bool dblgt(double val1, double val2) {
	if (checkZero(val1 - val2) == 0.0)
		return (false);
	return (val1 > val2);
}


static bool dbllt(double val1, double val2) {
	if (checkZero(val1 - val2) == 0.0)
		return (false);
	return (val1 < val2);
}


class coords {
public:
	coords(double xval = 0.0, double yval = 0.0, double zval = 0.0, bool doNormalize = false) : 
		x(xval), y(yval), z(zval)
	{
		x = checkZero(x);
		y = checkZero(y);
		z = checkZero(z);
		if (doNormalize)
			normalize();
	}

	coords(const coords &operand) {
		x = checkZero(operand.x);
		y = checkZero(operand.y);
		z = checkZero(operand.z);
	}

	double x, y, z;
	double length() const
	{
		return (sqrt(x*x + y*y + z*z));
	}

	void normalize() {
		double chkLen = length();
		if (chkLen != 1.0 && chkLen != 0) {
			x = (x / chkLen);
			y = (y / chkLen);
			z = (z / chkLen);
		}
	}

	coords operator-(const coords &operand) const
	{
		coords difference;
		difference.x = x - operand.x;
		if (fabs(difference.x) < MIN_ABS_DIFF)
			difference.x = 0.0;
		difference.y = y - operand.y;
		if (fabs(difference.y) < MIN_ABS_DIFF)
			difference.y = 0.0;
		difference.z = z - operand.z;
		if (fabs(difference.z) < MIN_ABS_DIFF)
			difference.z = 0.0;
		return (difference);
	}

	coords operator+(const coords &operand) const
	{
		coords sum;
		sum.x = x + operand.x;
		if (fabs(sum.x) < MIN_ABS_DIFF)
			sum.x = 0.0;
		sum.y = y + operand.y;
		if (fabs(sum.y) < MIN_ABS_DIFF)
			sum.y = 0.0;
		sum.z = z + operand.z;
		if (fabs(sum.z) < MIN_ABS_DIFF)
			sum.z = 0.0;
		return (sum);
	}

	bool operator==(const coords &operand) const
	{
		return (approxEqual(x, operand.x) &&
			approxEqual(y, operand.y) &&
			approxEqual(z, operand.z));
	}

	coords operator*(double factor) {
		coords product;
		product.x = x * factor;
		product.y = y * factor;
		product.z = z * factor;
		return (product);
	}

	coords& operator=(const coords &operand) {
		x = checkZero(operand.x);
		y = checkZero(operand.y);
		z = checkZero(operand.z);
		return (*this);
	}

};


ostream & operator<<(ostream &os, const coords &coordval) {
	os << "(" << coordval.x << ", " << coordval.y << ", " << coordval.z << ") ";
	return (os);
}


struct coordHash {
	size_t operator() (const coords &param) const {
		std::stringstream strval;
		strval << param;
		std::hash<string> hashfn;
		size_t retval = hashfn(strval.str());
		// cout << "Coords " << param << " hash " << retval << endl;
		return (retval);
	}
};

struct coordCmp {
	bool operator() (const coords &first, const coords &second) const {
		return (first == second);
	}
};

typedef unordered_map<coords, int, coordHash, coordCmp> coordList;


double dotProd(const coords &fac1, const coords &fac2) {
	double xpd = fac1.x * fac2.x;
	double ypd = fac1.y * fac2.y;
	double zpd = fac1.z * fac2.z;
	return (xpd + ypd + zpd);
}

coords crossProd(const coords &fac1, const coords &fac2) {
	coords prod;
	prod.x = (fac1.y * fac2.z) - (fac1.z * fac2.y);
	prod.y = (fac1.z * fac2.x) - (fac1.x * fac2.z);
	prod.z = (fac1.x * fac2.y) - (fac1.y * fac2.x);
	return (prod);
}


class matrix3x3 {
public:
	matrix3x3() {}

	void setRow(const coords &rowVal) {
		if (matrix2d.size() < 3)
			matrix2d.push_back(rowVal);
	}

	coords operator*(const coords &val) {
		coords product;
		product.x = dotProd(matrix2d[0], val);
		product.y = dotProd(matrix2d[1], val);
		product.z = dotProd(matrix2d[2], val);
		return (product);
	}

	vector<coords> matrix2d;
};


typedef struct boxSide {
	coords point;
	coords lineDir;
	double length;
} sideInfo;


typedef struct parts {
	int surfInd;
	string nameID, volumeName;
	bool createVol;
	string matNameStr;
	bool inBoolSolid;
} partDesc;


class AssemblyInfo;
class CylSurf;
class ConeSurf;
class DiskSurf;
class BoardSurf;
class TorusSurf;
class EllipsoidSurf;

class GenericSurf {
public:
	GenericSurf(long surfIDVal, ISurface *swSurf) :
		radiusTmp(0), areaTmp(0), diameterTmp(0), perimeterTmp(0),
		angleTmp(2.0 * M_PI), lengthTmp(0), surfID(surfIDVal), surfPtr(swSurf),
		wasOutput(false), subType(BOARD_ID) // Not used for non-plane surfaces
	{}

	virtual HRESULT boolFunc(VARIANT_BOOL *arg, ISurface *surfPtr) = 0;
	virtual HRESULT getParams(VARIANT *arg, ISurface *surfPtr) = 0;

	virtual double size() = 0;

	virtual CylSurf *cylPtr() {
		cout << "Bad call -- CylSurf should override\n";
		return (NULL);
	}

	virtual ConeSurf *conePtr() {
		return (NULL);
	}
	virtual TorusSurf *torusPtr() {
		return (NULL);
	}
	virtual DiskSurf *diskPtr() {
		return (NULL);
	}
	virtual BoardSurf *boardPtr() {
		return (NULL);
	}
	virtual EllipsoidSurf *ellipsoidPtr() {
		return (NULL);
	}

	virtual bool setSize(double radius, double area, double diameter, double perimeter, double angle, double length) {
		radiusTmp = radius; // Save these values just in case
		areaTmp = area;
		diameterTmp = diameter;
		perimeterTmp = perimeter;
		angleTmp = angle;
		lengthTmp = length;
		return (false);
	}
	virtual double getAngle()
	{
		return (angleTmp);
	}

	virtual double holeSize() // If surface is a hole
	{
		return (0.0);
	}

	double radiusTmp, areaTmp, diameterTmp, perimeterTmp, angleTmp, lengthTmp; // Just for temporary holding of values.

	const char *surfName();
	double showSurfParams(AssemblyInfo *assembly);

	long surfID;
	bool wasOutput;
	long partInd;
	vector<sideInfo> sideList;
	string matNameStr, compNameStr, pathNameStr, featureTypeStr;


	virtual bool sameBody(GenericSurf *second, bool unused = false)
	{
		// Check same position, same axis, and same component
		if (second != NULL && position == second->position && axis == second->axis &&
				compNameStr.compare(second->compNameStr) == 0)
			return (true);
		return (false);
	}

	void setSubType(const surfTypeIDs newType) {
		subType = newType;
	}

	surfTypeIDs getSubType() const {
		return (subType);
	}

	coords rotation, position, axis, startAxis, endAxis;
	// startAxis is where object starts, corresponding to the x-axis
	vector<coords> axisPts, axisList;
	// axisPts are points used to help find the axis
	// axisList area axis values used to help find the axis
	coords displace;
	coords overallRot;
	IFace2 *facePtr;

protected:
	ISurface *surfPtr;
	surfTypeIDs subType;
};

class ConeSurf : public GenericSurf {
public:
	ConeSurf(long surfIDVal, ISurface *swSurf) : GenericSurf(surfIDVal, swSurf), smRadius(0.0), lgRadius(0.0), height(0.0)
	{}

	virtual HRESULT boolFunc(VARIANT_BOOL *arg, ISurface *surfPtr) {
		return (surfPtr->IsCone(arg));
	}
	virtual HRESULT getParams(VARIANT *arg, ISurface *surfPtr) {
		return (surfPtr->get_ConeParams(arg));
	}

	virtual bool setSize(double radius, double area, double diam, double perimeter, double unused2, double heightVal)
	{
		// lgRadius = smRadius = height = 0.0;
		if (radius <= 0 && diam > 0)
			radius = diam / 2.0;
		cout << "Cone set size radius smRadius " << radius << " " << smRadius << endl;
		if (radius >= 0) {
			if (smRadius <= 0)
				smRadius = radius;
			if (perimeter > 0) {
				double radTmp = (perimeter / (2.0 * M_PI)) - smRadius;
				if (radTmp > 0.0) {
					lgRadius = radTmp;
					if (lgRadius < smRadius) {
						lgRadius = smRadius;
						smRadius = radTmp; // Make sure "smRadius" is the smaller value
					}
					if (height <= 0 && lgRadius > 0 && area > 0 && smRadius >= 0) {
						double ht = pow(area / (M_PI * (lgRadius + smRadius)), 2.0) - pow(lgRadius - smRadius, 2.0);
						if (ht >= 0.0) {
							height = sqrt(ht);
						}
					}
				}
			}
		}
		if (radius == -1.0 && heightVal > 0 && area <= 0 && perimeter <= 0 && diam <= 0) // Override height
			height = heightVal;
		if (height > 0)
			cout << "Cone height is currently = " << height << endl;
		return (lgRadius > 0 && height > 0);
	}

	virtual double size()
	{
		return (height);
	}

	virtual ConeSurf *conePtr() {
		return (this);
	}

	virtual bool sameBody(GenericSurf *second, bool unused = false) {
		if (GenericSurf::sameBody(second)) {
			ConeSurf *secondCone = second->conePtr();
			if (secondCone != NULL) {
				cout << "smRadii " << smRadius << " " << secondCone->smRadius << endl;
				cout << "lgRadii " << lgRadius << " " << secondCone->lgRadius << endl;
				cout << "height " << height << " " << secondCone->height << endl;
				return (approxEqual(smRadius, secondCone->smRadius) &&
					approxEqual(lgRadius, secondCone->lgRadius) &&
					approxEqual(height, secondCone->height));
			}
			else cout << "cone ptr null\n";
		}
		return (false);
	}

	double smRadius, lgRadius, height;
};

class CylSurf : public GenericSurf {
public:
	CylSurf(long surfIDVal, ISurface *swSurf) : GenericSurf(surfIDVal, swSurf), radius(0.0), length(0.0), angle(2.0 * M_PI)
	{}

	virtual HRESULT boolFunc(VARIANT_BOOL *arg, ISurface *surfPtr) {
		return (surfPtr->IsCylinder(arg));
	}
	virtual HRESULT getParams(VARIANT *arg, ISurface *surfPtr) {
		return (surfPtr->get_CylinderParams(arg));
	}

	virtual bool setSize(double rad, double area, double diameter, double perimeter, double angleVal, double lengthVal)
	{
		if (angleVal > 0.0) {
			if (angle != 2.0 * M_PI) {
				if (angleVal != angle)
					cout << "ERROR: inconsistent angle values! Old = " << angle << " new = " << angleVal << endl;
			}
			else angle = angleVal;
		}
		if (rad > radius)
			radius = rad;

		// Don't use lengthVal if it is circumference
		if (lengthVal > length && approxEqual(lengthVal, diameter / M_PI) == false)
			length = lengthVal;
		else if (area > 0 && diameter > 0.0) {
			 double newLength = area / (M_PI * diameter);
			 if (newLength > 0.0)
				length = newLength;
		}
		return (radius > 0 && length >= 0.001 && angle > 0); // Omit tiny cylinders
	}

	virtual double getAngle()
	{
		return (angle);
	}

	virtual double size()
	{
		return (length);
	}

	virtual double holeSize() // If cylinder is a hole, use diameter
	{
		return (2.0 * radius);
	}

	double radius, length, angle;

	// friend const char *outputCylDesc(const CylSurf *const surf1, const CylSurf *const surf2);

	virtual CylSurf *cylPtr() {
		return (this);
	}

	virtual bool sameBodyLoose(CylSurf *secondCyl) {
		if (secondCyl != NULL)
			return (approxEqual(angle, secondCyl->angle) &&
			approxEqual(length, secondCyl->length));	// Allow differing radii
		return (false);
	}

	virtual bool sameBody(GenericSurf *second, bool loose = false) {
		if (GenericSurf::sameBody(second, loose)) {
			CylSurf *secondCyl = second->cylPtr();
			if (secondCyl != NULL)
				return (sameBodyLoose(secondCyl) && (loose || approxEqual(radius, secondCyl->radius) &&
				approxEqual(angle * radius, secondCyl->angle * secondCyl->radius)));
		}
		return (false);
	}
};

class TorusSurf : public GenericSurf {
public:
	TorusSurf(long surfIDVal, ISurface *swSurf) : GenericSurf(surfIDVal, swSurf), 
		smRadius(0.0), lgRadius(0.0), majorRadius(0.0), angle(2.0 * M_PI)
	{}

	virtual HRESULT boolFunc(VARIANT_BOOL *arg, ISurface *surfPtr) {
		return (surfPtr->IsTorus(arg));
	}
	virtual HRESULT getParams(VARIANT *arg, ISurface *surfPtr) {
		return (surfPtr->get_TorusParams(arg));
	}

	virtual bool setSize(double majorRadiusVal, double area, double minorDiam, double perimeter, double angleVal, double heightVal)
	{
		// lgRadius = smRadius = height = 0.0;
		// cout << "Torus set size radius smRadius " << radius << " " << smRadius << endl;
		if (majorRadiusVal > 0 && minorDiam > 0) {
			double minorRadius = minorDiam / 2.0;
			majorRadius = majorRadiusVal;
			lgRadius = minorRadius;
		}
		double calcArea = 4.0 * M_PI * majorRadius * M_PI * lgRadius;
		if (calcArea > 0) {
			cout << "torus calc area " << calcArea << endl;
			double angleVal = 2.0 * M_PI * area / calcArea;
			if (angleVal > 0) {
				angle = angleVal;
				cout << "angle is " << angle << endl;
			}
		}
		return (lgRadius > 0 && majorRadius > 0);
	}

	virtual double size()
	{
		return (majorRadius);
	}

	virtual TorusSurf *torusPtr() {
		return (this);
	}

	virtual double getAngle()
	{
		return (angle);
	}

	virtual bool sameBody(GenericSurf *second, bool unused = false) {
		if (GenericSurf::sameBody(second)) {
			TorusSurf *secondTorus = second->torusPtr();
			if (secondTorus != NULL) {
				cout << "smRadii " << smRadius << " " << secondTorus->smRadius << endl;
				cout << "lgRadii " << lgRadius << " " << secondTorus->lgRadius << endl;
				cout << "majorRadius " << majorRadius << " " << secondTorus->majorRadius << endl;
				return (approxEqual(smRadius, secondTorus->smRadius) &&
					approxEqual(lgRadius, secondTorus->lgRadius) &&
					approxEqual(majorRadius, secondTorus->majorRadius) &&
					approxEqual(angle * majorRadius, secondTorus->angle * secondTorus->majorRadius));
			}
			else cout << "torus ptr null\n";
		}
		return (false);
	}

	double smRadius, lgRadius, majorRadius, angle;
};

class PlaneSurf : public GenericSurf {
public:
	PlaneSurf(long surfIDVal, ISurface *swSurf) : GenericSurf(surfIDVal, swSurf)
	{}

	PlaneSurf(const GenericSurf &tmp) : GenericSurf(tmp)
	{}


	virtual HRESULT boolFunc(VARIANT_BOOL *arg, ISurface *surfPtr) {
		return (surfPtr->IsPlane(arg));
	}
	virtual HRESULT getParams(VARIANT *arg, ISurface *surfPtr) {
		return (surfPtr->get_PlaneParams(arg));
	}

	virtual double size() // Default to no size (depth), but disks use diameter for size
	{
		return (0.0);
	}
};

class DiskSurf : public PlaneSurf {
public:
	DiskSurf(long surfIDVal, ISurface *swSurf) : PlaneSurf(surfIDVal, swSurf), inRadius(0.0), outRadius(0.0), angle(2.0 * M_PI)
	{
		subType = DISK_ID;
	}

	DiskSurf(const GenericSurf &tmp) : PlaneSurf(tmp), inRadius(0.0),
		outRadius(tmp.radiusTmp), angle(tmp.angleTmp)
	{
		subType = DISK_ID;
	}

	virtual bool setSize(double radius, double unused1, double unused4, double unused2, double angleVal, double unused3)
	{
		if (radius > 0) {	// Take the two biggest radii encountered for in and out radii
			if (dbllt(radius, outRadius)) {	// Smaller radii are probably holes
				if (dblgt(radius, inRadius)) {
					inRadius = radius;
				}
			}
			else {
				if (dblgt(radius, outRadius)) {
					inRadius = outRadius;
				}
				outRadius = radius;
			}
		}
		if (angleVal > 0.0) {
			if (angle != 2.0 * M_PI) {
				if (radius != 0) {
					if (approxEqual(radius * angleVal, radius * angle)) {
						if (angleVal != angle) {
							angle = (angleVal + angle) / 2.0;
						}
					}
					else cout << "ERROR: inconsistent angle values! Old = " << angle << " new = " << angleVal << endl;
				}
			}
			else angle = angleVal;
		}
		return (outRadius > 0 && angle > 0);
	}

	virtual double getAngle()
	{
		return (angle);
	}

	double inRadius, outRadius, angle;

	// friend const char *outputCylDesc(const CylSurf *const surf1, const CylSurf *const surf2);

	virtual DiskSurf *diskPtr() {
		return (this);
	}

	virtual bool sameBody(GenericSurf *second, bool unused = false) {
		if (GenericSurf::sameBody(second)) {
			DiskSurf *secondDisk = second->diskPtr();
			if (secondDisk != NULL)
				return (approxEqual(inRadius, secondDisk->inRadius) &&
				approxEqual(outRadius, secondDisk->outRadius) && approxEqual(angle * inRadius, secondDisk->angle * secondDisk->inRadius));
		}
		return (false);
	}

	virtual double size() // Use diameter for size
	{
		return (2.0 * outRadius);
	}

};

class BoardSurf : public PlaneSurf {
public:
	BoardSurf(long surfIDVal, ISurface *swSurf) : PlaneSurf(surfIDVal, swSurf), length(0.0), width(0.0), consistentVals(true)
	{}

	BoardSurf(const GenericSurf &tmp) : PlaneSurf(tmp), length(tmp.lengthTmp), width(0.0), consistentVals(true)
	{}

	virtual bool setSize(double unused5, double unused1, double unused4, double unused2, double unused3, double lengthVal)
	{
		if (lengthVal > 0) {
			if (length == 0) {
				length = lengthVal;
			}
			else if (width == 0) {
				width = lengthVal;
			}
			else if (lengthVal != length && lengthVal != width) {
				cout << "Attempt to change length/width.  New = " << lengthVal << " old l w = " << length << " " << width << endl;
				consistentVals = false;
				return (false);
			}
		}
		if (length < width) { // Switch length/width to make length the longer one
			lengthVal = length;
			length = width;
			width = lengthVal;
		}
		return (consistentVals && length > 0.001 && width > 0.001); // Eliminate mysterious tiny boards
	}

	double length, width;
	bool consistentVals; // Flag to show whether setting inconsistent values was attempted

	// friend const char *outputCylDesc(const CylSurf *const surf1, const CylSurf *const surf2);

	virtual BoardSurf *boardPtr() {
		return (this);
	}

	virtual bool sameBody(GenericSurf *second, bool unused = false) {
		if (GenericSurf::sameBody(second)) {
			BoardSurf *secondBoard = second->boardPtr();
			if (secondBoard != NULL)
				return (approxEqual(length, secondBoard->length) && approxEqual(width, secondBoard->width));
		}
		return (false);
	}
};

// test comment

class EllipsoidSurf : public GenericSurf {
	// Currently only implementing half-ellipsoid with circular face
public:
	EllipsoidSurf(long surfIDVal, ISurface *swSurf) : GenericSurf(surfIDVal, swSurf),
		ax(0.0), by(0.0), cz(0.0), zcutLow(0.0), zcutHi(0.0)
	{}

	virtual HRESULT boolFunc(VARIANT_BOOL *arg, ISurface *surfPtr) {
		return (surfPtr->IsRevolved(arg));
	}
	virtual HRESULT getParams(VARIANT *arg, ISurface *surfPtr) {
		return (surfPtr->GetRevsurfParams(arg));
	}

	virtual bool setSize(double rad, double area, double diameter, double perimeter, double unused1, double unused2)
	{
		if (rad > ax) {
			ax = by = rad;
		}
		else if (diameter > (ax * 2.0)) {
			ax = by = diameter / 2.0;
		}
		if (area > 0 && ax > 0 && by > 0) {
			// Use area of an ellipsoid, but assume input area is 1/2 full area
			cz = (3.0 * pow(area / (2.0 * M_PI), 1.6)) - pow(ax * by, 1.6);
			cz /= pow(ax, 1.6) + pow(by, 1.6);
			if (cz > 0.0)
				cz = pow(cz, 5.0 / 8.0);
			else cz = 0;
		}
		return (ax > 0 && by > 0 && cz > 0);
	}

	virtual double size()
	{
		return (2.0 * M_PI * ax);
	}

	double ax, by, cz, zcutLow, zcutHi;

	virtual EllipsoidSurf *ellipsoidPtr() {
		return (this);
	}

	virtual bool sameBody(GenericSurf *second, bool unused = false) {
		if (GenericSurf::sameBody(second)) {
			EllipsoidSurf *secondEll = second->ellipsoidPtr();
			if (secondEll != NULL)
				return (approxEqual(ax, secondEll->ax) &&
				approxEqual(by, secondEll->by) && approxEqual(cz, secondEll->cz) &&
				approxEqual(zcutLow, secondEll->zcutLow) && approxEqual(zcutHi, secondEll->zcutHi));
		}
		return (false);
	}
};

class shapeList {
public:
	vector<int> shapeInds;
	virtual  string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly) = 0;
};

class cylinderList : public shapeList {
public:
	virtual string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly);
};


class conesList : public shapeList {
public:
	virtual string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly);
};

class torusesList : public shapeList {
public:
	virtual string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly);
};

class disksList : public shapeList {
public:
	virtual string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly);
};

class boardsList : public shapeList {
public:
	virtual string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly);
};

class ellipList : public shapeList {
public:
	virtual string outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly);
};

typedef vector < pair<coords, int> > centerSurfIndArray;

template<class T>
class PatternFuncs {
public:
	virtual double getSpacing(T *patternType, AssemblyInfo &assembly) = 0;
};

template<class T>
class LinPattFuncs : public PatternFuncs<T> {
public:
	virtual double getSpacing(T *patternType, AssemblyInfo &assembly) override;
};

template<class T>
class CircPattFuncs : public PatternFuncs<T> {
public:
	virtual double getSpacing(T *patternType, AssemblyInfo &assembly) override;
};



class AssemblyInfo {
public:
	AssemblyInfo() :
		surfArrayInd(-1), after1stComp(false)
	{}

	void calcmeasure(CComPtr<IMeasure> &mymeasure, VARIANT &oneface, double radius, long surfID, double &sideLen);
	void outputParts();
	void outputSolids(shapeList *sList, bool looseMatch = false, bool singleSolids = false);
	void centerPosition();
	void getEdgeDist(IFace2 *const faceptr, CComPtr<IMeasure> &mymeasure, long surfID, double radius);
	void showFaceDetails(LPDISPATCH *srcptr, int srcindex, double *radius, long *surfID, CComPtr<IMeasure> &mymeasure);
	void outputShapeSetPart(shapeList *sList, const int ind1, const int ind2, bool averagePos = true);
	void outputHole(const long baseInd, const long holeInd);
	void breakUpFaces(shapeList *sList);
	void findHoles(shapeList *sList);
	void getLineInfo(CComPtr<ICurve> curveptr, double sideLen);
	void calcBoard();
	void getInitRot(const int index);
	coords getStartAxis(IEdge *const edgeptr, const coords &center, const coords &axis, double circRad, coords &endAxis);
	coords getbcurve(CComPtr<ICurve> curveptr);
	void getMates(IComponent2 *swSelectedComponent);
	// void showMateEntity(CComPtr<IFeature> swFeature);
	void showMate(IMate2 *matePtr, coords &antiAlignTot, int &alignCnt, coordList &displaceList,
		int mateInd, bool &adjustDisplace, int mateCnt, bool &meRadiiSame);
	void resetAxis(const coords &center, const coords &startVertex, const coords &endVertex);
	coords choseStartAxis(const coords &center, const coords &radVec, const coords &normal,
		const coords &startVertex, const coords &endVertex, coords &endAxis);
	void getSeedComps(ILocalLinearPatternFeatureData *const linpattern);
	void getRelatedComps(IComponent2 *baseComp);
	void getTransfDisplace(ILocalLinearPatternFeatureData *linpattern, double xspacing, double zspacing);
	void findTorusCenter();
	void chkEllipAxis();
	void chkPatterns(CComPtr<IFeature> swFeature, IModelDoc2* swModel, const CComBSTR &sTypeName);
	template<typename T>
	void procPattern(CComPtr<IFeature> swFeature, IModelDoc2* swModel, PatternFuncs<T> *pattfuncs);

	vector<GenericSurf *> surfArray;
	int surfArrayInd;
	bool after1stComp;
	char matNameStr[80];
	string compNameStr, pathNameStr, featureTypeStr;
	vector<coords> pattDisplaceList;

protected:
	cylinderList cylList;
	conesList coneList;
	torusesList torusList;
	disksList diskList;
	boardsList boardList;
	ellipList ellipsoidList;
	vector<partDesc> partArray;
	centerSurfIndArray edgeList;
	vector<pair<int, int>> holeList;
	coords displace;
	coords overallRot;
	string overallRotCmpNmStr;
};

VARIANT_BOOL retVal = VARIANT_FALSE;

HRESULT hres = NOERROR, hres2 = NOERROR;

static const coords zaxis(0.0, 0.0, 1.0),
	yaxis(0.0, 1.0, 0.0), xaxis(1.0, 0.0, 0.0),
	xaxisminus(-1.0, 0.0, 0.0), zaxisminus(0.0, 0.0, -1.0),
	origin(0, 0, 0);


const char *const cylname = "tube", *const conename = "cone";

//Function prototypes
void OpenAssembly(ISldWorks* swApp, IModelDoc2** swModel);
void TraverseFeatureManagerDesignTree(IModelDoc2* swModel, ISldWorks* swApp);
void CloseDocuments(ISldWorks* swApp);

static ofstream gdmlout;


class xmlElem {
public:
	xmlElem() {}

	const char *openElem(const char *label = NULL, const char *attrib = NULL, bool separate = false) {
		outputStr = "<";
		if (label != NULL) {
			outputStr += label;
		}
		if (attrib != NULL) {
			outputStr += attrib;
		}
		if (separate)
			outputStr += ">";
		return (outputStr.c_str());
	}

	const char *openSepElem(const char *label = NULL, const char *attrib = NULL) {
		return (openElem(label, attrib, true));
	}

	const char *closeElem(const char *label = NULL, bool separate = false) {
		if (separate)
			outputStr = "</";
		else outputStr = "/";
		if (label != NULL)
			outputStr += label;
		outputStr += ">";
		return (outputStr.c_str());
	}

	const char *closeSepElem(const char *label = NULL) {
		return (closeElem(label, true));
	}

	const char *attribute(const char *name, const char *value) {
		outputStr = " ";
		outputStr += name;
		outputStr += "=\"";
		outputStr += value;
		outputStr += "\"";
		return (outputStr.c_str());
	}

	const char *attribute(const char *name, double value) {
		char numstr[50];
		sprintf_s(numstr, "%g", value);
		return (attribute(name, numstr));
	}

	const char *indent(int numSpaces = 1) {
		outputStr = " ";
		for (int cnt = 1; cnt < numSpaces; ++cnt)
			outputStr += " ";
		return (outputStr.c_str());
	}
protected:
	string outputStr;
};

const char *const indent1 = " ", *const indent2 = "  ", *const indent3 = "   ";
const char *const endOpen = ">";

static bool begingdml() {
	if (CopyFileA("template.gdml", "design.gdml", false) == false) {
		std::cout << "Couldn't copy gdml template\n";
		return (false);
	}
	return (true);
}


static void printbstr(const char *textout, const CComBSTR &instr)
{
	CW2A outstr(instr);
	cout << textout << outstr << endl;
}


const char *const nameIncr(surfTypeIDs itemcode)
{
	static int itemCnt[sizeof(surftypes) / sizeof(const char *const)];
	int index = itemcode - 4001;
	static string name;
	name = surftypes[index];
	char numstr[50];
	sprintf_s(numstr, "%d", ++itemCnt[index]);
	name += numstr;
	return (name.c_str());
}


static double calcAngle(double val1, double val2) {
	double angle = 0.0;
	if (val2 != 0.0) {
		double ratio = 1.0;
		if (val1 == 0.0) {
			angle = M_PI_2;
			if (val2 < 0)
				angle = 1.5 * M_PI;
		}
		else {
			angle = atan(val2 / val1);
			ratio = val2 / val1;
		}
		// cout << "calc ratio " << ratio << " angle = " << angle << " val1 = " << val1 << " val2 = " << val2 << endl;
	}
	else if (val1 < 0)
		angle = M_PI;
	if (val1 < 0 && angle < M_PI_2 && angle > -M_PI_2)
		angle += M_PI;
	if (val2 < 0.0 && angle < M_PI && angle > 0)
		angle += M_PI; // Quadrant III or IV
	if (angle < 0.0) { // Keep angles positive
		angle = (2.0 * M_PI) + angle;
	}
	return (angle);
}

// calcCoords defaults to assumption angle is rotation of axis, not of object
static void calcCoords(double angle, double &val1, double &val2, bool rotAxis = true)
{
	if (rotAxis)
		angle = -angle;
	double newAngle = calcAngle(val1, val2) + angle;
	// cout << "rot angle " << angle << ", new angle " << newAngle << endl;

	double chkAngle = fabs(newAngle / M_PI);
	chkAngle *= 2.0;
	double lengthsqd = val1 * val1 + val2 * val2;
	// cout << "val1, val2 = " << val1 << ", " << val2 << " length2 = " << lengthsqd << " newangle = " << newAngle << endl;
	int intAngle = (int) (chkAngle + 0.2);
	if (approxEqual(chkAngle, intAngle) && (intAngle % 2) == 1) { // chkAngle is n/2
		val1 = 0.0;
		val2 = sqrt(lengthsqd);
	}
	else {
		val1 = sqrt(lengthsqd / (pow(tan(newAngle), 2.0) + 1));
		val1 = checkZero(val1);
		if (fabs(newAngle) > (M_PI / 2.0) && fabs(newAngle) < (3.0 * M_PI / 2.0))
			val1 *= -1.0;
		val2 = tan(newAngle) * val1;
		// cout << "val1, val2 = " << val1 << ", " << val2 << " length2 = " << lengthsqd << " newangle = " << newAngle << endl;
	}
	if (val2 > 0.0 && ((newAngle > M_PI && newAngle < (2.0 * M_PI)) || (newAngle < 0 && newAngle > (-M_PI))))
		val2 = -val2;
	val2 = checkZero(val2);
	// if (val2 == 0) {
		// cout << "val1, val2 = " << val1 << ", " << val2 << " length2 = " << lengthsqd << " newangle = " << newAngle << endl;
	// }
}


static double axisrot(double &start1, double &start2, double end1, double end2, bool rotAxis = true)
{
	if ((start1 == 0.0 && start2 == 0.0) || (end1 == 0.0 && end2 == 0.0))
		return (0.0);	// To/from origin, no rotation possible
	double angle1 = calcAngle(start1, start2);
	double angle2 = calcAngle(end1, end2);
	double angleDiff = angle2 - angle1;
	// cout << "axisrot angle diff " << angleDiff << endl;
	if (fabs(angleDiff) > M_PI) {
		if (angleDiff > 0.0)
			angleDiff = M_PI - angleDiff;
		else angleDiff = 2.0 * M_PI + angleDiff;
	}
	if (angleDiff != 0.0) {
		if (rotAxis)
			angleDiff = -angleDiff; // GDML rotations are rotations of axis
		calcCoords(angleDiff, start1, start2, rotAxis); // Update as if rotation occurred
	}
	return (angleDiff);
}


// rotVecZYX does rotation of axis, not object
static coords rotVecZYX(const coords &vec, const coords &angle)
{
	coords newvec(vec);
	// GDML appears to do zyx rotation order for volumes
	if (angle.z != 0.0)
		calcCoords(angle.z, newvec.x, newvec.y);
	// cout << "rotVecZYX before y rot = " << newvec << endl;
	if (angle.y != 0.0)
		calcCoords(angle.y, newvec.z, newvec.x);
	// cout << "rotVecZYX before x rot = " << newvec << endl;
	if (angle.x != 0.0)
		calcCoords(angle.x, newvec.y, newvec.z);
	// cout << "rotVecZYX after x rot = " << newvec << endl;
	return (newvec);
}


// rotVecXYZ does rotation of object, not axis
static coords rotVecXYZ(const coords &vec, const coords &angle)
{
	coords newvec(vec);
	// GDML appears to do XYZ rotation order for boolean solids
	if (angle.x != 0.0)
		calcCoords(angle.x, newvec.y, newvec.z, false);
	if (angle.y != 0.0)
		calcCoords(angle.y, newvec.z, newvec.x, false);
	if (angle.z != 0.0)
		calcCoords(angle.z, newvec.x, newvec.y, false);
	return (newvec);
}


// rotAnglesZYX does rotation of axis, not object
static coords rotAnglesZYX(coords start, const coords &end)
{
	coords angle;
	// GDML appears to do zyx rotation order for volumes
	angle.z = axisrot(start.x, start.y, end.x, end.y);
	// cout << "calc z angle " << angle.z << endl;
	double endz = 0.0;
	double zsquare = (end.y * end.y) + (end.z * end.z) - (start.y * start.y);
	zsquare = checkZero(zsquare);
	if (zsquare > 0.0) {
		endz = sqrt(zsquare);
		if (end.z < 0.0)
			endz = -endz; // Choose smaller angle to end.z
	}
	// For the y rotation, need to position vector for the final x rotation
	// In the x rotation, distance from the x axis (d^2 = y^2 + z^2) stays constant, so
	// need to find z before x rotation that will start at this constant distance,
	// so initial z^2 = d^2 - yi^2, where yi is initial y. During y rotation, yi is constant.
	// Need x/z for righthand rule
	// cout << "ZYX: y going from " << start << " to " << end << " actually endz " << endz << endl;
	angle.y = axisrot(start.z, start.x, endz, end.x);
	// cout << "ZYX: calc y angle " << angle.y << endl;
	if (start.y != end.y || start.z != end.z) {
		angle.x = axisrot(start.y, start.z, end.y, end.z);
		// cout << "ZYX: calc x angle " << angle.x << endl;
	}
	return (angle);
}


// rotAnglesXYZ does rotation of object, not axis
static coords rotAnglesXYZ(coords start, const coords &end)
{
	coords angle;
	// GDML appears to do xyz rotation order for boolean solids
	// cout << "XYZ: x going from " << start << " to " << end << endl;
	angle.x = axisrot(start.y, start.z, end.y, end.z, false);
	// cout << "XYZ: calc x angle " << angle.x << " start now " << start << endl;
	double endx = 0.0;
	double xsquare = (end.x * end.x) + (end.y * end.y) - (start.y * start.y);
	xsquare = checkZero(xsquare);
	if (xsquare > 0.0) {
		endx = sqrt(xsquare);
		if (end.x < 0.0)
			endx = -endx; // Choose smaller angle to end.x
	}
	
	// For the y rotation, need to position vector for the final z rotation
	// In the z rotation, distance from the z axis (d^2 = y^2 + x^2) stays constant, so
	// need to find x before z rotation that will start at this constant distance,
	// so initial x^2 = d^2 - yi^2, where yi is initial y. During y rotation, yi is constant.
	// Need x/z for righthand rule
	// cout << "XYZ: y going from " << start << " to " << end << "actual endx " << endx << endl;
	angle.y = axisrot(start.z, start.x, end.z, endx, false);
	// cout << "XYZ: calc y angle " << angle.y << endl;
	if (start.y != end.y || start.x != end.x) {
		// cout << "XYZ: z rot going from " << start << " to " << end << endl;
		angle.z = axisrot(start.x, start.y, end.x, end.y, false);
		// cout << "XYZ: calc z angle " << angle.z << endl;
	}
	return (angle);
}


int _tmain(int argc, _TCHAR* argv[])
{
	cout << "start up \n";
	if (begingdml() == false) {
		return (1);
	}
	gdmlout.open("design.gdml", ios_base::out | ios_base::app);
	if (gdmlout.good() == false) {
		cout << "Can't open output file\n";
		return (1);
	}
	//Initialize COM 
	CoInitialize(NULL);
	//Use ATL smart pointers
	CComPtr<ISldWorks> swApp;
	CComPtr<IModelDoc2> swModel;
	CComPtr<IAssemblyDoc> swAssemblyDoc;
	bool bDone;
	do {
		try {
			//Create an instance of SolidWorks
			HRESULT hres = swApp.CoCreateInstance(__uuidof(SldWorks), NULL, CLSCTX_LOCAL_SERVER);
			if (hres != S_OK)
				throw 0;
			bDone = true;
			IModelDoc2* swModel = NULL;
			//Open assembly
			OpenAssembly(swApp, &swModel);
			//Traverse FeatureManager design tree to get specified feature
			TraverseFeatureManagerDesignTree(swModel, swApp);
			//Close documents
			CloseDocuments(swApp);
			cout << "closed docs \n";
		}
		//Catch the exception and tell the user that SolidWorks is not running
		catch (int)
		{
			cout << "Error starting or attaching to a SolidWorks session." << endl;
			//Release COM references
			swApp = NULL;
			swModel = NULL;
			swAssemblyDoc = NULL;
			//Uninitialize COM
			CoUninitialize();
			return 1;
		}
	} while (!bDone);
	//Shut down SolidWorks 
	swApp->ExitApp();
	cout << "exit app \n";
	//Release COM reference
	swApp = NULL;
	swModel = NULL;
	swAssemblyDoc = NULL;
	//Uninitialize COM
	CoUninitialize();
	cout << "uninit \n";
	xmlElem beginEnd, attrib1, attrib2;
	gdmlout << beginEnd.openElem("setup") << attrib1.attribute("name", "Test1") << attrib2.attribute("version", "1.0");
	gdmlout << endOpen << endl;
	gdmlout << indent1 << beginEnd.openElem("world") << attrib1.attribute("ref", "World");
	gdmlout << beginEnd.closeElem() << endl;
	gdmlout << beginEnd.closeSepElem("setup") << endl;
	gdmlout << beginEnd.closeSepElem("gdml") << endl;
	// gdmlout << "<setup name=\"Test1\" version=\"1.0\">\n";	gdmlout << "</setup>\n";
	// gdmlout << "</gdml>\n";
//	gdmlout << "<setup name=\"Test1\" version=\"1.0\">\n";
//	gdmlout << " <world ref=\"World\"/>\n";
//	gdmlout << "</setup>\n";
//	gdmlout << "</gdml>\n";
	gdmlout.close();
	return 0;
}

/*
static int getFileName() {
	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED |
		COINIT_DISABLE_OLE1DDE);
	if (SUCCEEDED(hr))
	{
		IFileOpenDialog *pFileOpen;

		// Create the FileOpenDialog object.
		hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL,
			IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));

		if (SUCCEEDED(hr))
		{
			// Show the Open dialog box.
			pFileOpen->Show(0);
			hr = pFileOpen->Show(NULL);

			// Get the file name from the dialog box.
			if (SUCCEEDED(hr))
			{
				IShellItem *pItem;
				hr = pFileOpen->GetResult(&pItem);
				if (SUCCEEDED(hr))
				{
					PWSTR pszFilePath;
					hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);

					// Display the file name to the user.
					if (SUCCEEDED(hr))
					{
						MessageBox(NULL, pszFilePath, L"File Path", MB_OK);
						CoTaskMemFree(pszFilePath);
					}
					pItem->Release();
				}
			}
			pFileOpen->Release();
		}
		CoUninitialize();
	}
	return 0;
}
*/

void OpenAssembly(ISldWorks* swApp, IModelDoc2** swModel)

//Open assembly

{

	CComBSTR sFileName(L"C:\\Users\\Carl\\Documents\\solidworks\\tube\\Outer tube assy_version4_mdh_mar14.SLDASM");
	// CComBSTR sFileName2(L"C:\\Users\\Carl\\Documents\\solidworks\\tank\\tube-fitting_edin.SLDASM");
	CComBSTR sFileName2(L"C:\\Users\\Carl\\Documents\\solidworks\\tank\\LS-side-tank-1_edin.SLDASM");
	CComBSTR sFileName3(L"C:\\Users\\Carl\\Documents\\solidworks\\LZ_detector_assy_Oct14_CD1_a_20141120\\LZ_detector_assy_Oct14_CD1_a_1411.SLDASM");
	CComBSTR sFileName4(L"C:\\Users\\Carl\\Documents\\solidworks\\LZ_detector_assy_Oct14_CD1_a_20141120\\Water Tank_j_1410.SLDASM");
	CComBSTR sFileName5(L"C:\\Users\\Carl\\Documents\\solidworks\\simulation_model_a_20151204\\simulation_model_a_1512.SLDASM");
	CComBSTR sFileName6(L"C:\\Users\\Carl\\Documents\\solidworks\\lower_array_pack_and_go\\truss_lower_assy_mod_design_a_.SLDASM");
	CComBSTR sFileName7(L"C:\\Users\\Carl\\Documents\\solidworks\\lower_array_pack_and_go\\pmt_base_assy_.SLDASM");
	CComBSTR sFileName8(L"C:\\Users\\Carl\\Documents\\solidworks\\simulation_model_b_20160114\\simulation_model_b_1601.SLDASM");

	CComBSTR sDefaultConfiguration(L"Default");



	long fileerror, filewarning;



	IModelDoc2* swModelAssembly;

	OPENFILENAME ofn;
	wchar_t szFileName[MAX_PATH];

	ZeroMemory(&ofn, sizeof(ofn));
	ZeroMemory(szFileName, sizeof(szFileName));

	ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
	// ofn.hwndOwner = hwnd;
	// ofn.lpstrFilter = "Text Files (*.txt)\0*.txt\0All Files (*.*)\0*.*\0";
	ofn.lpstrFile = szFileName;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	// ofn.lpstrDefExt = "txt";

	if (GetOpenFileName(&ofn))
	{
		CComBSTR newFileName(szFileName);
		printbstr("open file ", newFileName);
		hres = swApp->OpenDoc6(newFileName, swDocASSEMBLY, swOpenDocOptions_Silent, sDefaultConfiguration, &fileerror, &filewarning, &swModelAssembly);

		*swModel = swModelAssembly; //Set the value of the input argument equal to address of the interface
	}
}


static void setPlaneSurf(GenericSurf **surf, surfTypeIDs newType) {
	GenericSurf *surfPtr = *surf;
	if (newType != surfPtr->getSubType() && surfPtr->surfID == PLANE_ID) {
		GenericSurf *tmpPtr = NULL;
		switch (newType) {
		case BOARD_ID:
			tmpPtr = new BoardSurf(*surfPtr);
			break;
		case DISK_ID:
			tmpPtr = new DiskSurf(*surfPtr);
			{ // Need block for local variable oldBoard in switch
				BoardSurf *oldBoard = surfPtr->boardPtr();
				// Transfer values from old to new
				if (tmpPtr != NULL && oldBoard != NULL) {
					double circumf = oldBoard->length;
					int cnt = 0;
					do {
						double radius = circumf / (2.0 * M_PI);
						if (radius > 0)
							(void)tmpPtr->setSize(radius, 0, 0, 0, 0, 0);
						circumf = oldBoard->width;
						++cnt;
					} while (cnt < 2);
				}
			}
			break;
		default:
			cout << "Invalid subType resetting\n";
			break;
		}
		if (tmpPtr != NULL) {
			delete surfPtr;
			*surf = tmpPtr;
		}
	}
}


void AssemblyInfo::findTorusCenter() {
	TorusSurf *theTorus = surfArray[surfArrayInd]->torusPtr();
	// Expecting two circular faces of partial torus
	if (theTorus != NULL && surfArray[surfArrayInd]->axisList.size() == 2 && surfArray[surfArrayInd]->axisPts.size() == 2) {
		coords radVec1 = crossProd(surfArray[surfArrayInd]->axis, surfArray[surfArrayInd]->axisList[0]);
		coords radVec2 = crossProd(surfArray[surfArrayInd]->axis, surfArray[surfArrayInd]->axisList[1]);
		radVec1.normalize();
		radVec2.normalize();
		if (radVec1.length() > 0 && radVec2.length() > 0) {
			vector<coords> centerCands;
			double majorRadius = theTorus->majorRadius;
			// Radius vectors are along direction between circle centers and torus center,
			// but we don't know which vector direction is correct.
			// So we construct two points on either side of the circle center.
			// The torus center will be the point that is the same for both circle centers.
			centerCands.push_back((radVec1 * majorRadius) + surfArray[surfArrayInd]->axisPts[0]);
			centerCands.push_back((radVec1 * -majorRadius) + surfArray[surfArrayInd]->axisPts[0]);
			centerCands.push_back((radVec2 * majorRadius) + surfArray[surfArrayInd]->axisPts[1]);
			centerCands.push_back((radVec2 * -majorRadius) + surfArray[surfArrayInd]->axisPts[1]);
			coords normal; // Normal to axis, in plane of torus
			for (int ind = 0; ind < 2; ++ind) {
				cout << "center candidates " << ind << " and " << ind + 2 << " " << centerCands[ind] << centerCands[ind + 2] << endl;
				if (centerCands[ind] == centerCands[2] ||
					centerCands[ind] == centerCands[3]) {
					surfArray[surfArrayInd]->position = centerCands[ind];
					normal = surfArray[surfArrayInd]->axisPts[ind] - centerCands[ind];
					break;
				}
			}
			resetAxis(surfArray[surfArrayInd]->position, surfArray[surfArrayInd]->axisPts[0], surfArray[surfArrayInd]->axisPts[1]);
			// Can reverse surfArray[surfArrayInd]->axis

			surfArray[surfArrayInd]->startAxis = choseStartAxis(surfArray[surfArrayInd]->position, surfArray[surfArrayInd]->axis,
				normal, surfArray[surfArrayInd]->axisPts[0], surfArray[surfArrayInd]->axisPts[1], surfArray[surfArrayInd]->endAxis);
		}
		else {
			cout << "Bad radial vectors " << radVec1 << radVec2 << endl;
			cout << "axis " << surfArray[surfArrayInd]->axis << " circle axes " << surfArray[surfArrayInd]->axisList[0] << surfArray[surfArrayInd]->axisList[1] << endl;
		}
	}
}


void AssemblyInfo::calcmeasure(CComPtr<IMeasure> &mymeasure, VARIANT &oneface, double radius, long surfID, double &length)
{
	hres = mymeasure->Calculate(oneface, &retVal);
	if (hres == S_OK && retVal) {
		double val = 0.0, area = 0.0, diam = 0.0;
		hres = mymeasure->get_Area(&area);
		if (hres == S_OK) {
			if (hres == S_OK && area > 0)
				cout << "area [m^2] " << area << endl;
			else area = 0.0;	// get_Area failure sets area = -1
			hres = mymeasure->get_Diameter(&diam);
			if (hres == S_OK && diam > 0) {
				cout << "diam [m] " << diam << endl;
			}
			hres = mymeasure->get_Length(&length);
			if (hres == S_OK && length > 0) {
				cout << "length [m] " << length << endl;
				if (radius == 0 && surfArray[surfArrayInd]->surfID == OFFSET_ID) {
					// Assume offset length is a circumference
					radius = length / (2.0 * M_PI);
					cout << "Radius set to " << radius << endl;
				}
			}
			double arcLen = 0.0;
			hres = mymeasure->get_ArcLength(&arcLen);
			if (hres == S_OK && arcLen > 0)
				cout << "arc length [m] " << arcLen << endl;
			double angle = 0.0;
			hres = mymeasure->get_Angle(&angle);
			if (hres == S_OK && angle > 0) {
				cout << "angle [radians] " << angle << endl;
				if (radius == 0 && arcLen > 0) {
					radius = arcLen / angle;
					cout << "Radius set to " << radius << endl;
					// Circular edge indicates a disk if the surface is a plane
					if (surfArray[surfArrayInd]->surfID == PLANE_ID)
						setPlaneSurf(&(surfArray[surfArrayInd]), DISK_ID);
				}
			}
			if (area > 0 && diam > 0) {
				double cylen = area / (M_PI * diam);
				cout << "calculated cylinder length [m] " << cylen << endl;
			}
			val = 0.0;
			hres = mymeasure->get_Perimeter(&val);
			bool validSize = surfArray[surfArrayInd]->setSize(radius, area, diam, val, angle, length);
			if (validSize) {
				if (surfID == CYLINDER_ID && (cylList.shapeInds.empty() || cylList.shapeInds.back() != surfArrayInd)) {
					cylList.shapeInds.push_back(surfArrayInd); // Add to cylinder list
					cout << "Found cylinder surf index = " << std::dec << surfArrayInd << " length " << surfArray[surfArrayInd]->size() << endl;
				}
				else if (surfID == TORUS_ID) {
					if ((torusList.shapeInds.empty() || torusList.shapeInds.back() != surfArrayInd)) {
						torusList.shapeInds.push_back(surfArrayInd);
						cout << "Found torus surf index = " << std::dec << surfArrayInd << " width " << surfArray[surfArrayInd]->size() << endl;
					}
					if (surfArray[surfArrayInd]->getAngle() < 2.0 * M_PI) {
						findTorusCenter();
					}
				}
				else if (surfID == S_REVOLVE_ID) {
					if ((ellipsoidList.shapeInds.empty() || ellipsoidList.shapeInds.back() != surfArrayInd)) {
						ellipsoidList.shapeInds.push_back(surfArrayInd);
						cout << "Found ellipsoid surf index = " << std::dec << surfArrayInd << " circumference " << surfArray[surfArrayInd]->size() << endl;
						chkEllipAxis();
					}
				}
				else if (surfID == PLANE_ID) {
					switch(surfArray[surfArrayInd]->getSubType()) {
					case DISK_ID:
						if ((diskList.shapeInds.empty() || diskList.shapeInds.back() != surfArrayInd)) {
							diskList.shapeInds.push_back(surfArrayInd);
							cout << "Found disk surf index = " << std::dec << surfArrayInd << endl;
						}
						break;
					case BOARD_ID:
						if ((boardList.shapeInds.empty() || boardList.shapeInds.back() != surfArrayInd)) {
							boardList.shapeInds.push_back(surfArrayInd);
							cout << "Found board surf index = " << std::dec << surfArrayInd << endl;
						}
						break;
					default:
						cout << "Unknown sub-type\n";
					}
				}
			}
			else if (surfID == PLANE_ID && surfArray[surfArrayInd]->getSubType() == BOARD_ID &&
				boardList.shapeInds.empty() == false && boardList.shapeInds.back() == surfArrayInd) {
					boardList.shapeInds.pop_back();
					cout << "Removed surf index = " << surfArrayInd << " because it has inconsistent size" << endl;
					cout << "Board list size is now " << boardList.shapeInds.size() << endl;
					if (boardList.shapeInds.empty() == false) {
						cout << "Last board is now " << boardList.shapeInds.back() << endl;
					}
			}
			if (hres == S_OK && val > 0) {
				cout << "perimeter [m] " << val << endl;
				cout << "radius [m] " << radius << endl;
				double radius2 = 0.0;
				if (radius >= 0) {
					radius2 = (val / (2.0 * M_PI)) - radius;
					double lgRad = radius2;
					if (lgRad < radius){
						lgRad = radius;
						radius = radius2; // Make sure "radius" is the smaller value
					}
					if (lgRad > 0 && area > 0 && radius >= 0) {
						double ht = pow(area / (M_PI * (lgRad + radius)), 2.0) - pow(lgRad - radius, 2.0);
						if (ht > 0.0) {
							ht = sqrt(ht);
							cout << " cone radii and height validSize " << lgRad << " " << radius << " " << ht << " " << validSize << endl;
							if (validSize && (surfID == CONE_ID || surfID == BSURF_ID || surfID == OFFSET_ID) &&
								(coneList.shapeInds.empty() || coneList.shapeInds.back() != surfArrayInd)) {
								coneList.shapeInds.push_back(surfArrayInd); // Add to cone list
								cout << "Found cone surf index = " << surfArrayInd << endl;
							}
						}
					}
				}
			}
			hres = mymeasure->get_TotalArea(&val);
			if (hres == S_OK && val > 0)
				cout << "totarea [m^2] " << val << endl;
			hres = mymeasure->get_TotalLength(&val);
			if (hres == S_OK && val > 0)
				cout << "totlength [m] " << val << endl;
		}
	}
	else cout << "calc fail hres retval " << std::hex << hres << " " << retVal << endl;
}


static double *varDblArrayAccess(VARIANT &paramInfo, long &paramBegin, long &paramLim)
{
	if (paramInfo.pparray != NULL) {
		SAFEARRAY *psaparamInfo = V_ARRAY(&paramInfo);
		double *pParamDblArray = NULL;
		hres = SafeArrayAccessData(psaparamInfo, (void **)&pParamDblArray);
		if (hres == S_OK && pParamDblArray != NULL) {
			paramBegin = paramLim = -1;
			hres = SafeArrayGetLBound(psaparamInfo, 1, &paramBegin);
			hres = SafeArrayGetUBound(psaparamInfo, 1, &paramLim);
			if (hres == S_OK)
				return (pParamDblArray);
		}
	}
	return (NULL);
}


double GenericSurf::showSurfParams(AssemblyInfo *assembly)
{
	double radius = 0.0;
	if (boolFunc(&retVal, surfPtr) == S_OK && retVal) {
		VARIANT paramInfo;
		VariantInit(&paramInfo);
		hres = getParams(&paramInfo, surfPtr);  // variant
		if (hres == S_OK && paramInfo.pparray != NULL) {
			SAFEARRAY *psaparamInfo = V_ARRAY(&paramInfo);
			double *pParamDblArray = NULL;
			hres = SafeArrayAccessData(psaparamInfo, (void **)&pParamDblArray);
			if (hres == S_OK && pParamDblArray != NULL) {
				long paramBegin = -1, paramLim = -1;
				hres = SafeArrayGetLBound(psaparamInfo, 1, &paramBegin);
				hres = SafeArrayGetUBound(psaparamInfo, 1, &paramLim);
				if (hres == S_OK && (paramLim - paramBegin) >= 5) {
					coords normal(pParamDblArray[paramBegin], pParamDblArray[paramBegin + 1], pParamDblArray[paramBegin + 2], true);
					coords axis(pParamDblArray[paramBegin + 3], pParamDblArray[paramBegin + 4], pParamDblArray[paramBegin + 5], true);
					cout << "origin/normal " << normal << endl;
					cout << "axis/root " << axis << endl;
					// Use normal as the axis for a plane.
					if ((assembly->surfArray)[assembly->surfArrayInd]->surfID == PLANE_ID)
						(assembly->surfArray)[assembly->surfArrayInd]->axis = normal;
					else (assembly->surfArray)[assembly->surfArrayInd]->axis = axis;
					if ((paramLim - paramBegin) >= 6) {
						radius = pParamDblArray[paramBegin + 6];
						cout << "radius " << radius << endl;
					}
					if ((paramLim - paramBegin) >= 7 && (assembly->surfArray)[assembly->surfArrayInd]->surfID == TORUS_ID) {
						double minorRadius = pParamDblArray[paramBegin + 7];
						cout << "minorRadius " << minorRadius << endl;
						setSize(radius, 0, minorRadius * 2.0, 0, 0, 0);
					}
				}
			}
		}
		else cout << "Failed to get params, hres " << std::hex << hres << endl;
		VARIANT point;
		VariantInit(&point);
		hres = surfPtr->GetClosestPointOn(1.0, 1.0, 1.0, &point);
		long paramBegin, paramLim;
		double *ptarray = varDblArrayAccess(point, paramBegin, paramLim);
		if (ptarray != NULL)
			for (int index = paramBegin; index <= paramLim; ++index) {
			double val = ptarray[index];
			if (fabs(val) < 0.00001)
				val = 0.0;
			if (index == paramBegin)
				cout << "closest point value? for (1,1,1) (" << val << ", ";
			else if (index == paramBegin + 1)
				cout << val << ", ";
			else if (index == paramBegin + 2 || index == paramBegin + 4)
				cout << val << ")" << endl;
			else if (index == paramBegin + 3)
				cout << "u,v (" << val << ", ";
			}
	}
	return (radius);
}


// Only works for axis that is major axis -- don't use in general case
coords calcVecOnAxis(coords axis, coords point)
{
	double dotProdval = dotProd(axis, point);
	coords vecOnAxis = axis;
	double veclen = 0.0, axlen = axis.length();
	if (axlen > 0)
		veclen = dotProdval / axlen;
	vecOnAxis.x = vecOnAxis.x * veclen;
	vecOnAxis.y = vecOnAxis.y * veclen;
	vecOnAxis.z = vecOnAxis.z * veclen;
	return (vecOnAxis);
}


void AssemblyInfo::centerPosition() {
	// Move position from edge into center of part.
	cout << "Position before resetting " << surfArray[surfArrayInd]->position << endl;
	double length = surfArray[surfArrayInd]->axis.length();
	if (length <= 0.0) {
		coords zaxis(0.0, 0.0, 1.0);	// Use default axis
		surfArray[surfArrayInd]->axis = zaxis;
		cout << "0 axis -- replaced with z axis\n";
		length = surfArray[surfArrayInd]->axis.length();
	}
	surfArray[surfArrayInd]->axis.normalize();	// Make sure axis is unit vector
	if (length <= 0.0)
		length = 1.0;	// Protect against unlikely occurrence
	cout << "surf size = " << surfArray[surfArrayInd]->size() << endl;
	double factor = surfArray[surfArrayInd]->size() / (2.0 * length);
	cout << "Factor = " << factor << " axis = " << surfArray[surfArrayInd]->axis << endl;
	if (fabs(factor) > 100000000.0)
		factor = 1.0;
	coords product = surfArray[surfArrayInd]->axis * factor;
	cout << "Product = " << product << endl;
	coords sum = surfArray[surfArrayInd]->position + product;
	cout << "Sum is " << sum << endl;
	surfArray[surfArrayInd]->position = sum;
	cout << "Position after resetting " << surfArray[surfArrayInd]->position << endl;
}


static void chkCurve(CComPtr<ICurve> curveptr) {
	if (hres == S_OK && curveptr) {
		hres = curveptr->IsBcurve(&retVal);
		cout << "IsBcurve: " << (hres == S_OK && retVal) << endl;
		hres = curveptr->IsEllipse(&retVal);
		cout << "IsEllipse: " << (hres == S_OK && retVal) << endl;
		hres = curveptr->IsLine(&retVal);
		cout << "IsLine: " << (hres == S_OK && retVal) << endl;
		hres = curveptr->IsTrimmedCurve(&retVal);
		cout << "IsTrimmedCurve: " << (hres == S_OK && retVal) << endl;
		hres = curveptr->IsCircle(&retVal);
		cout << "IsCircle: " << (hres == S_OK && retVal) << endl;
	}
}


coords AssemblyInfo::getbcurve(CComPtr<ICurve> curveptr) {
	VARIANT_BOOL cubic = true, irrational = false, 
		nonperiodic = false, closed = true;
	CComPtr<ISplineParamData> params;
	coords goodCenter(9999, 9999, 9999);
	hres = curveptr->GetBCurveParams5(cubic, irrational, nonperiodic, closed, &params);
	if (hres == S_OK && params) {
		long ptCnt = 0;
		hres = params->get_ControlPointsCount(&ptCnt);
		if (hres == S_OK) {
			cout << "Bcurve num of control points = " << std::dec << ptCnt << endl;
			long dimen = 0;
			hres = params->get_Dimension(&dimen);
			cout << "Num of dimensions = " << dimen << endl;
			VARIANT controlPts;
			VariantInit(&controlPts);
			hres = params->GetControlPoints(&controlPts, &retVal);
			if (hres == S_OK && retVal) {
				SAFEARRAY *ptsView = V_ARRAY(&controlPts);
				double *ptsptr = NULL;
				long lStartBound = 0;
				long lEndBound = 0;
				SafeArrayGetLBound(ptsView, 1, &lStartBound);
				SafeArrayGetUBound(ptsView, 1, &lEndBound);
				hres = SafeArrayAccessData(ptsView, (void**)&ptsptr);
				if (hres == S_OK && ptsptr) {
					coords center(0.0, 0.0, 0.0);
					coords beginPt, quarterPt;
					for (int ind = lStartBound; ind <= lEndBound - 2; ind += dimen) {
						coords pt(ptsptr[ind], ptsptr[ind + 1], ptsptr[ind + 2]);
						// cout << "Control pt #" << ind / dimen << " " << pt << endl;
						center = pt + center;
						if (ind == lStartBound)
							beginPt = pt;
						else {
							int quarterWay = (lEndBound - lStartBound + 1) / dimen;
							quarterWay /= 4; // Go a quarter of the way around circle
							if (ind == (quarterWay * dimen))
								quarterPt = pt;
						}
					}
					double factor = (double) dimen / (lEndBound - lStartBound + 1.0);
					center = center * factor;
					cout << "Center pt " << center << endl;
					if (surfArray[surfArrayInd]->axis.length() == 0 && (surfArray[surfArrayInd]->surfID == BSURF_ID || surfArray[surfArrayInd]->surfID == OFFSET_ID)) {
						surfArray[surfArrayInd]->axisPts.push_back(center);		// Save center to later calculate axis
					}
					goodCenter = center;
					cout << "beginPt and quarterPt = " << beginPt << " " << quarterPt << endl;
					coords planeVec1 = center - beginPt;
					coords planeVec2 = center - quarterPt;
					planeVec1.normalize();
					planeVec2.normalize();
					coords axis = crossProd(planeVec1, planeVec2);  // Normal to face is possible axis
					axis.normalize();
					cout << "planeVec1 & 2 and normal for axis " << planeVec1 << " " << planeVec2 << " " << axis << endl;
					surfArray[surfArrayInd]->axisList.push_back(axis);
				}
				else cout << "Can't open array = " << hres << endl;
				SafeArrayUnaccessData(ptsView);
				SafeArrayDestroy(ptsView);

			}
			else cout << "Can't get control points: " << hres << endl;
		}
	}
	return (goodCenter);
}


void AssemblyInfo::getLineInfo(CComPtr<ICurve> curveptr, double sideLen) {
	VARIANT lineParams;
	VariantInit(&lineParams);
	hres = curveptr->get_LineParams(&lineParams);
	if (hres == S_OK) {
		long paramBegin, paramLim;
		double *paramArray = varDblArrayAccess(lineParams, paramBegin, paramLim);
		if (paramLim - paramBegin >= 5) {
			coords lineRoot(paramArray[paramBegin], paramArray[paramBegin + 1], paramArray[paramBegin + 2]);
			coords lineDir(paramArray[paramBegin + 3], paramArray[paramBegin + 4], paramArray[paramBegin + 5]);
			cout << "Line root " << lineRoot << endl;
			cout << "Line direction " << lineDir << endl;
			sideInfo thisSide;
			thisSide.point = lineRoot;
			thisSide.lineDir = lineDir;
			thisSide.length = sideLen;
			surfArray[surfArrayInd]->sideList.push_back(thisSide);
		}
		else cout << "Too few line parameters: " << paramLim << endl;
	}
	else cout << "Can't get line params\n";
}


/* Not good --only works for axis that is major axis
VARIANT point;
VariantInit(&point);
hres = edgeptr->GetClosestPointOn(0.0, 0.0, 0.0, &point);
long paramBegin, paramLim;
double *ptarray = varDblArrayAccess(point, paramBegin, paramLim);
if (ptarray != NULL) {
for (int index = paramBegin; index <= paramLim; ++index) {
double val = ptarray[index];
if (fabs(val) < 0.00001)
val = 0.0;
if (index == paramBegin) {
cout << "distance to origin (" << val << ", ";
closestPt.x = val;
}
else if (index == paramBegin + 1) {
cout << val << ", ";
closestPt.y = val;
}
else if (index == paramBegin + 2) {
cout << val << ")" << endl;
closestPt.z = val;
} else if (index == paramBegin + 3)
cout << "u = " << val << "\n";
} // end for xyzu
closestPt = calcVecOnAxis(surfArray[surfArrayInd]->axis, closestPt);

if (iIndex == lStartBound)
surfArray[surfArrayInd]->position = closestPt;
else if (closestPt.length() < surfArray[surfArrayInd]->position.length()) {
	prevPt = surfArray[surfArrayInd]->position;
	surfArray[surfArrayInd]->position = closestPt;
}
else prevPt = closestPt;
					} // end if not null
					*/


static SAFEARRAY *assignvararray(int facetype, LPDISPATCH *srcptr, int srcindex, int size = 1)
{
	SAFEARRAY *cparray = NULL;
	SAFEARRAYBOUND aDim[1];
	aDim[0].lLbound = 0;
	aDim[0].cElements = size;
	cparray = SafeArrayCreate(facetype, 1, aDim);
	LPDISPATCH *dspptr = NULL;
	hres = SafeArrayAccessData(cparray, (void**)&dspptr);
	if (hres == S_OK && dspptr && srcptr) {
		dspptr[0] = srcptr[srcindex];
		SafeArrayUnaccessData(cparray);
	}
	else cout << "bad ptrs h d s " << std::hex << hres << " " << dspptr << " " << srcptr << endl;
	return (cparray);
}


static coords findCenter(const vector<sideInfo> &sideList) {
	coords center;
	// std::cout << " Candidate points are ";
	for (int ind = 0; ind < 4; ++ind) {
		// cout << sideList[ind].point << endl;
		center = center + sideList[ind].point;
	}
	center = center * 0.25;
	// std::cout << "Candidate center is " << center << endl;
	return (center);
}


// Note that "side" may get its lineDir changed
static coords getBoxVertex(sideInfo &side, const coords &approxCenter) {
	coords chkDir = approxCenter - side.point;
	if (dotProd(chkDir, side.lineDir) < 0)	// If lineDir pointing away from center
		side.lineDir = side.lineDir * -1.0;		// Reverse it
	coords pt1 = side.lineDir * side.length;
	pt1 = pt1 + side.point;
	return (pt1);
}


void AssemblyInfo::calcBoard() {
	// Only considered four-sided boards for now
	BoardSurf *boardptr = surfArray[surfArrayInd]->boardPtr();
	if (surfArray[surfArrayInd]->sideList.size() == 4 && boardptr != NULL) {
		// double longSide = boardptr->length, width = boardptr->width;
		// double hypotenuse = sqrt(longSide * longSide + width * width);
		coords approxCenter = findCenter(surfArray[surfArrayInd]->sideList);
		// bool foundSides = false;
		vector< vector<sideInfo> > sideLists;
		for (int ind1 = 0; ind1 < 2; ++ind1) {
			int ind2 = ind1 + 2;
			// Construct the other two points by following the line direction a distance equal to the line length.
			// Note original lineDir may be corrected by getBoxVertex
			coords pt1 = getBoxVertex(surfArray[surfArrayInd]->sideList[ind1], approxCenter);
			coords pt2 = getBoxVertex(surfArray[surfArrayInd]->sideList[ind2], approxCenter);
			vector<sideInfo> testList(4);
			testList[ind1] = surfArray[surfArrayInd]->sideList[ind1];
			testList[ind2] = surfArray[surfArrayInd]->sideList[ind2];
			int ind3 = 1;
			if (ind1 > 0)
				ind3 = 0;
			testList[ind3] = surfArray[surfArrayInd]->sideList[ind3];
			testList[ind3].point = pt1;
			ind3 += 2;
			testList[ind3] = surfArray[surfArrayInd]->sideList[ind3];
			testList[ind3].point = pt2;
			sideLists.push_back(testList);
			/*
			for (int ind2 = ind1 + 1; ind2 < 4 && foundSides == false; ++ind2) {
				coords diag = surfArray[surfArrayInd]->sideList[ind2].point - surfArray[surfArrayInd]->sideList[ind1].point;
				cout << "Correct hypotenuse and calculated val = " << hypotenuse << " " << diag.length() << endl;
				if (checkZero(diag.length() - hypotenuse) == 0.0) {
					cout << "Found good hypo " << endl;
					coords lineDir1 = surfArray[surfArrayInd]->sideList[ind1].lineDir;
					coords lineDir2 = surfArray[surfArrayInd]->sideList[ind2].lineDir;
					if (dotProd(diag, lineDir1) < 0)
						lineDir1 = lineDir1 * -1.0; // Reverse line direction
					if (dotProd(diag, lineDir2) > 0) // lineDir2 should go in opposite direction
						lineDir2 = lineDir2 * -1.0; // Reverse line direction
					// Construct the other two points by following the line direction a distance equal to the line length.
					coords pt1 = lineDir1 * surfArray[surfArrayInd]->sideList[ind1].length;
					pt1 = pt1 + surfArray[surfArrayInd]->sideList[ind1].point;
					coords pt2 = lineDir2 * surfArray[surfArrayInd]->sideList[ind2].length;
					pt2 = pt2 + surfArray[surfArrayInd]->sideList[ind2].point;
					bool usedPt1 = false;
					for (int ind3 = 0; ind3 < 4; ++ind3) {
						if (ind3 != ind1 && ind3 != ind2) {
							if (usedPt1 == false) {
								cout << "Resetting " << surfArray[surfArrayInd]->sideList[ind3].point << " to " << pt1 << endl;
								surfArray[surfArrayInd]->sideList[ind3].point = pt1;
								usedPt1 = true;
							}
							else surfArray[surfArrayInd]->sideList[ind3].point = pt2;
						}
					}
					foundSides = true;
				}
			} // end for
			*/
		} // end for ind1
		coords center1 = findCenter(sideLists[0]);
		coords center2 = findCenter(sideLists[1]);
		coords dist1 = center1 - approxCenter;
		coords dist2 = center2 - approxCenter;
		vector<sideInfo> &goodList = sideLists[0];
		// cout << "distance 1, distance 2 = " << dist1.length() << " " << dist2.length() << endl;
		if (dist1.length() > dist2.length())
			goodList = sideLists[1];
		coords center;
		// std::cout << " Four points are ";
		for (int ind = 0; ind < 4; ++ind) {
			surfArray[surfArrayInd]->sideList[ind] = goodList[ind];
			cout << surfArray[surfArrayInd]->sideList[ind].point << " len = " << surfArray[surfArrayInd]->sideList[ind].length;
			cout << " dir = " << surfArray[surfArrayInd]->sideList[ind].lineDir << endl;
			center = center + surfArray[surfArrayInd]->sideList[ind].point;
		}
		center = center * 0.25;
		std::cout << "Center is " << center << endl;
		surfArray[surfArrayInd]->position = center;
		int index = 0;
		if (surfArray[surfArrayInd]->sideList[0].length > surfArray[surfArrayInd]->sideList[1].length) {
			index = 1;
		}
		// startAxis is the short axis of the board, corresponding to the x-axis
		surfArray[surfArrayInd]->startAxis = surfArray[surfArrayInd]->sideList[index].lineDir;
		surfArray[surfArrayInd]->startAxis.normalize();
		cout << "Setting startAxis to dir = " << surfArray[surfArrayInd]->startAxis << " for len = " << surfArray[surfArrayInd]->sideList[index].length << endl;
	}
}

static HRESULT getEdgeClosestPt(IEdge *const edgeptr, const coords &testpt, coords &closestPt) {
	VARIANT point;
	VariantInit(&point);
	hres = edgeptr->GetClosestPointOn(testpt.x, testpt.y, testpt.z, &point);
	long paramBegin, paramLim;
	double *ptarray = varDblArrayAccess(point, paramBegin, paramLim);
	if (ptarray != NULL) {
		for (int index = paramBegin; index <= paramLim; ++index) {
			double val = ptarray[index];
			if (fabs(val) < 0.00001)
				val = 0.0;
			if (index == paramBegin) {
				// cout << "distance to testpt (" << val << ", ";
				closestPt.x = val;
			}
			else if (index == paramBegin + 1) {
				// cout << val << ", ";
				closestPt.y = val;
			}
			else if (index == paramBegin + 2) {
				// cout << val << ")" << endl;
				closestPt.z = val;
			}
			// else if (index == paramBegin + 3)
				// cout << "u = " << val << "\n";
		} // end for xyzu
		return (S_OK);
	}
	return (-1);
}

// Compares two sides with a standard axis. The more parallel one is chosen as the startSide.
// If both sides are about equally close, and one points along the axis and one points opposite,
// choose the one pointing along.
// If both sides give same dot product with stdAxis, return false because we need to check another axis.
// Returns true if startSide and endSide are correct (note they may get exchanged).
static bool cmpWithStdAxis(const coords &stdAxis, coords &startSide, coords &endSide) {
	double startVal = dotProd(stdAxis, startSide);
	double endVal = dotProd(stdAxis, endSide);
	if (dblgt(fabs(endVal), fabs(startVal)) ||
		(checkZero(fabs(startVal) - fabs(endVal)) == 0.0 && (startVal * endVal) < 0 && endVal > 0)) {
		coords tmpSide = startSide;
		startSide = endSide;
		endSide = tmpSide;
	}
	else if (checkZero(startVal - endVal) == 0.0) // Both values same
		return (false);
	return (true);
}


void AssemblyInfo::resetAxis(const coords &center, const coords &startVertex, const coords &endVertex) {
	coords startSide = startVertex - center;
	coords endSide = endVertex - center;
	if (cmpWithStdAxis(xaxis, startSide, endSide) == false && cmpWithStdAxis(yaxis, startSide, endSide) == false)
		(void) cmpWithStdAxis(zaxis, startSide, endSide);
	coords goodAxis = crossProd(startSide, endSide);
	cout << "Comparing current axis with calculated good axis direction " << surfArray[surfArrayInd]->axis << " " << goodAxis << endl;
	if (dotProd(goodAxis, surfArray[surfArrayInd]->axis) < 0.0)	// Current axis points wrong way
		surfArray[surfArrayInd]->axis = surfArray[surfArrayInd]->axis * -1.0;
}


coords AssemblyInfo::choseStartAxis(const coords &center, const coords &axis, const coords &normal,
	const coords &startVertex, const coords &endVertex, coords &endAxis) {
	// Have to choose which end of radVec corresponds to right hand rule
	const coords radVec = endVertex - startVertex;
	coords chkAxis = crossProd(radVec, normal);
	cout << "radVec " << radVec << " chkAxis " << chkAxis << endl;
	coords keyVertex = startVertex, otherVertex = endVertex;
	if (dotProd(chkAxis, axis) > 0) {
		keyVertex = endVertex;
		otherVertex = startVertex;
	}
	coords startAxis = keyVertex - center;
	startAxis.normalize();
	cout << "Good start axis " << startAxis << " axis " << axis << " normal " << normal << endl;
	endAxis = otherVertex - center;
	endAxis.normalize();
	return (startAxis);
}


// Start axis is chosen to be most parallel to a standard axis and for start-to-end to match right hand rule
coords AssemblyInfo::getStartAxis(IEdge *const edgeptr, const coords &center, const coords &axis, double circRad, coords &endAxis) {
	CComPtr<IVertex> startvertexptr;
	hres = edgeptr->IGetStartVertex(&startvertexptr);
	if (hres == S_OK && startvertexptr) {
		VARIANT startPt;
		VariantInit(&startPt);
		hres = startvertexptr->GetPoint(&startPt);
		if (hres == S_OK) {
			long paramBegin, paramLim;
			double *ptarray = varDblArrayAccess(startPt, paramBegin, paramLim);
			long numParams = paramLim - paramBegin + 1;
			if (ptarray != NULL && numParams >= 3) {
				coords startVertex(ptarray[0], ptarray[1], ptarray[2]);
				CComPtr<IVertex> endvertexptr;
				hres = edgeptr->IGetEndVertex(&endvertexptr);
				if (hres == S_OK && endvertexptr) {
					VARIANT endPt;
					VariantInit(&endPt);
					hres = endvertexptr->GetPoint(&endPt);
					if (hres == S_OK) {
						long endParamBegin, endParamLim;
						double *endptarray = varDblArrayAccess(endPt, endParamBegin, endParamLim);
						long numEndParams = endParamLim - endParamBegin + 1;
						if (endptarray != NULL && numEndParams >= 3) {
							coords endVertex(endptarray[0], endptarray[1], endptarray[2]);
							cout << "Start vertex = " << startVertex << " endVertex = " << endVertex << endl;
							resetAxis(center, startVertex, endVertex); // Can reverse surfArray[surfArrayInd]->axis
							const coords radVec = endVertex - startVertex;
							coords normal = crossProd(radVec, axis);
							if (normal.length() == 0)
								cout << "Bad normal to radius and axis\n";
							else {
								normal = normal * (1.1 * circRad / normal.length());
								int cnt = 0;
								do {
									coords testpt = normal + center;
									coords closestPt;
									hres = getEdgeClosestPt(edgeptr, testpt, closestPt);
									if (hres == S_OK) {
										coords distance = closestPt - testpt;
										if (distance.length() < circRad) {
											return (choseStartAxis(center, axis, normal, startVertex, endVertex, endAxis));
										}
										else cout << "distance " << distance << " radius " << circRad << " closept " << closestPt << " testpt " << testpt << endl;
									}
									normal = normal * -1.0;	// Try opposite direction
									++cnt;
								} while (cnt < 2);
								cout << "ERROR: Couldn't find start axis\n";
							} // end else
						} // end if endptarray != NULL
					} // end if got endpt
				}
				else cout << "Failed to get end vertex\n";
			} // end if start ptarray != NULL
		} // end if got startVertex
	} else cout << "Failed to get start vertex\n";
	coords badAxis;
	return (badAxis);
}


void AssemblyInfo::chkEllipAxis()
{
	EllipsoidSurf *ellip = surfArray[surfArrayInd]->ellipsoidPtr();
	if (ellip != NULL) {
		// double testLen = ellip->cz * 1.1;
		coords testVec = surfArray[surfArrayInd]->axis * ellip->cz;
		coords testPt = surfArray[surfArrayInd]->position + testVec;
		VARIANT point;
		VariantInit(&point);
		hres = surfArray[surfArrayInd]->facePtr->GetClosestPointOn(testPt.x, testPt.y, testPt.z, &point);
		long paramBegin, paramLim;
		double *ptarray = varDblArrayAccess(point, paramBegin, paramLim);
		if (ptarray != NULL && paramLim - paramBegin >= 2) {
			coords facePt(ptarray[paramBegin], ptarray[paramBegin + 1], ptarray[paramBegin + 2]);
			coords farPt = surfArray[surfArrayInd]->position - testVec;
			cout << "Point on face  " << facePt << " test pt " << testPt << " far pt " << farPt << endl;
			coords nearDist = facePt - testPt;
			coords farDist = facePt - farPt;
			if (farDist.length() < nearDist.length()) {
				surfArray[surfArrayInd]->axis = surfArray[surfArrayInd]->axis * -1.0;
				cout << "Reverse axis since ellipsoid bulges out other way. New axis " << surfArray[surfArrayInd]->axis << endl;
			}
		}
	}
}


void AssemblyInfo::getEdgeDist(IFace2 *const faceptr, CComPtr<IMeasure> &mymeasure, long surfID, double radius)
{
	VARIANT edgearray;
	VariantInit(&edgearray);
	hres = faceptr->GetEdges(&edgearray);
	if (hres == S_OK) {
		SAFEARRAY *edgesview = V_ARRAY(&edgearray);
		LPDISPATCH *srcptr = NULL;
		long lStartBound = 0;
		long lEndBound = 0;
		SafeArrayGetLBound(edgesview, 1, &lStartBound);
		SafeArrayGetUBound(edgesview, 1, &lEndBound);
		hres = SafeArrayAccessData(edgesview, (void**)&srcptr);
		if (hres == S_OK && srcptr) {
			coords closestPt(9999, 9999, 9999), prevPt;
			bool prevPtSet = false;
			for (int iIndex = lStartBound; iIndex <= lEndBound; iIndex++) {
				VARIANT oneEdge;
				VariantInit(&oneEdge);
				oneEdge.vt = edgearray.vt;
				int edgetype = edgearray.vt & ~(VT_ARRAY);
				oneEdge.parray = assignvararray(edgetype, srcptr, iIndex);
				double sideLen = 0.0;
				if (oneEdge.parray != NULL) {
					calcmeasure(mymeasure, oneEdge, radius, surfID, sideLen);
				}
				IEdge *edgeptr = NULL;
				hres = (srcptr[iIndex])->QueryInterface(__uuidof(IEdge), reinterpret_cast<void**>(&edgeptr));
				if (hres == S_OK && edgeptr) {
					CComPtr<ICurve> curveptr;
					hres = edgeptr->IGetCurve(&curveptr);
					// cout << "IGetCurve result = " << hres << endl;
					if (hres == S_OK && curveptr) {
						// CComPtr<ICurve> basecurveptr;
						// hres = curveptr->GetBaseCurve(&basecurveptr);
						// chkCurve(basecurveptr);
						hres = curveptr->IsBcurve(&retVal);
						bool bcurve = (hres == S_OK && retVal);
						// cout << "IsBcurve: " << bcurve << endl;
						//hres = curveptr->IsEllipse(&retVal);
						//cout << "IsEllipse: " << (hres == S_OK && retVal) << endl;
						hres = curveptr->IsLine(&retVal);
						bool goodLine = (hres == S_OK && retVal);
						// cout << "IsLine: " << goodLine << endl;
						if (goodLine)
							getLineInfo(curveptr, sideLen);
						// hres = curveptr->IsTrimmedCurve(&retVal);
						// cout << "IsTrimmedCurve: " << (hres == S_OK && retVal) << endl;
						hres = curveptr->IsCircle(&retVal);
						if (hres == S_OK && retVal) {
							// double circleParams[7];
							// for (int ind = 0; ind < 7; ++ind)
							// circleParams[ind] = -1.0;
							VARIANT circleParams;
							VariantInit(&circleParams);
							hres = curveptr->get_CircleParams(&circleParams);
							if (hres == S_OK) {
								long paramBegin, paramLim;
								double *paramArray = varDblArrayAccess(circleParams, paramBegin, paramLim);
								if (paramLim - paramBegin >= 6) {
									coords center(paramArray[paramBegin], paramArray[paramBegin + 1], paramArray[paramBegin + 2]);
									coords axis(paramArray[paramBegin + 3], paramArray[paramBegin + 4], paramArray[paramBegin + 5], true);
									if (axis.length() != 0) {
										axis.normalize();
										surfArray[surfArrayInd]->axisList.push_back(axis);
										if (surfArray[surfArrayInd]->axis.length() == 0) {
											surfArray[surfArrayInd]->axis = axis;
											cout << "Setting surface axis to circle axis since surface axis was zero.\n";
										}
									}
									cout << "Circle center " << center << endl;
									if (surfArray[surfArrayInd]->surfID == BSURF_ID || surfArray[surfArrayInd]->surfID == OFFSET_ID ||
										surfArray[surfArrayInd]->surfID == TORUS_ID) {
										surfArray[surfArrayInd]->axisPts.push_back(center);		// Save center to later calculate axis
									}
									double circRad = paramArray[paramBegin + 6];
									cout << "Circle radius = " << circRad << "; axis " << axis << endl;
									if (surfArray[surfArrayInd]->getAngle() < 2.0 * M_PI && surfArray[surfArrayInd]->startAxis.length() == 0)
										surfArray[surfArrayInd]->startAxis = getStartAxis(edgeptr, center, surfArray[surfArrayInd]->axis,
										circRad, surfArray[surfArrayInd]->endAxis);
									closestPt = center;
									// Circular edge indicates a disk if the surface is a plane
									if (surfArray[surfArrayInd]->surfID == PLANE_ID) {
										setPlaneSurf(&(surfArray[surfArrayInd]), DISK_ID);
										if (surfArray[surfArrayInd]->setSize(circRad, 0, 0, 0, 0, 0) &&
												(diskList.shapeInds.empty() || diskList.shapeInds.back() != surfArrayInd)) {
											diskList.shapeInds.push_back(surfArrayInd);
											cout << "Found disk surf index = " << std::dec << surfArrayInd << endl;
										}
										if (boardList.shapeInds.empty() == false && boardList.shapeInds.back() == surfArrayInd)
											boardList.shapeInds.pop_back(); // Remove from board list
									}
								}
								else cout << "Too few circle parameters: " << paramLim << endl;
							}
							else cout << "Can't get circle params\n";
						}
						else if (bcurve) {
							closestPt = getbcurve(curveptr);
						}
						else cout << "Edge not a bcurve or circle\n";
						edgeList.push_back(make_pair(closestPt, surfArrayInd));
						if (iIndex == lStartBound)
							surfArray[surfArrayInd]->position = closestPt;
						else if (closestPt.length() < surfArray[surfArrayInd]->position.length()) {
							prevPt = surfArray[surfArrayInd]->position;
							prevPtSet = true;
							surfArray[surfArrayInd]->position = closestPt;
						}
						else {
							prevPt = closestPt;
							prevPtSet = true;
						}
					}
				}
			} // end for edges array
			calcBoard();
			if (surfID != TORUS_ID && surfArray[surfArrayInd]->axisList.size() >= 2 && surfArray[surfArrayInd]->axisPts.size() >= 2) {
				coords closeToAxis = surfArray[surfArrayInd]->axisPts[1] - surfArray[surfArrayInd]->axisPts[0];
				coords closeToAxiswLen = closeToAxis;
				closeToAxis.normalize();
				double axisDiff1 = dotProd(surfArray[surfArrayInd]->axisList[0], closeToAxis);
				double axisDiff2 = dotProd(surfArray[surfArrayInd]->axisList[1], closeToAxis);
				int index = 0;
				// closeToAxis is line between top and bottom centers. It should be close to the real axis.
				// Chose the axisList that is more parallel to closeToAxis (bigger dot product).
				if (fabs(axisDiff1) < fabs(axisDiff2))
					index = 1;
				surfArray[surfArrayInd]->axis = surfArray[surfArrayInd]->axisList[index];
				cout << "Possibly choosing new axis among options: " << surfArray[surfArrayInd]->axis << "\n";
				double coneLen = sqrt(fabs(dotProd(surfArray[surfArrayInd]->axis, closeToAxiswLen)));
				cout << "Calculated center-to-center cone length = " << coneLen << endl;
				(void) surfArray[surfArrayInd]->setSize(-1, 0, 0, 0, 0, coneLen);	// Override cone len
			}
			if (surfID != PLANE_ID && prevPtSet) {
				coords bodyVec = prevPt - surfArray[surfArrayInd]->position;	// Vector into body of part
				cout << "Body vec = " << bodyVec << " surfind " << surfArrayInd << endl;
				if (dotProd(bodyVec, surfArray[surfArrayInd]->axis) < 0.0) {
					surfArray[surfArrayInd]->axis = surfArray[surfArrayInd]->axis * -1.0;  // Reverse axis
					surfArray[surfArrayInd]->startAxis = surfArray[surfArrayInd]->endAxis; // Use other secondary axis
				}
			}
		}
		SafeArrayUnaccessData(edgesview);
		SafeArrayDestroy(edgesview);
	}
}



GenericSurf *makeSurface(ISurface *swSurf, long *surfID)
// surfID is assumed to be initialized to -1.
{
	if (swSurf)
	{
		if (swSurf->Identity(surfID) == S_OK)
		{
			switch (*surfID)
			{
			case CYLINDER_ID:
				return(new CylSurf(*surfID, swSurf));
				break;
			case CONE_ID:
				return(new ConeSurf(*surfID, swSurf));
				break;
			case TORUS_ID:
				return(new TorusSurf(*surfID, swSurf));
				break;
			case S_REVOLVE_ID:
				return(new EllipsoidSurf(*surfID, swSurf));
				break;
			case PLANE_ID:
				return(new BoardSurf(*surfID, swSurf));
				// Default to BoardSurf.
				// If circular edge found, then convert to DISK_ID
				break;
			case BSURF_ID:
			case OFFSET_ID:
				return(new ConeSurf(*surfID, swSurf)); // For now, treat bsurf & offsets as cones
				break;
			}
		}
	}
	cout << "Failed to find surface identity --defaulting to Cone\n";
	return(new ConeSurf(*surfID, swSurf));
}


const char *GenericSurf::surfName()
{
	long index = surfID;
	index -= 4001;
	if (index >= 0 && index < (sizeof(surftypes) / sizeof(char *)))
		return (surftypes[index]);
	else return ("null name");
}



void AssemblyInfo::showFaceDetails(LPDISPATCH *srcptr, int srcindex, double *radius, long *surfID, CComPtr<IMeasure> &mymeasure)
{
		IFace2 *faceptr = NULL;
		hres = (srcptr[srcindex])->QueryInterface(__uuidof(IFace2), reinterpret_cast<void**>(&faceptr));
		if (hres == S_OK && faceptr) {
			cout << "\n face num " << std::dec << srcindex << endl;
			struct IDispatch *pSurfDisp;
			ISurface *swSurf = NULL;
			hres = faceptr->GetSurface(&pSurfDisp);
			if (hres == S_OK && pSurfDisp) {
				hres = pSurfDisp->QueryInterface(__uuidof(ISurface), reinterpret_cast<void**>(&swSurf));
				if (hres == S_OK && swSurf) {
					surfArray.push_back(makeSurface(swSurf, surfID));
					++surfArrayInd;
					cout << "surf name is " << surfArray[surfArrayInd]->surfName() << endl;
					*radius = surfArray[surfArrayInd]->showSurfParams(this);
					surfArray[surfArrayInd]->displace = displace;
					cout << "Setting index " << surfArrayInd << " to displace " << displace << endl;
					if (overallRotCmpNmStr.size() == 0 || overallRotCmpNmStr.compare(compNameStr) == 0)
						surfArray[surfArrayInd]->overallRot = overallRot;
					surfArray[surfArrayInd]->matNameStr = matNameStr;
					surfArray[surfArrayInd]->compNameStr = compNameStr;
					surfArray[surfArrayInd]->pathNameStr = pathNameStr; 
					surfArray[surfArrayInd]->featureTypeStr = featureTypeStr;
					surfArray[surfArrayInd]->facePtr = faceptr;
				}
				else cout << "can't convert surf h ptr " << std::hex << hres << " " << swSurf << endl;
				pSurfDisp->Release();
			}
			else cout << "can't get surf h dptr " << std::hex << hres << " " << pSurfDisp << endl;
			long cnt = -1;
			hres = faceptr->GetEdgeCount(&cnt);
			if (hres == S_OK) {
				cout << "edge cnt " << cnt << endl;
				if (cnt > 0)
					getEdgeDist(faceptr, mymeasure, *surfID, *radius);
			}
			/*
			IFeature *faceFeature = NULL;
			hres = faceptr->IGetFeature(&faceFeature);
			if (hres == S_OK && faceFeature) {
				CComBSTR sGetFeatureName;
				hres = faceFeature->get_Name(&sGetFeatureName);
				if (hres == S_OK && sGetFeatureName) {
					CW2A featurenam(sGetFeatureName);
					cout << " **** face feature name " << featurenam << endl;
				}
			}
			else cout << "Couldn't get face feature hres ptr = " << hres << " " << faceFeature << endl;
			*/
		}
}


static void showDistances(VARIANT &twofaces, CComPtr<IMeasure> &mymeasure)
{
	if (mymeasure == NULL) {
		cout << "bad measure\n";
		return;
	}
	hres = mymeasure->Calculate(twofaces, &retVal);
	if (hres == S_OK && retVal) {
		double val = 0.0;
		hres = mymeasure->get_DeltaX(&val);
		if (hres == S_OK) {
			cout << "Delta xyz (" << val;
		}
		else cout << "Bad delta  hres " << std::hex << hres << endl;
		hres = mymeasure->get_DeltaY(&val);
		if (hres == S_OK) {
			cout << ", " << val;
		}
		hres = mymeasure->get_DeltaZ(&val);
		if (hres == S_OK) {
			cout << ", " << val << ")" << endl;
		}
		hres = mymeasure->get_Distance(&val);
		if (hres == S_OK) {
			cout << "distance  " << val << endl;
		}
		hres = mymeasure->get_Projection(&val);
		if (hres == S_OK) {
			cout << "projected distance  " << val << endl;
		}
		hres = mymeasure->get_TotalArea(&val);
		if (hres == S_OK) {
			cout << "total area  " << val << endl;
		}
		hres = mymeasure->get_TotalLength(&val);
		if (hres == S_OK) {
			cout << "total length  " << val << endl;
		}
	}
	else cout << "Bad 2 face calc hres retval " << std::hex << hres << " " << retVal << endl;
}


/* leftover example
if (firstFace) {
	twofaces.parray = assignvararray(facetype, srcptr, iIndex, 2);
	firstFace = false;
}
else {
	LPDISPATCH *arrayptr = NULL;
	hres = SafeArrayAccessData(twofaces.parray, (void**)&arrayptr);
	if (hres == S_OK && arrayptr && srcptr) {
		arrayptr[1] = srcptr[iIndex]; // Assign into twofaces
		// hres = swSelMgr->SuspendSelectionList(&lNumSelections);
		// cout << "trying to suspend sel list hres numremoved " << std::hex << hres << " " << lNumSelections << endl;
		// swSelMgr->GetSelectedObjectCount2(-1, &lNumSelections);
		// cout << "current num selections " << lNumSelections << endl;
		// hres = swSelMgr->AddSelectionListObjects(twofaces, swSelData, &lNumSelections);
		// cout << "trying to add two faces to sel hres numadded " << std::hex << hres << " " << lNumSelections << endl;
		// swSelMgr->GetSelectedObjectCount2(-1, &lNumSelections);
		// cout << "current num selections " << lNumSelections << endl;
		// VARIANT nullvar;
		// VariantInit(&nullvar);
		// nullvar.vt = VT_NULL;
		// hres = mymeasure->Calculate(nullvar, &retVal);
		showDistances(twofaces, mymeasure);
	}
	else {
		cout << "Can't get ptr into 2 faces hres ptr " << std::hex << hres << " " << arrayptr << endl;
		SafeArrayUnaccessData(twofaces.parray);
	}
}
*/

static void getFaceDistances(int faceIndList[20], int listSz, LPDISPATCH *srcptr, CComPtr<IMeasure> &mymeasure,
	int facetype)
{
	for (int ind = 0; ind < listSz - 1; ++ind)
	{
		VARIANT twofaces;
		VariantInit(&twofaces);
		twofaces.vt = facetype;
		cout << "Starting with index " << faceIndList[ind] << endl;
		twofaces.parray = assignvararray(VT_DISPATCH, srcptr, faceIndList[ind], 2);
		if (twofaces.parray != NULL) {
			for (int ind2 = ind + 1; ind2 < listSz; ++ind2)
			{
				LPDISPATCH *arrayptr = NULL;
				hres = SafeArrayAccessData(twofaces.parray, (void**)&arrayptr);
				if (hres == S_OK && arrayptr && srcptr) {
					cout << "compared with index " << faceIndList[ind2] << endl;
					if (srcptr[faceIndList[ind2]] != NULL) {
						arrayptr[1] = srcptr[faceIndList[ind2]]; // Assign into twofaces
						SafeArrayUnaccessData(twofaces.parray);
						showDistances(twofaces, mymeasure);
					}
					else cout << "bad src reference\n";
				}
			}
			SafeArrayDestroy(twofaces.parray);
		}
		else cout << "bad twofaces\n";
	}
}


// Assumes cone1 and cone2 are two surfaces of same cone
 string conesList::outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly)
{
	ConeSurf *cone1 = (assembly->surfArray)[ind1]->conePtr(), *cone2 = (assembly->surfArray)[ind2]->conePtr();
	if (cone1 != NULL && cone2 != NULL) {
		double inSmRadius = cone1->smRadius, outSmRadius = cone2->smRadius;
		double inLgRadius = cone1->lgRadius, outLgRadius = cone2->lgRadius;
		if (outLgRadius < inLgRadius)
		{
			outSmRadius = inSmRadius;	// Exchange in/out radii to be correct
			outLgRadius = inLgRadius;
			inSmRadius = cone2->smRadius;
			inLgRadius = cone2->lgRadius;
		}
		else if (outLgRadius == inLgRadius) {
			// Only one surface -- it's a hole
			inSmRadius = inLgRadius = 0.0;
		}
		const char *const name = nameIncr(CONE_ID);
		xmlElem beginEnd, attrib1, attrib2;
		gdmlout << indent1 << beginEnd.openElem("cone") << attrib1.attribute("name", name) << attrib2.attribute("z", cone1->height * units);
		gdmlout << attrib1.attribute("rmin1", inLgRadius * units ) << attrib2.attribute("rmin2", inSmRadius * units);
		gdmlout << attrib1.attribute("rmax1", outLgRadius * units ) << attrib2.attribute("rmax2", outSmRadius * units);
		gdmlout << attrib1.attribute("deltaphi", "TWOPI") << beginEnd.closeElem() << endl;
		return (name);
	}
	cout << "Bad cone indexes " << ind1 << " " << ind2 << endl;
	return ("null name");
}



// Assumes cyl1 and cyl2 are two surfaces of same cylinder
 string cylinderList::outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly)
{
	CylSurf *cyl1 = (assembly->surfArray)[ind1]->cylPtr(), *cyl2 = (assembly->surfArray)[ind2]->cylPtr();
	if (cyl1 != NULL && cyl2 != NULL) {
		double smRadius = cyl1->radius, lgRadius = cyl2->radius;
		if (lgRadius < smRadius)
		{
			lgRadius = smRadius;
			smRadius = cyl2->radius;
		}
		else if (lgRadius == smRadius) // Only one surface -- it's a hole or solid
			smRadius = 0;
		const char *const name = nameIncr(CYLINDER_ID);
		xmlElem beginEnd, attrib1, attrib2;
		gdmlout << indent1 << beginEnd.openElem("tube") << attrib1.attribute("name", name) << attrib2.attribute("z", cyl1->length * units);
		gdmlout << attrib1.attribute("rmin", smRadius * units ) << attrib2.attribute("rmax", lgRadius * units);
		gdmlout << attrib1.attribute("deltaphi", cyl1->angle) << beginEnd.closeElem() << endl;
		return (name);
	}
	cout << "Bad cylinder indexes " << ind1 << " " << ind2 << endl;
	return ("null name");
}


 string torusesList::outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly)
 {
	 TorusSurf *torus1 = (assembly->surfArray)[ind1]->torusPtr(), *torus2 = (assembly->surfArray)[ind2]->torusPtr();
	 if (torus1 != NULL && torus2 != NULL) {
		 double smRadius = torus1->lgRadius, smRad2 = torus2->lgRadius;
		 // lgRadius of inner torus is the inner surface (rmin)
		 if (smRad2 > smRadius)
		 {
			 smRad2 = smRadius;
			 smRadius = torus2->smRadius;
		 }
		 double lgRadius = torus1->lgRadius, lgRad2 = torus2->lgRadius;
		 if (lgRadius < lgRad2)
		 {
			 lgRadius = lgRad2;
			 lgRad2 = torus1->lgRadius;
		 }
		 if (lgRadius == smRadius) // Only one surface -- it's solid
			 smRadius = 0;
		 const char *const name = nameIncr(TORUS_ID);
		 xmlElem beginEnd, attrib1, attrib2;
		 gdmlout << indent1 << beginEnd.openElem("torus") << attrib1.attribute("name", name) << attrib2.attribute("rtor", torus1->majorRadius * units );
		 gdmlout << attrib1.attribute("rmin", smRadius * units ) << attrib2.attribute("rmax", lgRadius * units );
		 gdmlout << attrib1.attribute("startphi", 0.0);
		 gdmlout << attrib1.attribute("deltaphi", torus1->angle) << beginEnd.closeElem() << endl;
		 return (name);
	 }
	 cout << "Bad torus indexes " << ind1 << " " << ind2 << endl;
	 return ("null name");
 }


 string disksList::outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly)
 {
	 DiskSurf *disk1 = (assembly->surfArray)[ind1]->diskPtr(), *disk2 = (assembly->surfArray)[ind2]->diskPtr();
	 if (disk1 != NULL && disk2 != NULL && disk1->outRadius > 0) {
		 const char *const name = nameIncr(DISK_ID);
		 xmlElem beginEnd, attrib1, attrib2;
		 double inRad = disk1->inRadius, outRad = disk1->outRadius;
		 if (disk2->inRadius < inRad)
			 inRad = disk2->inRadius;
		 if (disk2->outRadius > outRad)
			 outRad = disk2->outRadius;
		 coords diffPos = (assembly->surfArray)[ind1]->position - (assembly->surfArray)[ind2]->position;
		 double thickness = MIN_THICKNESS;
		 double newThick = diffPos.length() * 2.0; // disk1 postion has already been averaged
		 double angle = disk1->angle;
		 if (disk2->angle > angle)
			 angle = disk2->angle;
		 if (newThick > MIN_THICKNESS && newThick < 0.7)
			 thickness = newThick;
		 gdmlout << indent1 << beginEnd.openElem("tube") << attrib1.attribute("name", name);
		 gdmlout << attrib1.attribute("rmin", inRad * units ) << attrib2.attribute("rmax", outRad * units );
		 gdmlout << attrib1.attribute("deltaphi", angle) << attrib2.attribute("z", thickness * units ) << beginEnd.closeElem() << endl;
		 return (name);
	 }
	 cout << "Bad disk indexes " << ind1 << " " << ind2 << endl;
	 return ("null name");
 }


 string boardsList::outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly)
 {
	 BoardSurf *board1 = (assembly->surfArray)[ind1]->boardPtr(), *board2 = (assembly->surfArray)[ind2]->boardPtr();
	 if (board1 != NULL && board2 != NULL) {
		 const char *const name = nameIncr(BOARD_ID);
		 xmlElem beginEnd, attrib1, attrib2;
		 gdmlout << indent1 << beginEnd.openElem("box") << attrib1.attribute("name", name);
		 // y axis appears to be default long axis
		 gdmlout << attrib1.attribute("x", board1->width * units ) << attrib2.attribute("y", board1->length * units );
		 gdmlout << attrib1.attribute("z", MIN_THICKNESS * units ) << beginEnd.closeElem() << endl;
		 return (name);
	 }
	 cout << "Bad board indexes " << ind1 << " " << ind2 << endl;
	 return ("null name");
 }


 string ellipList::outputShapeDesc(const int ind1, const int ind2, AssemblyInfo *assembly)
 {
	 EllipsoidSurf *ellip1 = (assembly->surfArray)[ind1]->ellipsoidPtr(), *ellip2 = (assembly->surfArray)[ind2]->ellipsoidPtr();
	 if (ellip1 != NULL && ellip2 != NULL) {
		 const char *const name = nameIncr(S_REVOLVE_ID);
		 xmlElem beginEnd, attrib1, attrib2;
		 gdmlout << indent1 << beginEnd.openElem("ellipsoid") << attrib1.attribute("name", name) << attrib2.attribute("ax", ellip1->ax * units );
		 gdmlout << attrib1.attribute("by", ellip1->by * units ) << attrib2.attribute("cz", ellip1->cz * units );
		 gdmlout << attrib1.attribute("zcut1", ellip1->zcutLow * units ) << beginEnd.closeElem() << endl;
		 return (name);
	 }
	 cout << "Bad ellipsoid indexes " << ind1 << " " << ind2 << endl;
	 return ("null name");
 }


void AssemblyInfo::outputShapeSetPart(shapeList *sList, const int ind1, const int ind2, bool averagePos)
{
	partDesc newPart;
	newPart.surfInd = ind1;
	newPart.matNameStr = surfArray[ind1]->matNameStr;
	if (ind1 != ind2 && averagePos) {
		coords avgpos = surfArray[ind1]->position + surfArray[ind2]->position;
		avgpos = avgpos * 0.5;
		surfArray[ind1]->position = avgpos;  // Average out any minor differences in position
	}
	newPart.createVol = true;
	newPart.inBoolSolid = false;
	newPart.nameID = sList->outputShapeDesc(ind1, ind2, this);
	if (newPart.nameID.compare("null name") != 0 && newPart.nameID.length() > 0) {
		surfArray[ind1]->partInd = surfArray[ind2]->partInd = partArray.size();
		partArray.push_back(newPart);
		surfArray[ind1]->wasOutput = surfArray[ind2]->wasOutput = true;
		cout << "Part name " << newPart.nameID << " from matching face indexes " << std::dec << ind1 << " " << ind2 << endl;
	}

}


void AssemblyInfo::outputHole(const long baseInd, const long holeInd)
{
	xmlElem beginEnd, attrib1, attrib2, attrib3;
	const string name = nameIncr(SUBTRACTION_ID);
	const string &baseName = partArray[surfArray[baseInd]->partInd].nameID;
	const string &subtractName = partArray[surfArray[holeInd]->partInd].nameID;
	gdmlout << indent1 << beginEnd.openSepElem("subtraction", attrib1.attribute("name", name.c_str())) << endl;
	gdmlout << indent2 << beginEnd.openElem("first") << attrib1.attribute("ref", baseName.c_str());
	gdmlout << beginEnd.closeElem() << endl;
	gdmlout << indent2 << beginEnd.openElem("second") << attrib1.attribute("ref", subtractName.c_str());
	gdmlout << beginEnd.closeElem() << endl;
	coords basePos = surfArray[baseInd]->position, holePos = surfArray[holeInd]->position;
	coords relPos = holePos - basePos;
	// Base object is considered by GDML to be along z-axis
	coords revrot = rotAnglesXYZ(surfArray[baseInd]->axis, zaxis); // Get rotation of base into z-axis
	cout << "Rotate base " << surfArray[baseInd]->axis << " into z by doing " << revrot << endl;
	coords newAxis = rotVecXYZ(surfArray[holeInd]->axis, revrot); // Apply this to the hole axis
	cout << "Hole axis " << surfArray[holeInd]->axis << " becomes new " << newAxis << endl;
	coords rotation = rotAnglesXYZ(zaxis, newAxis); // Then perform this rotation to place hole wrt base that is along z-axis
	coords newRelPos = rotVecXYZ(relPos, revrot); // Also Apply base rotation to relative position to get position for GDML
	cout << "Hole pos moves from " << relPos << " to " << newRelPos << endl;
	gdmlout << indent2 << beginEnd.openElem("position") << attrib1.attribute("name", nameIncr(POSITION_ID));
	gdmlout << attrib1.attribute("x", newRelPos.x * units ) << attrib2.attribute("y", newRelPos.y * units) << attrib3.attribute("z", newRelPos.z * units );
	gdmlout << beginEnd.closeElem() << endl;
	gdmlout << indent2 << beginEnd.openElem("rotation") << attrib1.attribute("name", nameIncr(ROTATION_ID));
	gdmlout << attrib1.attribute("x", rotation.x) << attrib2.attribute("y", rotation.y);
	gdmlout << attrib3.attribute("z", rotation.z);
	gdmlout << beginEnd.closeElem() << endl;
	gdmlout << indent1 << beginEnd.closeSepElem("subtraction") << endl;
	partDesc newPart;
	newPart.surfInd = baseInd;
	newPart.createVol = true;
	newPart.nameID = name;
	newPart.matNameStr = surfArray[baseInd]->matNameStr;
	partArray.push_back(newPart);
	// Base and hole now together in "subtraction".
	// Don't create volumes for them.
	partArray[surfArray[baseInd]->partInd].createVol = false;
	partArray[surfArray[holeInd]->partInd].createVol = false;
}


void AssemblyInfo::outputSolids(shapeList *sList, bool looseMatch, bool singleSolids)
{
	for (vector<int>::iterator it = sList->shapeInds.begin(); it != sList->shapeInds.end(); ++it)
	{
		if (surfArray[*it]->wasOutput == false) {
			if (singleSolids == false) {
				for (int step = 0; step == 0 || (looseMatch && step == 1); ++step) {
					if (surfArray[*it]->wasOutput == false) {
						for (vector<int>::iterator it2 = (it + 1); it2 != sList->shapeInds.end(); ++it2)
						{
							if (surfArray[*it2]->wasOutput == false) {
								if (surfArray[*it]->sameBody(surfArray[*it2], (step == 1))) {
									outputShapeSetPart(sList, *it, *it2);
									break;
								}
								else {
									// cout << "No match indexes " << std::dec << *it << " " << *it2 << " ";
									// cout << "pos1 pos2 " << surfArray[*it]->position << " " << surfArray[*it2]->position;
									// cout << " axis1 axis2 " << surfArray[*it]->axis << " " << surfArray[*it2]->axis;
									// cout << " size1 size2 " << surfArray[*it]->size() << " " << surfArray[*it2]->size() << endl;
								}
							}
						} // end for shapeInds
					} // end if was output == false
				} // end for steps
			}
			if (surfArray[*it]->wasOutput == false &&
				(singleSolids || surfArray[*it]->surfID == PLANE_ID || surfArray[*it]->surfID == BSURF_ID || surfArray[*it]->surfID == TORUS_ID)) {
				cout << "Outputting single face " << *it << endl;
				outputShapeSetPart(sList, *it, *it);
			}
		}
	}
}


void AssemblyInfo::breakUpFaces(shapeList *sList) {
	// Check for cylinders with inner & outer faces of different lengths
	for (vector<int>::iterator it = sList->shapeInds.begin(); it != sList->shapeInds.end(); ++it)
	{
		if (surfArray[*it]->wasOutput == false) {
			for (vector<int>::iterator it2 = sList->shapeInds.begin(); it2 != sList->shapeInds.end(); ++it2)
			{
				if (it != it2 && surfArray[*it2]->wasOutput == false) {
					if (surfArray[*it]->compNameStr.compare(surfArray[*it2]->compNameStr) == 0 &&
							surfArray[*it]->axis == surfArray[*it2]->axis) {
						coords separation = surfArray[*it]->position - surfArray[*it2]->position;
						if (surfArray[*it]->size() >= surfArray[*it2]->size() && separation.length() <= (surfArray[*it]->size() / 2.0)) {
							CylSurf *cyl1 = surfArray[*it]->cylPtr(), *cyl2 = surfArray[*it2]->cylPtr();
							// Allow radii to be merely close to each other
							if (cyl1 != NULL && cyl2 != NULL && (approxEqual(cyl1->radius, cyl2->radius) || fabs(cyl1->radius - cyl2->radius) < 1.0)) {
								outputShapeSetPart(sList, *it2, *it, false); // Use length & position of it2, the shorter one
								cout << "Matching overlap indexes " << std::dec<< *it << " " << *it2 << endl;
							}
							else cout << "No overlap, radii mismatch " << cyl1->radius << " " << cyl2->radius << endl;
						}
					}
					else {
						// cout << "No overlapping match -- indexes " << std::dec << *it << " " << *it2 << " ";
						// cout << "pos1 pos2 " << surfArray[*it]->position << " " << surfArray[*it2]->position;
						// cout << " axis1 axis2 " << surfArray[*it]->axis << " " << surfArray[*it2]->axis;
						// cout << " size1 size2 " << surfArray[*it]->size() << " " << surfArray[*it2]->size() << endl;
					}
				}
			} // end for
		}
	}
}


void AssemblyInfo::findHoles(shapeList *sList) {
	// Now check for left-over cylinders that might be holes
	// For now, only allow cylindrical holes
	for (vector<int>::iterator it = sList->shapeInds.begin(); it != sList->shapeInds.end(); ++it)
	{
		if (surfArray[*it]->wasOutput == false) {
			// Leftover cylinder -- actually a hole
			for (centerSurfIndArray::iterator edInd = edgeList.begin(); edInd != edgeList.end(); ++edInd) {
				if (edInd->second != *it && surfArray[edInd->second]->wasOutput && partArray[surfArray[edInd->second]->partInd].inBoolSolid == false) {
					if (surfArray[*it]->compNameStr.compare(surfArray[edInd->second]->compNameStr) == 0 &&
						edInd->first == surfArray[*it]->position && surfArray[*it]->holeSize() < surfArray[edInd->second]->size()) {
						// Found matching edge for shape that was output
						cout << "Found hole -- shape " << *it << " edge " << edInd->second << endl;
						outputShapeSetPart(sList, *it, *it);
						partArray[surfArray[edInd->second]->partInd].inBoolSolid = true;
						pair<int, int> baseHoleInds = make_pair(edInd->second, *it);
						holeList.push_back(baseHoleInds);
						break;
					}
					else {
						// cout << "Hole doesn't match edge. Hole pos = " << surfArray[*it]->position << " edge pos = " << edInd->first;
						// cout << " hole ind = " << *it << " edge owner ind = " << edInd->second << endl;
					}
				}
			}
			if (surfArray[*it]->wasOutput == false)
				cout << "Left-over surface can't find matching edge: " << *it << endl;
		} // end if
	} // end for
}


void AssemblyInfo::getInitRot(const int index) {
	/*
	cout << "Init rot index and main axis " << index << " " << surfArray[index]->axis << " start axis " << surfArray[index]->startAxis << endl;
	coords transfAngle = rotAnglesZYX(surfArray[index]->axis, zaxis); // Find how to rotate object axis into z
	cout << "Angle for axis into z " << transfAngle << endl;
	coords transfXaxis = rotVecZYX(surfArray[index]->startAxis, transfAngle); // Perform this on startAxis
	cout << "Position of startAxis after applying angle " << transfXaxis << endl;
	coords angleForX = rotAnglesZYX(xaxis, transfXaxis); // Find how to rotate x-axis to position for start
	*/
	double sanChk = checkZero(dotProd(surfArray[index]->startAxis, surfArray[index]->axis));
	if (sanChk != 0.0) {
		cout << "*** startAxis and axis not perpendicular! Can't get initial rotation!\n";
		return;
	}
	coords chkAxis = rotVecZYX(xaxis, surfArray[index]->rotation); // Apply rotation to x-axis to see where it lands
	// cout << "x-axis lands here " << chkAxis << " but should be here " << surfArray[index]->startAxis << endl;
	// cout << "Intended axis is " << surfArray[index]->axis << endl;
	sanChk = checkZero(dotProd(chkAxis, surfArray[index]->axis));
	if (sanChk != 0.0) {
		cout << "*** chkAxis and axis not perpendicular! Even at wrong angle, should still be perpendicular!\n";
		return;
	}
	double remainAngle = dotProd(chkAxis, surfArray[index]->startAxis);
	if (chkAxis.length() > 0 && surfArray[index]->startAxis.length() > 0) {
		remainAngle = remainAngle / (chkAxis.length() * surfArray[index]->startAxis.length());
		// cout << "do arccos on " << remainAngle << endl;
		if (remainAngle > 1.0) // Slight inaccuracy can cause acos to give nan
			remainAngle = 1.0;
		else if (remainAngle < -1.0)
			remainAngle = -1.0;
		remainAngle = acos(remainAngle);
	} 
	else {
		cout << "Invalid null final x-axis -- failing to rotate axis. Bad rotation = " << surfArray[index]->rotation << endl;
		return;
	}
	remainAngle = checkZero(remainAngle);
	if (remainAngle != 0.0) {
		// cout << "remainAngle is " << remainAngle << endl;
		int factor = 1;
		double chkAngle = -1.0;
		coords tryRot = surfArray[index]->rotation;
		for (int cnt = 0; cnt < 2 && chkAngle != 0.0; ++cnt) {
			tryRot.z = remainAngle * factor;
			// cout << "Applying rotation " << tryRot << " to " << xaxis << endl;
			coords chkAxis2 = rotVecZYX(xaxis, tryRot); // Apply rotation to x-axis to see where it lands
			chkAngle = dotProd(chkAxis2, surfArray[index]->startAxis);
			if (chkAxis2.length() > 0)
				chkAngle = checkZero(((chkAngle / (chkAxis2.length() * surfArray[index]->startAxis.length())) - 1.0), 0.001);
			else {
				cout << "Invalid null test final x-axis -- failing to rotate axis, bad rotation = " << tryRot << endl;
				return;
			}
			factor = -1;
			if (chkAngle != 0.0 && cnt > 0) {
				cout << "Destination axis in wrong place = " << chkAxis2 << " with dotProd = " << chkAngle + 1 << endl;
			}
		}
		if (chkAngle == 0.0) {
			surfArray[index]->rotation.z = tryRot.z;
			// cout << "rotation is now " << surfArray[index]->rotation << endl;
		}
		else cout << "Couldn't find any rotation for remaining angle = " << remainAngle << " check cos " << chkAngle + 1 << endl;
	}
	else cout << "Zero remain angle, no rotation adjustment needed\n";
	/*
	if (approxEqual(angleForX.x, 0) == false || approxEqual(angleForX.y, 0) == false || approxEqual(surfArray[index]->rotation.z, 0) == false) {
		cout << "Bad calculation of z rot = " << angleForX << " for rotation " << surfArray[index]->rotation;
		cout << " surfind " << index << endl;
		cout << "Final axis and start axis = " << surfArray[index]->axis << " " << surfArray[index]->startAxis << endl;
		cout << "transfAngle and transf axis = " << transfAngle << " " << transfXaxis << endl;
	}
	else surfArray[index]->rotation.z = angleForX.z;
	*/
}


void AssemblyInfo::outputParts()
{
	for (int index = 0; index <= surfArrayInd; ++index)
	{
		if (surfArray[index]->overallRot.length() > 0) { // Apply overall rotation for this component
			surfArray[index]->axis = rotVecZYX(surfArray[index]->axis, surfArray[index]->overallRot);
			if (surfArray[index]->startAxis.length() > 0)
				surfArray[index]->startAxis = rotVecZYX(surfArray[index]->startAxis, surfArray[index]->overallRot);
		}
		surfArray[index]->rotation = rotAnglesZYX(zaxis, surfArray[index]->axis);
		// Partial disks and cylinders and boards need initial rotation
		if (surfArray[index]->startAxis.length() > 0 && (surfArray[index]->getAngle() != (2.0 * M_PI) ||
				(surfArray[index]->surfID == PLANE_ID && surfArray[index]->getSubType() == BOARD_ID))) {
			getInitRot(index);
		}
		cout << "Index " << std::dec << index << " axis " << surfArray[index]->axis;
		cout << " rotation " << surfArray[index]->rotation << endl;
	}
	cout << "Output cones\n";
	outputSolids(&coneList);
	cout << "Output toruses\n";
	outputSolids(&torusList);
	cout << "Output ellipsoids\n";
	outputSolids(&ellipsoidList);
	cout << "Output disks\n";
	outputSolids(&diskList);
	cout << "Output boards\n";
	outputSolids(&boardList);
	cout << "Output cylinders\n";
	outputSolids(&cylList, true); // Allow matches with differ radii but same lengths
	breakUpFaces(&cylList);	// Inner & outer faces may have different lengths
	findHoles(&cylList);	// For now, only cylinders can be holes.
	// After all shapes processed, create holes.
	for (vector<pair<int, int>>::iterator holeInd = holeList.begin(); holeInd != holeList.end(); ++holeInd) {
		outputHole(holeInd->first, holeInd->second);
	}
	outputSolids(&cylList, false, true); // Find bulk cylinders with only single face
	xmlElem beginEnd, attrib1, attrib2, attrib3;
	gdmlout << beginEnd.closeSepElem("solids") << endl << endl;
	gdmlout << beginEnd.openSepElem("structure") << endl;
	for (vector<partDesc>::iterator it = partArray.begin(); it != partArray.end(); ++it) {
		if (it->createVol) {
			it->volumeName = nameIncr(VOLUME_ID);
			gdmlout << indent1 << beginEnd.openSepElem("volume", attrib1.attribute("name", it->volumeName.c_str())) << endl;
			gdmlout << indent2 << beginEnd.openElem("materialref") << attrib1.attribute("ref", it->matNameStr.c_str());
			gdmlout << beginEnd.closeElem() << endl;
			gdmlout << indent2 << beginEnd.openElem("solidref") << attrib1.attribute("ref", it->nameID.c_str());
			gdmlout << beginEnd.closeElem() << endl;
			gdmlout << indent1 << beginEnd.closeSepElem("volume") << endl;
		}
	}
	gdmlout << indent1 << beginEnd.openSepElem("volume", attrib1.attribute("name", "World")) << endl;
	gdmlout << indent2 << beginEnd.openElem("materialref") << attrib1.attribute("ref", "Air");
	gdmlout << beginEnd.closeElem() << endl;
	gdmlout << indent2 << beginEnd.openElem("solidref") << attrib1.attribute("ref", "WorldBox");
	gdmlout << beginEnd.closeElem() << endl;
	for (vector<partDesc>::iterator it = partArray.begin(); it != partArray.end(); ++it) {
		if (it->createVol) {
			gdmlout << indent2 << beginEnd.openSepElem("physvol") << endl;
			gdmlout << indent3 << beginEnd.openElem("volumeref") << attrib1.attribute("ref", it->volumeName.c_str());
			gdmlout << beginEnd.closeElem() << endl;
			string pos("center");
			// coords relPos = surfArray[it->surfInd]->position - surfArray[partArray.begin()->surfInd]->position;
			coords relPos = surfArray[it->surfInd]->position;
			if (surfArray[it->surfInd]->overallRot.length() > 0) { // Apply overall rotation for this component
				double posLen = relPos.length();
				cout << "index " << it->surfInd << " RelPos rotates from " << relPos;
				coords revRot = surfArray[it->surfInd]->overallRot;
				relPos = rotVecZYX(relPos, revRot);
				if (posLen != relPos.length()) {
					cout << "relPos len changed from " << posLen << " to " << relPos.length() << endl;
					relPos.normalize();
					relPos = relPos * posLen;
				}
				cout << " to " << relPos << endl;
			}
			// relPos = surfArray[it->surfInd]->position + surfArray[it->surfInd]->displace;
			if (surfArray[it->surfInd]->displace.length() > 0)
				cout << "Apply displacement displace: " << surfArray[it->surfInd]->displace << ", pos " << relPos << endl;
			relPos = relPos + surfArray[it->surfInd]->displace;
			gdmlout << indent3 << beginEnd.openElem("position") << attrib1.attribute("name", nameIncr(POSITION_ID));
			gdmlout << attrib1.attribute("x", relPos.x * units ) << attrib2.attribute("y", relPos.y * units) << attrib3.attribute("z", relPos.z * units);
			gdmlout << beginEnd.closeElem() << endl;
			gdmlout << indent3 << beginEnd.openElem("rotation") << attrib1.attribute("name", nameIncr(ROTATION_ID));
			gdmlout << attrib1.attribute("z", surfArray[it->surfInd]->rotation.z) << attrib2.attribute("y", surfArray[it->surfInd]->rotation.y);
			gdmlout << attrib3.attribute("x", surfArray[it->surfInd]->rotation.x);
			gdmlout << beginEnd.closeElem() << endl;
			gdmlout << indent2 << beginEnd.closeSepElem("physvol") << endl;
		}
	}
	gdmlout << indent1 << beginEnd.closeSepElem("volume") << endl;
	gdmlout << beginEnd.closeSepElem("structure") << endl << endl;
}

static void testrot()
{
	coords start, end;
	start.x = 0.0;
	start.y = 0.0;
	start.z = 1.0;
	end.x = 1.0;
	end.y = 0.0;
	end.z = 0.0;
	coords result = rotAnglesZYX(start, end);
	cout << " 0,0,1 to 1,0,0 is angle " << result.x << "," << result.y << "," << result.z << endl;
	start.x = 1.0;
	start.y = 1.0;
	start.z = 1.0;
	end.x = -1.0;
	end.y = -1.0;
	end.z = 1.0;
	result = rotAnglesZYX(start, end);
	cout << " 1,1,1 to -1,-1,1 is angle " << result.x << "," << result.y << "," << result.z << endl;
	start.x = 3.0;
	start.y = 2.0;
	start.z = 1.0;
	end.x = -1.0;
	end.y = -3.0;
	end.z = -2.0;
	result = rotAnglesZYX(start, end);
	cout << " 3,2,1 to -1,-3,-2 is angle " << result.x << "," << result.y << "," << result.z << endl;
}


const char *getTypeStr(long typeNum) {
	switch (typeNum) {
	case swMateCOINCIDENT:
		return ("conincident");
	case swMateCOORDINATE:
		return ("coordinate");
	case swMateANGLE:
		return ("angle");
	}
	return ("other");
}

const char *getAlignStr(long alignNum) {

	switch (alignNum) {
	case swMateAlignALIGNED:
		return ("aligned");
	case swMateAlignANTI_ALIGNED:
		return ("anti-aligned");
	case swMateAlignCLOSEST:
		return ("closest");
	}
	return ("unknown");
}


void AssemblyInfo::showMate(IMate2 *matePtr, coords &antiAlignTot, int &alignCnt,
	coordList &displaceList, int mateInd, bool &adjustDisplace, int mateCnt, bool &meRadiiSame)
{
	// cout << "Found Mate2\n";
	bool overallRotUnset = (overallRot.length() == 0);
	long align = -1;
	hres = matePtr->get_Alignment(&align);
	cout << "Alignment = " << getAlignStr(align) << endl;
	long mateType = -1;
	hres = matePtr->get_Type(&mateType);
	cout << "Type = " << getTypeStr(mateType) << endl;
	long paramsSize = 0;
	hres = matePtr->GetMateEntityCount(&paramsSize);
	if (hres == S_OK) {
		// cout << "MateEntity2 count = " << paramsSize << endl;
		if (paramsSize > 0) {
			coords newAxis, direction1, location1;
			double radius1 = 0.0;
			for (long index = 0; index < paramsSize; ++index) {
				IMateEntity2 *mateEntity = NULL;
				hres = matePtr->MateEntity(index, &mateEntity);
				if (hres == S_OK && mateEntity) {
					// cout << "Found mate entity\n";
					long arraySize = 0;
					hres = mateEntity->GetEntityParamsSize(&arraySize);
					if (hres == S_OK) {
						// cout << "Entity param size = " << arraySize << endl;
						if (arraySize > 0) {
							long mateRefType = -1;
							hres = mateEntity->get_ReferenceType2(&mateRefType);
							cout << "Mate Reference type " << mateRefType << endl;
							VARIANT entityParams;
							VariantInit(&entityParams);
							hres = mateEntity->get_EntityParams(&entityParams);
							if (hres == S_OK) {
								long paramBegin, paramLim;
								double *paramArray = varDblArrayAccess(entityParams, paramBegin, paramLim);
								if (paramLim - paramBegin >= 7) {
									coords location(paramArray[0], paramArray[1], paramArray[2]);
									coords direction(paramArray[3], paramArray[4], paramArray[5]);
									double meRadius = paramArray[6];
									cout << "MateEntity location = " << location << endl;
									cout << "MateEntity direction = " << direction << endl;
									cout << "MateEntity radii 1 and 2 = " << meRadius << " " << paramArray[7] << endl;
									int rank = displaceList[location];
									// cout << "Location " << location << " current rank " << ++rank << endl;
									displaceList[location] = ++rank;
									// (displaceList[location])++;
									if (align == swMateAlignCLOSEST) {
										// displace = displace - location;
										if (index == 0) {
											newAxis = direction;
											newAxis.normalize();
										}
										else if (index == 1 && direction == xaxis && newAxis.length() > 0) {
											newAxis = newAxis * -1.0;
											cout << "New axis is = " << newAxis << endl;
											overallRot = rotAnglesZYX(xaxis, newAxis);
											cout << "Overall rotation set to = " << overallRot << endl;
										}
										else cout << "Expecting std. xaxis to new axis, with 2 mate entities; index, base axis = " <<
											index << " " << newAxis << endl;
									}
									else if (align == swMateAlignALIGNED) {
										// Give precedence to 1st Datumplane or second aligned entry when there are 2 or fewer mates
										if ((index == 0 && mateRefType == swSelDATUMPLANES) ||
												(mateCnt < 3 && index == 1)) {
											adjustDisplace = false;
											displace = location;
										}
										else if (adjustDisplace) {
											if ((alignCnt / 2) % 2 == 1) // Subtract every other ALIGNED mate pair
												location = location * -1.0;
											displace = displace + location;
											++alignCnt;
										}
									}
									else if (align == swMateAlignANTI_ALIGNED) {
										if (index == 0) {
											if (mateRefType == swSelDATUMPLANES) { // Anti-aligned datumplane seems to take precedence
												displace = location;
												newAxis = direction;
												newAxis.normalize();
												newAxis = newAxis * -1.0;
												cout << "New axis is = " << newAxis << endl;
												overallRot = rotAnglesZYX(xaxis, newAxis);
												cout << "Overall rotation set to = " << overallRot << endl;
												adjustDisplace = false;
											} else location = location * -1.0;
										} else if (mateCnt < 3 && overallRot.length() == 0 && direction.length() > 0 && direction1.length() > 0) {
											// coords chkAxis = direction * -1.0;
											// if (!(chkAxis == direction1)) { // Make sure directions are not trivial flip
												// Axis flip can orient parts in reverse of correct direction
												newAxis = direction;
												newAxis.normalize();
												overallRot = rotAnglesZYX(direction1, newAxis);
												cout << "Overall rotation set to = " << overallRot << endl;
											// }
											// else cout << "Ignoring trivial axis flip " << direction << " into " << direction1 << endl;
										}
										antiAlignTot = antiAlignTot + location;
									}
									if (index == 0) {
										direction1 = direction;
										location1 = location;
										radius1 = meRadius;
									}
									else {
										if (meRadius > 0) {	// First radius seems to be an additional displacement
											if (location == location1) {
												if (meRadiiSame) {
													meRadiiSame = approxEqual(meRadius, radius1);
													if (meRadiiSame) { // Only adjust if radii were consistent for mate pairs
														double adjDisplace = meRadius * 4.0;
														if (adjDisplace < 1.0)	// Radius value may need adjustment, but not too big
															meRadius = adjDisplace;
														displace.y += meRadius;
													}
												}
											} 											
											else displace.z -= meRadius;
											cout << "May be using MateEntity radius 1 = " << meRadius << ", displace is now " << displace << endl;
										}
										if (overallRot.length() == 0 && (direction == xaxis || direction == xaxisminus)) {
											if (direction1 == xaxisminus || (location1.length() > 0 && direction1 == xaxis)) {
												// Pair of +-x axes (same or different) seems to indicate x should be rotated into -z
												// but only if mate isn't empty (first seen in SolidWorks 2014
												coords begAxis = xaxis;
												if (mateInd > 1)	// 3rd mate seems to indicate should rotate from y instead of x
													begAxis = yaxis;
												overallRot = rotAnglesZYX(begAxis, zaxis);
												cout << "Overall rotation set to = " << overallRot << " because of +-x pair " << endl;
											}
										}
										// else cout << "Direction1 " << direction1 << ", direction2 " << direction << " overall " << overallRot << endl;
									}
								}
								/*
								struct IDispatch *pMateRef = NULL;
								hres = mateEntity->get_Reference(&pMateRef);
								if (hres == S_OK && pMateRef) {
									IEntity *mateRefPtr = NULL;
									hres = pMateRef->QueryInterface(__uuidof(IEntity), reinterpret_cast<void**>(&mateRefPtr));
									if (hres == S_OK && mateRefPtr) {
										CComBSTR sRefName;
										hres = mateRefPtr->get_ModelName(&sRefName);
										if (hres == S_OK && sRefName) {
											printbstr("Entity model name", sRefName);
										}
										else cout << "Couldn't get model name hres ptr " << std::hex << hres << " " << sRefName << endl;
									}
									else cout << "Couldn't convert Mate Reference hres ptr " << std::hex << hres << " " << mateRefPtr << endl;
								}
								else cout << "Couldn't get Mate Reference hres ptr " << hres << " " << pMateRef << endl;
								*/
								IComponent2* swComponentNxt = NULL;
								hres = mateEntity->get_ReferenceComponent(&swComponentNxt);
								if (hres == S_OK && swComponentNxt) {
									CComBSTR sCmpName;
									hres = swComponentNxt->get_Name2(&sCmpName);
									if (hres == S_OK && sCmpName) {
										printbstr("comp name ", sCmpName);
										if (overallRotUnset && overallRot.length() > 0 && align == swMateAlignANTI_ALIGNED && mateInd == 0) {
											// Make sure overallRot was just set by this mate
											// Only restrict overall rotation if it comes from first anti-aligned mate
											overallRotCmpNmStr = CW2A(sCmpName);
										}
									}
								}
								else cout << "Couldn't get mate entity ref comp hres ptr " << hres << " " << swComponentNxt << endl;
							}
							else cout << "Couldn't get entity params hres = " << hres << endl;
						}
					}
				}
				else cout << "Couldn't find mate entity\n";
			} // end for loop of mate entities
		}
	}
	else cout << "Couldn't get MateEntity2 from mate hres ptr " << hres << " " << matePtr << endl;
}

/*
void AssemblyInfo::showMateEntity(CComPtr<IFeature> swFeature) {
	CComPtr<IFeature> swSubFeature;
	hres = swFeature->IGetFirstSubFeature(&swSubFeature);
	while (hres == S_OK && swSubFeature) {
		cout << "Found sub-feature\n";
		CComBSTR sGetFeatureName;
		hres = swSubFeature->get_Name(&sGetFeatureName);
		CW2A featurenam(sGetFeatureName);
		if (hres == S_OK && sGetFeatureName) {
			cout << "sub-feature name " << featurenam << endl;
		}
		struct IDispatch *pMate2;
		hres = swSubFeature->GetSpecificFeature2(&pMate2);
		if (hres == S_OK && pMate2) {
			IMate2 *matePtr = NULL;
			hres = pMate2->QueryInterface(__uuidof(IMate2), reinterpret_cast<void**>(&matePtr));
			if (hres == S_OK && matePtr) {
				showMate(matePtr);
			}
		}
		cout << "Getting next subfeature\n";
		CComPtr<IFeature> swNxtSubFeature;
		hres = swFeature->IGetNextSubFeature(&swNxtSubFeature);
		swSubFeature = swNxtSubFeature;
	} // end while
	cout << "Couldn't find sub-feature hres ptr " << hres << " " << swSubFeature << endl;
}
*/


void AssemblyInfo::getMates(IComponent2 *swSelectedComponent) {
	displace = origin; // Clear values from previous component
	overallRot = origin;
	overallRotCmpNmStr.clear();
	VARIANT mateList;
	VariantInit(&mateList);
	long nMateCount = -1;
	hres = swSelectedComponent->GetMates(&mateList);
	if (hres == S_OK) {
		if (hres == S_OK && mateList.pparray != NULL) {
			cout << "Found mate list\n";
			SAFEARRAY* psaMate = V_ARRAY(&mateList);
			LPDISPATCH* pMateDispArray = NULL;
			long nMateHighIndex = -1;
			hres = SafeArrayAccessData(psaMate, (void **)&pMateDispArray);
			if (hres == S_OK && pMateDispArray != NULL) {
				// Get index number of highest array element
				// The array range is from 0 to highIndex
				hres = SafeArrayGetUBound(psaMate, 1, &nMateHighIndex);
				if (hres == S_OK) {
					// Actual number of array elements is nBodyHighIndex + 1
					nMateCount = nMateHighIndex + 1;
					cout << "num mates " << std::dec << nMateCount << endl;
					coords antiAlignTot;
					int alignCnt = 0;
					coordList displaceList;
					bool adjustDisplace = true;
					bool meRadiiSame = true;
					for (int i = 0; i < nMateCount; i++) {
						CComQIPtr <IMate2>  pMate;
						// Calls AddRef() on IDispatch ---> refcount = 2
						pMate = pMateDispArray[i];
						if (pMate != NULL) {
							showMate(pMate, antiAlignTot, alignCnt, displaceList, i, adjustDisplace, nMateCount, meRadiiSame);
						}
					}
					if (adjustDisplace) {
						displace = displace + antiAlignTot;
						if (displaceList.size() > 0) {
							// cout << "Displace list size = " << displaceList.size() << endl;
							int rank = -1;
							coords betterDisplace;
							for (coordList::const_iterator ind = displaceList.begin(); ind != displaceList.end(); ++ind) {
								int newRank = ind->second;
								if (rank < newRank) {
									rank = newRank;
									betterDisplace = ind->first;
								}
							}
							if (rank > 3) {
								displace = betterDisplace;
								// cout << "Displacement replaced by rank " << rank << endl;
							}
						}
					}
					cout << "Final displacment = " << displace << " aatot = " << antiAlignTot << endl;

				}
			}
		}
		else cout << "No mate list found\n";
	}
	else cout << "Couldn't get mate list from component hres = " << hres << endl;
	if (after1stComp == false) {
		if (nMateCount > 4)
			displace = origin;
		// Keep 1st displacement but don't trust large number of mates, but ignore 1st rotation
		overallRot = origin;
		overallRotCmpNmStr.clear();
		after1stComp = true;
	}
}


static void processComp(IComponent2* swSelectedComponent, CComPtr<ISelectData> &swSelData,
		AssemblyInfo &assembly) {
	CComPtr<IMeasure> mymeasure;
	bool measok = false;
	hres = swSelectedComponent->Select4(VARIANT_FALSE, swSelData, VARIANT_TRUE, &retVal);
	cout << "comp select4 results hres retval " << std::hex << hres << " " << retVal << endl;
	// hres = swSelectedComponent->IsHidden();
	struct IDispatch *swModelNxt = NULL;
	IModelDoc2 *nxtmodel = NULL;
	IPartDoc *partptr = NULL;
	hres = swSelectedComponent->GetModelDoc2(&swModelNxt);
	if (hres == S_OK && swModelNxt) {
		cout << "trying model of component\n";
		hres = swModelNxt->QueryInterface(__uuidof(IModelDoc2), reinterpret_cast<void**>(&nxtmodel));
		if (hres == S_OK && nxtmodel) {
			cout << "trying part of component\n";
			hres2 = swModelNxt->QueryInterface(__uuidof(IPartDoc), reinterpret_cast<void**>(&partptr));
			CComBSTR dbase(L"");
			if (hres2 == S_OK && partptr) {
				cout << "got part, getting material\n";
				CComBSTR materialName(L"");
				hres = partptr->GetMaterialPropertyName2(L"Default", &dbase, &materialName);
				strcpy_s(assembly.matNameStr, CW2A(materialName));
				char *spc = NULL;
				char *spot = assembly.matNameStr;
				while ((spc = strpbrk(spot, " ()*")) != NULL) {
					// Convert disallowed characters to avoid Geant4 errors
					switch (*spc) {
					case ' ':
						*spc = '_';
						break;
					case '(':
					case ')':
						*spc = '.';
						break;
					default:
						*spc = '-';
					}
					spot = spc + 1;
				}
				cout << "mat name " << assembly.matNameStr << endl;
				printbstr("db name ", dbase);
			}
			CComPtr<IModelDocExtension> swModelDocExt;
			double val = 0;
			hres = nxtmodel->get_Extension(&swModelDocExt);
			if (hres == S_OK && swModelDocExt) {
				CComPtr<IMassProperty> mymassprop;
				hres = swModelDocExt->CreateMassProperty(&mymassprop);
				if (hres == S_OK && mymassprop) {
					hres = mymassprop->get_Density(&val);
					if (hres == S_OK) {
						cout << "dens [kg/m^3] " << val << endl;
						hres = mymassprop->get_Mass(&val);
						cout << "mass [kg] " << val << endl;
						if (hres == S_OK)
							hres = mymassprop->get_SurfaceArea(&val);
						cout << "surf area [m^2] " << val << endl;
						if (hres == S_OK)
							hres = mymassprop->get_Volume(&val);
						cout << "vol [m^3] " << val << endl;
					}
				}
				CComBSTR sNullStr(L"");
				// hres = swModelDocExt->SelectByID2(sBodyDesc, sNullStr, 0.0, 0.0, 0.0, VARIANT_FALSE, 13, NULL, swSelectOptionDefault, &retVal);
				// cout << " select hres retval " << std::hex << hres << " " << retVal << endl;
				// Appears to be important for opening multiple parts
				if (true || hres == S_OK) {
					hres = swModelDocExt->CreateMeasure(&mymeasure);
					if (hres == S_OK && mymeasure) {
						measok = true;
						// calcmeasure(mymeasure, oneface);
					}
				}
				else cout << "select failed hres " << std::hex << hres << endl;
			}
			// TraverseFeatureManagerDesignTree(nxtmodel, &swComponentNxt);
		}
		else cout << "can't find nxt model\n";
		VARIANT bodyinfo, bodyvar;
		VariantInit(&bodyinfo);
		VariantInit(&bodyvar);
		hres = swSelectedComponent->GetBodies3(swAllBodies, &bodyinfo, &bodyvar);
		if (hres == S_OK && bodyvar.pparray != NULL) {
			SAFEARRAY* psaBody = V_ARRAY(&bodyvar);
			LPDISPATCH* pBodyDispArray = NULL;
			long nBodyHighIndex = -1;
			long nBodyCount = -1;
			hres = SafeArrayAccessData(psaBody, (void **)&pBodyDispArray);
			if (hres == S_OK && pBodyDispArray != NULL) {
				// Get index number of highest array element
				// The array range is from 0 to highIndex
				hres = SafeArrayGetUBound(psaBody, 1, &nBodyHighIndex);
				if (hres == S_OK) {
					// Actual number of array elements is nBodyHighIndex + 1
					nBodyCount = nBodyHighIndex + 1;
					cout << "num bodies " << nBodyCount << endl;
					for (int i = 0; i < nBodyCount; i++) {
						CComQIPtr <IBody2>  pBody;
						// Calls AddRef() on IDispatch ---> refcount = 2
						pBody = pBodyDispArray[i];
						if (pBody != NULL) {
							hres = pBody->Select(VARIANT_TRUE, 0, &retVal);
							if (hres == S_OK) {
								CComBSTR sBodyDesc(L"");
								hres = pBody->get_Name(&sBodyDesc);
								if (hres == S_OK)
									printbstr("body name ", sBodyDesc);
								hres = pBody->GetSelectionId(&sBodyDesc);
								if (hres == S_OK)
									printbstr("body ID ", sBodyDesc);
								long numFaces = 0;
								hres = pBody->GetFaceCount(&numFaces);
								if (hres == S_OK)
									cout << "num faces " << numFaces << endl;
								VARIANT facearray;
								VARIANT oneface;
								VariantInit(&facearray);
								hres = pBody->GetFaces(&facearray);
								cout << "getfaces hres pparray " << std::hex << hres << " " << facearray.pparray << endl;
								VariantInit(&oneface);
								oneface.vt = facearray.vt;
								int facetype = facearray.vt & ~(VT_ARRAY);
								cout << "facetype & fvt are " << facetype << " " << oneface.vt << endl;
								if (measok && facetype > 0) {
									SAFEARRAY *facesview = V_ARRAY(&facearray);
									LPDISPATCH *srcptr = NULL;
									long lStartBound = 0;
									long lEndBound = 0;
									SafeArrayGetLBound(facesview, 1, &lStartBound);
									SafeArrayGetUBound(facesview, 1, &lEndBound);
									hres = SafeArrayAccessData(facesview, (void**)&srcptr);
									if (hres == S_OK && srcptr) {
										// int faceIndList[20], fcInd = 0;
										for (int iIndex = lStartBound; iIndex <= lEndBound; iIndex++) {
											double radius = 0.0;
											long surfID = -1;
											oneface.parray = assignvararray(facetype, srcptr, iIndex);
											assembly.showFaceDetails(srcptr, iIndex, &radius, &surfID, mymeasure);
											if (oneface.parray != NULL) {
												double unused = 0.0;
												assembly.calcmeasure(mymeasure, oneface, radius, surfID, unused);
												if (surfID != PLANE_ID && surfID != TORUS_ID && surfID != S_REVOLVE_ID)
													assembly.centerPosition();
												else cout << "Position " << assembly.surfArray[assembly.surfArrayInd]->position << endl;
												SafeArrayDestroy(oneface.parray);
											}
										}  // end for
									}
									else cout << "bad ptr h  s " << std::hex << hres << " " << srcptr << endl;
									SafeArrayUnaccessData(facesview);
									SafeArrayDestroy(facesview);
								}
								else cout << "bad face type\n";
								hres = pBody->DeSelect(&retVal);
								cout << "DeSelect body hres retVal " << hres << " " << retVal << endl;
								cout << endl; // Blank line to separate bodies
							}
							else cout << "couldn't select body " << endl;
							// After this call ---> refcount = 1
							// When it goes out of scope ---> refcount = 0
							// pBody.Release();
						}
					}
				}
			}
			hres = SafeArrayUnaccessData(psaBody);
			hres = SafeArrayDestroy(psaBody);
		}
		else {
			cout << "getbodies failed hres pparray " << std::hex << hres << " " << bodyvar.pparray << endl;
			// getChildComponents(swSelectedComponent);
		}
	} else cout << "failed to get model doc hres = " << hres << " ptr = " << swModelNxt << endl;
}


static void procComponents(IComponent2* swSelectedComponent, CComPtr<ISelectData> &swSelData,
	AssemblyInfo &assembly, bool getMates = true) {
	static bool debugAllow = true;
	CComBSTR sPathName(L"");
	hres = swSelectedComponent->GetPathName(&sPathName);
	if (hres == S_OK && sPathName) {
		printbstr("comp path name ", sPathName);
		assembly.pathNameStr = CW2A(sPathName);
	}
	CComBSTR sCmpName(L"");
	hres = swSelectedComponent->get_Name2(&sCmpName);
	if (hres == S_OK && sCmpName) {
		printbstr("comp name ", sCmpName);
		assembly.compNameStr = CW2A(sCmpName);
	}
	int numChil = 0;
	hres = swSelectedComponent->IGetChildrenCount(&numChil);
	cout << "num children " << numChil << endl;
	if (getMates)
		assembly.getMates(swSelectedComponent);
	if (numChil > 0) {
		debugAllow = true;
		VARIANT compArray;
		VariantInit(&compArray);
		hres = swSelectedComponent->GetChildren(&compArray);
		if (hres == S_OK && compArray.pparray != NULL) {
			SAFEARRAY* psaComp = V_ARRAY(&compArray);
			LPDISPATCH* pCompDispArray = NULL;
			long nCompHighIndex = -1;
			long nCompCount = -1;
			hres = SafeArrayAccessData(psaComp, (void **)&pCompDispArray);
			if (hres == S_OK && pCompDispArray != NULL) {
				// Get index number of highest array element
				// The array range is from 0 to highIndex
				hres = SafeArrayGetUBound(psaComp, 1, &nCompHighIndex);
				if (hres == S_OK) {
					// Actual number of array elements is nBodyHighIndex + 1
					nCompCount = nCompHighIndex + 1;
					cout << "num components " << nCompCount << endl;
					for (int i = 0; i < nCompCount; i++) {
						CComQIPtr <IComponent2>  pComp;
						pComp = pCompDispArray[i];
						if (pComp != NULL) {
							CComBSTR sCmpName(L"");
							hres = pComp->get_Name2(&sCmpName);
							if (hres == S_OK && sCmpName) {
								printbstr("child comp name ", sCmpName);
							}
							procComponents(pComp, swSelData, assembly, false);
						}
						else cout << "Bad component ptr\n";
					}
				}
			}
			else cout << "Failed to access component array -- hres " << std::hex << hres << " ptr " << pCompDispArray << endl;
			SafeArrayUnaccessData(psaComp);
			SafeArrayDestroy(psaComp);
		}
		else cout << "Failed to get component array -- hres " << std::hex << hres << " ptr " << compArray.pparray << endl;
	}
	else {
		if (debugAllow)
			processComp(swSelectedComponent, swSelData, assembly);
	}
}


static coords getVarCoords(CComPtr<IMathVector> transfCoords, const char *const msg) {
	coords retCoord;
	VARIANT theCoord;
	VariantInit(&theCoord);
	hres = transfCoords->get_ArrayData(&theCoord);
	if (hres == S_OK) {
		SAFEARRAY *coordView = V_ARRAY(&theCoord);
		double *coordptr = NULL;
		long lStartBound = 0;
		long lEndBound = 0;
		SafeArrayGetLBound(coordView, 1, &lStartBound);
		SafeArrayGetUBound(coordView, 1, &lEndBound);
		hres = SafeArrayAccessData(coordView, (void**)&coordptr);
		if (hres == S_OK && coordptr) {
			for (int ind = lStartBound; ind <= lEndBound - 2; ind += 3) {
				coords coordval(coordptr[ind], coordptr[ind + 1], coordptr[ind + 2]);
				cout << "Got " << msg << " = " << coordval << endl;
				retCoord = coordval;
			}
		}
		else cout << "Couldn't access variant array, hres = " << hres << endl;
		SafeArrayUnaccessData(coordView);
		SafeArrayDestroy(coordView);
	}
	else cout << "Couldn't get coord hres " << hres << endl;
	return (retCoord);
}


// Gets related components to Local Linear Pattern seed component and
// sets their displacement to match the pattern displacement
void AssemblyInfo::getRelatedComps(IComponent2 *baseComp) {
	CComBSTR sCmpName;
	hres = baseComp->get_Name2(&sCmpName);
	if (hres == S_OK && sCmpName)
		printbstr("seed comp name ", sCmpName);
	// string baseCmpName = CW2A(sCmpName);
	CComBSTR sPathName(L"");
	hres = baseComp->GetPathName(&sPathName);
	if (hres == S_OK && sPathName) {
		printbstr("seed comp path name ", sPathName);
		string seedPath = CW2A(sPathName);
		static const string patternType("ReferencePattern");
		static const string referenceType("Reference");
		string compName;
		unsigned int patternCnt = -1;
		for (vector<GenericSurf *>::iterator it = surfArray.begin(); it != surfArray.end(); ++it) {
			if ((*it)->pathNameStr.compare(seedPath) == 0) {
				if ((*it)->featureTypeStr.compare(referenceType) == 0) {
					if (overallRot.length() == 0 && (*it)->overallRot.length() != 0)	// Get overallRot from base seed component
						overallRot = (*it)->overallRot;
				} else if ((*it)->featureTypeStr.compare(patternType) == 0) {
					if (compName.size() == 0 || (*it)->compNameStr.compare(compName) != 0) {
						compName = (*it)->compNameStr;
						++patternCnt;
					}
					if ((*it)->overallRot.length() == 0)
						(*it)->overallRot = overallRot;		// Set from seed component
					if ((*it)->displace.length() == 0 && pattDisplaceList.size() > patternCnt)
						(*it)->displace = pattDisplaceList[patternCnt];
				}
			}	// end if match path name
		}	// end for
	}
}


// Find the seed component of the Local Linear Pattern and 
// gets related components to Local Linear Pattern seed component and
// sets their displacement to match the pattern displacement
void AssemblyInfo::getSeedComps(ILocalLinearPatternFeatureData *const linpattern) {
	VARIANT seedarr;
	VariantInit(&seedarr);
	hres = linpattern->get_SeedComponentArray(&seedarr);
	if (hres == S_OK) {
		SAFEARRAY *seedView = V_ARRAY(&seedarr);
		LPDISPATCH *seedptr = NULL;
		long lStartBound = 0;
		long lEndBound = 0;
		SafeArrayGetLBound(seedView, 1, &lStartBound);
		SafeArrayGetUBound(seedView, 1, &lEndBound);
		hres = SafeArrayAccessData(seedView, (void**) &seedptr);
		if (hres == S_OK && seedptr) {
			for (int ind = lStartBound; ind <= lEndBound; ind++) {
				CComQIPtr<IFeature> featptr = seedptr[ind];
				cout << "Found feature ptr " << featptr << endl;
				if (featptr) {
					struct IDispatch *pComp;
					hres = featptr->GetSpecificFeature2(&pComp);
					if (hres == S_OK && pComp) {
						IComponent2 *compPtr = NULL;
						hres = pComp->QueryInterface(__uuidof(IComponent2), reinterpret_cast<void**>(&compPtr));
						if (hres == S_OK && compPtr) {
							getRelatedComps(compPtr);
						}
						else cout << "Couldn't get comp, hres = " << hres << " ptr =  " << compPtr << endl;

					}
					else cout << "Couldn't get specific feature comp, hres = " << hres << " ptr =  " << pComp << endl;
				}
				else cout << "No feature ptr\n";
			}
		}
		else cout << "Couldn't access variant array, hres = " << hres << endl;
		SafeArrayUnaccessData(seedView);
		SafeArrayDestroy(seedView);
	}
	else cout << "Couldn't get seed hres " << hres << endl;
}


// Calculates the displacement pattern for the Local Linear Pattern.
// It gets the transformation matrix and uses it and the spacing to
// calculate the displacements of the three components.
// It appears the two base directions of the linear pattern are x (D1) and z (D2).
// Currently only support a pattern with 4 vertices.
void AssemblyInfo::getTransfDisplace(ILocalLinearPatternFeatureData *linpattern, double xspacing, double zspacing)
{
	pattDisplaceList.clear();
	overallRot = origin;	// Clear previous value
	overallRotCmpNmStr.clear();
	CComPtr<IMathTransform> transform;
	hres = linpattern->GetTransform(1, &transform);
	cout << "transform fetch hres = " << hres << " ptr = " << transform << endl;
	if (hres == S_OK && transform) {
		CComPtr<IMathVector> xtrans, ytrans, ztrans, translat;
		double scale = -1.0;
		hres = transform->IGetData2(&xtrans, &ytrans, &ztrans, &translat, &scale);
		if (hres == S_OK) {
			cout << "Got transf data \n";
			coords row;
			matrix3x3 pattRot;
			if (xtrans) {
				row = getVarCoords(xtrans, "x coord");
				pattRot.setRow(row);
			}
			if (ytrans) {
				row = getVarCoords(ytrans, "y coord");
				pattRot.setRow(row);
			}
			if (ztrans) {
				row = getVarCoords(ztrans, "z coord");
				pattRot.setRow(row);
			}
			if (translat) {
				coords xdir = pattRot * xaxis;
				xdir = xdir * -1.0;		// Direction seems to be reversed
				xdir.normalize();
				// overallRot = rotAnglesZYX(xaxis, xdir);
				// cout << "x to " << xdir << " is angle for overallRot" << overallRot << endl;
				coords zdir = pattRot * zaxis;
				zdir.normalize();
				coords pattDisplace = getVarCoords(translat, "translat");
				cout << "Pattern loc 1 displace = " << pattDisplace << endl;
				pattDisplaceList.push_back(pattDisplace);
				coords rotDisplaceX = pattDisplace + (xdir * xspacing);
				cout << "Pattern loc 2 displace = " << rotDisplaceX << endl;
				pattDisplaceList.push_back(rotDisplaceX);
				coords rotDisplaceZ = (zdir * zspacing);
				coords bothDisplace = rotDisplaceX + rotDisplaceZ;
				cout << "Pattern loc 3 displace = " << bothDisplace << endl;
				pattDisplaceList.push_back(bothDisplace);
			}
			cout << "Scale = " << scale << endl;
		}
	}

}

// ********* Finish this method. It should already find the Spacing
// What else is needed? What does spacing mean for circular pattern?
// Seed components are components that are related to the pattern.
template<class T>
double CircPattFuncs<T>::getSpacing(T *patternType, AssemblyInfo &assembly) {
	// Need similar code as for linear pattern, but circular pattern has different methods
	double spacing = 0;
	hres = thePattern->get_Spacing(&spacing);
	cout << "Circ spacing " << spacing << endl;
	return(spacing);
}

template<class T>
double LinPattFuncs<T>::getSpacing(T *thePattern, AssemblyInfo &assembly) {
	double xspacing = 0;
	hres = thePattern->get_D1Spacing(&xspacing);
	cout << "D1 spacing " << xspacing << endl;
	double zspacing = 0;
	hres = thePattern->get_D2Spacing(&zspacing);
	cout << "D2 spacing " << zspacing << endl;
	VARIANT_BOOL revDir = VARIANT_FALSE;
	hres = thePattern->get_D1ReverseDirection(&revDir);
	cout << "D1 reverse direction " << revDir << endl;
	long axisType = -1;
	hres = thePattern->GetD1AxisType(&axisType);
	cout << "D1 axis type is " << axisType << endl;
	hres = thePattern->get_D2ReverseDirection(&revDir);
	cout << "D2 reverse direction " << revDir << endl;
	hres = thePattern->GetD2AxisType(&axisType);
	cout << "D2 axis type is " << axisType << endl;
	assembly.getTransfDisplace(thePattern, xspacing, zspacing);
	long cnt = -1;
	hres = thePattern->GetSeedComponentCount(&cnt);
	cout << "Seed component count is " << cnt << endl;
	if (cnt > 0) {
		assembly.getSeedComps(thePattern);
	}
	hres = thePattern->GetSkippedItemCount(&cnt);
	cout << "Skipped item count is " << cnt << endl;
	hres = thePattern->get_D1TotalInstances(&cnt);
	cout << "D1 instances is " << cnt << endl;
	long d2Cnt = -1;
	hres = thePattern->get_D2TotalInstances(&d2Cnt);
	cout << "D2 instances is " << d2Cnt << endl;
	int totInstances = cnt + d2Cnt - 1; // Don't count seed instance
	cout << "Instance total = " << totInstances << endl;
	struct IDispatch *RefAxisDisp;
	hres = thePattern->get_D1Axis(&RefAxisDisp);
	if (hres == S_OK && RefAxisDisp) {
		UINT typeCnt = -1;
		hres = RefAxisDisp->GetTypeInfoCount(&typeCnt);
		cout << "type count = " << typeCnt << " hres " << hres << endl;
		CComPtr<ITypeInfo> spTypeInfo;
		hres = RefAxisDisp->GetTypeInfo(0, 0, &spTypeInfo);
		if (hres == S_OK && spTypeInfo) {
			CComBSTR sITypeName, sDocName, sHelp, sFile;
			DWORD context = -1;
			hres = spTypeInfo->GetDocumentation(-1, &sITypeName, &sDocName, &context, &sFile);
			printbstr("type name ", sITypeName);
			printbstr("doc name ", sDocName);
			printbstr("file name ", sFile);
		}
		else cout << "No typeinfo, res " << hres << " ptr " << spTypeInfo << endl;
		IID unused = IID_NULL;
		CComBSTR refName(L"axis");
		DISPID dispID;
		hres = RefAxisDisp->GetIDsOfNames(unused, &refName, 1, GetUserDefaultLCID(), &dispID);
		cout << "getids hres = " << std::hex << hres << " dispid = " << dispID << endl;
		CComPtr<IRefAxis> refAxis;
		// hres = RefAxisDisp->QueryInterface(IID_IRefAxis, reinterpret_cast<void**>(&refAxis));
		// RefAxisDisp->Release();
		if (false) {
			double *axisArray = new double[7];
			hres = refAxis->IGetRefAxisParams(axisArray);
			cout << "Found ref axis feature data\n";
			if (hres == S_OK) {
				coords startPt(axisArray[0], axisArray[1], axisArray[2]);
				coords endPt(axisArray[3], axisArray[4], axisArray[5]);
				cout << "start and end pts" << startPt << " " << endPt << endl;
			}
		}
		else cout << "Couldn't get refaxis " << std::hex << hres << " ptr " << refAxis << endl;
	}
	thePattern->ReleaseSelectionAccess();
	return (xspacing);
}

template<typename T>
void AssemblyInfo::procPattern(CComPtr<IFeature> swFeature, IModelDoc2* swModel, PatternFuncs<T> *pattfuncs) {
	struct IDispatch *pPatternDisp;
	T *thePattern;
	hres = swFeature->GetDefinition(&pPatternDisp);
	if (hres == S_OK && pPatternDisp) {
		cout << "Found Local Circular or Linear Pattern definition " << endl;
		hres = pPatternDisp->QueryInterface(__uuidof(typename T), reinterpret_cast<void**>(&thePattern));
		pPatternDisp->Release();
		if (hres == S_OK && thePattern) {
			cout << "Found Local Circular or Linear Pattern! " << endl;
			VARIANT_BOOL accessOk = VARIANT_FALSE;
			hres = thePattern->AccessSelections(swModel, NULL, &accessOk);
			if (hres == S_OK && accessOk == VARIANT_TRUE) {
				double valpatt = pattfuncs->getSpacing(thePattern, *this);
			}
			else cout << "Can't access selections\n";
		}
		else cout << "Failed to get local pattern retval = " << hres << " ptr = " << thePattern << endl;
	}
	else cout << "Failed to get local pattern definition retval = " << hres << " ptr = " << pPatternDisp << endl;
}

void AssemblyInfo::chkPatterns(CComPtr<IFeature> swFeature, IModelDoc2* swModel, const CComBSTR &sTypeName) {
	const CComBSTR sLocalPatternTypeName(L"LocalLPattern");
	if (VarBstrCmp(sTypeName, sLocalPatternTypeName, 0, 0) == 1) {
		LinPattFuncs<ILocalLinearPatternFeatureData> pattfuncs;
		procPattern<ILocalLinearPatternFeatureData>(swFeature, swModel, &pattfuncs);
		return;
		/******  
		Above line should do everything that is below. Can delete below when sure all is OK
		Also need if for circular pattern
		*/
		struct IDispatch *pPatternDisp;
		ILocalLinearPatternFeatureData *linpattern;
		hres = swFeature->GetDefinition(&pPatternDisp);
		if (hres == S_OK && pPatternDisp) {
			cout << "Found Local Linear Pattern definition " << endl;
			hres = pPatternDisp->QueryInterface(__uuidof(ILocalLinearPatternFeatureData), reinterpret_cast<void**>(&linpattern));
			pPatternDisp->Release();
			if (hres == S_OK && linpattern) {
				cout << "Found Local Linear Pattern! " << endl;
				VARIANT_BOOL accessOk = VARIANT_FALSE;
				hres = linpattern->AccessSelections(swModel, NULL, &accessOk);
				if (hres == S_OK && accessOk == VARIANT_TRUE) {
					double xspacing = 0;
					hres = linpattern->get_D1Spacing(&xspacing);
					cout << "D1 spacing " << xspacing << endl;
					double zspacing = 0;
					hres = linpattern->get_D2Spacing(&zspacing);
					cout << "D2 spacing " << zspacing << endl;
					VARIANT_BOOL revDir = VARIANT_FALSE;
					hres = linpattern->get_D1ReverseDirection(&revDir);
					cout << "D1 reverse direction " << revDir << endl;
					long axisType = -1;
					hres = linpattern->GetD1AxisType(&axisType);
					cout << "D1 axis type is " << axisType << endl;
					hres = linpattern->get_D2ReverseDirection(&revDir);
					cout << "D2 reverse direction " << revDir << endl;
					hres = linpattern->GetD2AxisType(&axisType);
					cout << "D2 axis type is " << axisType << endl;
					getTransfDisplace(linpattern, xspacing, zspacing);
					long cnt = -1;
					hres = linpattern->GetSeedComponentCount(&cnt);
					cout << "Seed component count is " << cnt << endl;
					if (cnt > 0) {
						getSeedComps(linpattern);
					}
					hres = linpattern->GetSkippedItemCount(&cnt);
					cout << "Skipped item count is " << cnt << endl;
					hres = linpattern->get_D1TotalInstances(&cnt);
					cout << "D1 instances is " << cnt << endl;
					long d2Cnt = -1;
					hres = linpattern->get_D2TotalInstances(&d2Cnt);
					cout << "D2 instances is " << d2Cnt << endl;
					int totInstances = cnt + d2Cnt - 1; // Don't count seed instance
					cout << "Instance total = " << totInstances << endl;
					struct IDispatch *RefAxisDisp;
					hres = linpattern->get_D1Axis(&RefAxisDisp);
					if (hres == S_OK && RefAxisDisp) {
						UINT typeCnt = -1;
						hres = RefAxisDisp->GetTypeInfoCount(&typeCnt);
						cout << "type count = " << typeCnt << " hres " << hres << endl;
						CComPtr<ITypeInfo> spTypeInfo;
						hres = RefAxisDisp->GetTypeInfo(0, 0, &spTypeInfo);
						if (hres == S_OK && spTypeInfo) {
							CComBSTR sITypeName, sDocName, sHelp, sFile;
							DWORD context = -1;
							hres = spTypeInfo->GetDocumentation(-1, &sITypeName, &sDocName, &context, &sFile);
							printbstr("type name ", sITypeName);
							printbstr("doc name ", sDocName);
							printbstr("file name ", sFile);
						}
						else cout << "No typeinfo, res " << hres << " ptr " << spTypeInfo << endl;
						IID unused = IID_NULL;
						CComBSTR refName(L"axis");
						DISPID dispID;
						hres = RefAxisDisp->GetIDsOfNames(unused, &refName, 1, GetUserDefaultLCID(), &dispID);
						cout << "getids hres = " << std::hex << hres << " dispid = " << dispID << endl;
						CComPtr<IRefAxis> refAxis;
						// hres = RefAxisDisp->QueryInterface(IID_IRefAxis, reinterpret_cast<void**>(&refAxis));
						// RefAxisDisp->Release();
						if (false) {
							double *axisArray = new double[7];
							hres = refAxis->IGetRefAxisParams(axisArray);
							cout << "Found ref axis feature data\n";
							if (hres == S_OK) {
								coords startPt(axisArray[0], axisArray[1], axisArray[2]);
								coords endPt(axisArray[3], axisArray[4], axisArray[5]);
								cout << "start and end pts" << startPt << " " << endPt << endl;
							}
						}
						else cout << "Couldn't get refaxis " << std::hex << hres << " ptr " << refAxis << endl;
					}
					linpattern->ReleaseSelectionAccess();
				}
				else cout << "Can't access selections\n";
			}
			else cout << "Failed to get local linear pattern retval = " << hres << " ptr = " << linpattern << endl;
		}
		else cout << "Failed to get local linear pattern definition retval = " << hres << " ptr = " << pPatternDisp << endl;
	}
}


void TraverseFeatureManagerDesignTree(IModelDoc2* swModel, ISldWorks* swApp)
// Traverse FeatureManager design tree to get the
// specified feature in FeatureManager design tree
{
	//Use ATL smart pointers

	CComPtr<IFeature> swFeature;

	CComBSTR sGetFeatureName(L"");

	CComBSTR sFeatureName(L"Outer tube_version3_ajp_june2014-1");
	// CComBSTR sFeatureName2(L"hv_outertube cone_ajp_version4_mdh_mar14-1");
	CComBSTR sFeatureName2(L"LS-side-tank-1_edin-1");
	CComBSTR sFeatureName3(L"hv_outertube cone_ajp_version4_mdh_mar14-1");
	// CComBSTR sFeatureName2(L"tube-fitting_edin-1");
	CComBSTR sFeatureName4(L"tyvek-side-tank_edin-1");
	CComBSTR sFeatureName5(L"tyvek-side-tank_edin-2");
	CComBSTR sFeatureName6(L"reflector-1_edin-1");
	CComBSTR sFeatureName7(L"lifting eye_edin-5");
	CComBSTR sFeatureName8(L"lifting eye_edin-6");
	CComBSTR sFeatureName9(L"lifting eye_edin-7");
	CComBSTR sFeatureName10(L"lifting eye_edin-8");
	CComBSTR sFeatureName11(L"lifting eye_edin-1");
	CComBSTR sFeatureName12(L"LocalLPattern1");
	// fitting-stub_edin-1
	//  LS-side-tank-1_edin-1 LocalLPattern1
	// fitting - flange_edin - 1
	//	teflon - tube_edin - 1
	//  tube-fitting_edin-1

	// testrot();
	bool bFoundComponents = false;

	hres = swModel->IFirstFeature(&swFeature);
	if (hres != S_OK || swFeature == NULL)
		return;

	// If the name of the feature matches

	// then select the feature

	xmlElem beginEnd, attrib1, attrib2, attrib3;
	gdmlout << beginEnd.openSepElem("solids") << endl;
	gdmlout << indent1 << beginEnd.openElem("box") << attrib1.attribute("name", "WorldBox");
	gdmlout << attrib1.attribute("x", "10000.0") << attrib2.attribute("y", "10000.0") << attrib3.attribute("z", "10000.0");
	gdmlout << beginEnd.closeElem() << endl;
	AssemblyInfo assembly;
	do	{
		hres = swFeature->get_Name(&sGetFeatureName);
		CW2A featurenam(sGetFeatureName);
		if (hres == S_OK && sGetFeatureName) {
			cout << "feature name " << featurenam << endl;
		}
		CComBSTR sTypeName;
		hres = swFeature->GetTypeName2(&sTypeName);
		if (hres == S_OK && sTypeName) {
			CW2A typeNameStr(sTypeName);
			cout << "Feature type name is " << typeNameStr << endl;
			assembly.featureTypeStr = typeNameStr;
			CComBSTR sMateGroup("MateGroup");
			// if (VarBstrCmp(sTypeName, sMateGroup, 0, 0) == 1) {
				// showMateEntity(swFeature);
			// }
		}
		else cout << "No type name\n";
		if (true || VarBstrCmp(sGetFeatureName, sFeatureName12, 0, 0) == 1)
			// if (VarBstrCmp(sGetFeatureName, sFeatureName7, 0, 0) != 1 && VarBstrCmp(sGetFeatureName, sFeatureName8, 0, 0) != 1 &&
			// VarBstrCmp(sGetFeatureName, sFeatureName9, 0, 0) != 1 && VarBstrCmp(sGetFeatureName, sFeatureName10, 0, 0) != 1 &&
			// VarBstrCmp(sGetFeatureName, sFeatureName11, 0, 0) != 1)
		// if (VarBstrCmp(sGetFeatureName, sFeatureName, 0, 0) == 1 || VarBstrCmp(sGetFeatureName, sFeatureName3, 0, 0) == 1 ||
			// VarBstrCmp(sGetFeatureName, sFeatureName2, 0, 0) == 1 || VarBstrCmp(sGetFeatureName, sFeatureName6, 0, 0) == 1 ||
			// VarBstrCmp(sGetFeatureName, sFeatureName6, 0, 0) == 1)
		{
			CComPtr<ISelectionMgr> swSelMgr;
			cout << "feature match " << endl;

			hres = swFeature->Select2(true, 1, &retVal);
			cout << "after select2 " << endl;
			if (hres == S_OK && retVal && swModel) {
				cout << "good select2 model  selmgr " << swModel << " " << swSelMgr << endl;
				hres = swModel->get_ISelectionManager(&swSelMgr);
				if (hres == S_OK && swSelMgr) {
					long lNumSelections = 0;
					swSelMgr->GetSelectedObjectCount2(-1, &lNumSelections);
					cout << "num selections = " << lNumSelections << endl;

					struct IDispatch *pComponentDisp;
					CComPtr<ISelectData> swSelData;
					hres = swSelMgr->CreateSelectData(&swSelData);
					if (hres == S_OK && swSelData)
						cout << "got select data \n";
					hres = swSelMgr->GetSelectedObject6(1, -1, &pComponentDisp);
					if (hres == S_OK && pComponentDisp) {
						IComponent2* swSelectedComponent;
						hres = pComponentDisp->QueryInterface(__uuidof(IComponent2), reinterpret_cast<void**>(&swSelectedComponent));
						pComponentDisp->Release();
						if (hres == S_OK && swSelectedComponent) {
							cout << "found component " << endl;
							procComponents(swSelectedComponent, swSelData, assembly);
						}
						else {
							cout << "Failed to get selected component retval = " << std::hex << hres << " ptr = " << swSelectedComponent << endl;
							assembly.chkPatterns(swFeature, swModel, sTypeName);
						}
						long deselOK = 0;
						long indexList[1] = { 1 };
						hres = swSelMgr->IDeSelect2(1, indexList, -1, &deselOK);
						cout << "Result of deselect hres retval " << std::hex << hres << " " << deselOK << endl;
					} else cout << "Failed to get selected objects retval = " << hres << " ptr = " << pComponentDisp << endl;
				}
				hres = swFeature->DeSelect(&retVal); // Said to be dangerous and might fail
				if (hres != S_OK || !retVal)
					cout << "Failed to DeSelect feature\n";
				cout << endl; // Blank line to separate features
			} // end if select feature
		} 
		//Get next feature

		CComPtr<IFeature> swFeatureNext;

		hres = swFeature->IGetNextFeature(&swFeatureNext);

		swFeature.Release();

		swFeature = swFeatureNext;
		// static int cnter = 0;
		// if (++cnter > 20)
			// bFoundComponents = true;
	} while (hres == S_OK && swFeature && !bFoundComponents);
	assembly.outputParts();
}


void CloseDocuments(ISldWorks* swApp)

//Close assembly and drawing documents

{

	swApp->CloseAllDocuments(true, &retVal);

}

