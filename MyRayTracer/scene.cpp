#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include "maths.h"
#include "scene.h"


Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */
	normal = (P2 - P1) % (P0 - P1);
	normal.normalize();

	//Calculate the Min and Max for bounding box
	Min = Vector(+FLT_MAX, +FLT_MAX, +FLT_MAX);
	Max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (int i = 0; i < 3; i++) {
		if (points[i].x < Min.x) Min.x = points[i].x;
		if (points[i].x > Max.x) Max.x = points[i].x;
		if (points[i].y < Min.y) Min.y = points[i].y;
		if (points[i].y > Max.y) Max.y = points[i].y;
		if (points[i].z < Min.z) Min.z = points[i].z;
		if (points[i].z > Max.z) Max.z = points[i].z;
	}


	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t) {
	double a = this->points[0].x - this->points[1].x, b = this->points[0].x - this->points[2].x, c = r.direction.x, d = this->points[0].x - r.origin.x;
	double e = this->points[0].y - this->points[1].y, f = this->points[0].y - this->points[2].y, g = r.direction.y, h = this->points[0].y - r.origin.y;
	double i = this->points[0].z - this->points[1].z, j = this->points[0].z - this->points[2].z, k = r.direction.z, l = this->points[0].z - r.origin.z;

	double m = f * k - g * j, n = h * k - g * l, p = f * l - h * j;
	double q = g * i - e * k, s = e * j - f * i;

	double inv_denom = 1.0 / (a * m + b * q + c * s);

	double e1 = d * m - b * n - c * p;
	double beta = e1 * inv_denom;

	if (beta < 0.0)
		return (false);

	double ray = ray = e * l - h * i;
	double e2 = a * n + d * q + c * ray;
	double gamma = e2 * inv_denom;

	if (gamma < 0.0)
		return (false);

	if (beta + gamma > 1.0)
		return (false);

	double e3 = a * p - b * ray + d * s;
	t = e3 * inv_denom;

	if (t < 0.001f)
		return (false);

	if (t > 0)
		return t;
	return (false);
}

Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
	float l;

	//Calculate the normal plane: counter-clockwise vectorial product.


	PN = (P2 - P1) % (P0 - P1);
	PN.normalize();

	if ((l = PN.length()) == 0.0)
	{
		cerr << "DEGENERATED PLANE!\n";
	}
	else
	{
		PN.normalize();
		//Calculate D
		D = -(P0 * PN);
	}
}

//
// Ray/Plane intersection test.
//

bool Plane::intercepts(Ray& r, float& t)
{
	float aux = PN * r.direction;
	if (abs(aux) < 0.0001f) {
		return false;
	}
	t = -(r.origin * PN + D) / aux;
	if (t > 0)
		return t;
	return false;
}

Vector Plane::getNormal(Vector point)
{
	return PN;
}

//
// Ray/Sphere intersection test.
//
bool Sphere::intercepts(Ray& r, float& t)
{
	float dOC = pow(this->center.x - r.origin.x, 2) + pow(this->center.y - r.origin.y, 2) + pow(this->center.z - r.origin.z, 2);
	float b = r.direction.x * (this->center.x - r.origin.x) + r.direction.y * (this->center.y - r.origin.y) +
		r.direction.z * (this->center.z - r.origin.z);

	if (dOC > this->SqRadius&& b < 0)
		return false;
	float ra = pow(b, 2) - dOC + this->SqRadius;

	if (ra < 0)
		return false;
	if (dOC > this->SqRadius) {
		t = b - sqrt(ra);
	}
	else {
		t = b + sqrt(ra);
	}
	if (t > 0)
		return t;
	else
		return false;
}


Vector Sphere::getNormal(Vector point)
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox() {
	Vector a_min;
	a_min.x = this->center.x - this->radius;
	a_min.y = this->center.y - this->radius;
	a_min.z = this->center.z - this->radius;
	Vector a_max;
	a_max.x = this->center.x + this->radius;
	a_max.y = this->center.y + this->radius;
	a_max.z = this->center.z + this->radius;
	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

//
// Ray/aaBox intersection test using Kay and Kajiya algorithm.
//
bool aaBox::intercepts(Ray& ray, float& t)
{

	float tDist = INFINITY;
	float tProx = -INFINITY;

	float tx_max, ty_max, tz_max, tx_min, ty_min, tz_min;


	float a = 1.0f / ray.direction.x;
	if (a >= 0) {
		tx_min = (this->GetBoundingBox().min.x - ray.origin.x) * a;
		tx_max = (this->GetBoundingBox().max.x - ray.origin.x) * a;
	}
	else {
		tx_max = (this->GetBoundingBox().min.x - ray.origin.x) * a;
		tx_min = (this->GetBoundingBox().max.x - ray.origin.x) * a;
	}
	if (tx_min > tProx) {
		tProx = tx_min;
		if (a >= 0)
			this->Normal = Vector(-1, 0, 0);
		else
			this->Normal = Vector(1, 0, 0);
	}
	if (tx_max < tDist)
		tDist = tx_max;
	if (tProx > tDist)
		return false;
	if (tDist < 0)
		return false;

	float b = 1.0f / ray.direction.y;
	if (b >= 0) {
		ty_min = (this->GetBoundingBox().min.y - ray.origin.y) * b;
		ty_max = (this->GetBoundingBox().max.y - ray.origin.y) * b;
	}
	else {
		ty_max = (this->GetBoundingBox().min.y - ray.origin.y) * b;
		ty_min = (this->GetBoundingBox().max.y - ray.origin.y) * b;
	}
	if (ty_min > tProx) {
		tProx = ty_min;
		if (b >= 0)
			this->Normal = Vector(0, -1, 0);
		else
			this->Normal = Vector(0, 1, 0);
	}
	if (ty_max < tDist)
		tDist = ty_max;
	if (tProx > tDist)
		return false;
	if (tDist < 0)
		return false;

	float c = 1.0f / ray.direction.z;
	if (c >= 0) {
		tz_min = (this->GetBoundingBox().min.z - ray.origin.z) * c;
		tz_max = (this->GetBoundingBox().max.z - ray.origin.z) * c;
	}
	else {
		tz_max = (this->GetBoundingBox().min.z - ray.origin.z) * c;
		tz_min = (this->GetBoundingBox().max.z - ray.origin.z) * c;
	}
	if (tz_min > tProx) {
		tProx = tz_min;
		if (c >= 0)
			this->Normal = Vector(0, 0, -1);
		else
			this->Normal = Vector(0, 0, 1);
	}
	if (tz_max < tDist)
		tDist = tz_max;
	if (tProx > tDist)
		return false;
	if (tDist < 0)
		return false;
	if (tProx > 0) {
		t = tProx;
		return true;
	}
	return false;

}



Vector aaBox::getNormal(Vector point)
{
	return this->Normal;
}

Scene::Scene()
{}

Scene::~Scene()
{
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}

vector<Object*> Scene::getObejcts()
{
	return objects;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadFirebox(const char* sky_dir)
{
	char* filenames[6];
	char buffer[100]; 
	const char* maps[] = { "/fleft.jpg", "/fleft.jpg", "/fleft.jpg", "/fleft.jpg", "/fleft.jpg", "/fleft.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char*)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}

	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte*)malloc(size);
		ILubyte* bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

void Scene::LoadSkybox(const char* sky_dir)
{
	char* filenames[6];
	char buffer[100];
	const char* maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char*)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}

	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte*)malloc(size);
		ILubyte* bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char* token, const char* name)
{
	file >> token;
	if (strcmp(token, name))
		cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char* name)
{
	const	int	lineSize = 1024;
	string	cmd;
	char		token[256];
	ifstream	file(name, ios::in);
	Material* material;

	material = NULL;

	if (file >> cmd)
	{
		while (true)
		{

			if (cmd == "f")   //Material
			{
				double Kd, Ks, Shine, T, ior;
				Color cd, cs;

				file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

				material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
			}

			else if (cmd == "s")    //Sphere
			{
				Vector center;
				float radius;
				Sphere* sphere;

				file >> center >> radius;
				sphere = new Sphere(center, radius);
				if (material) sphere->SetMaterial(material);
				this->addObject((Object*)sphere);
			}

			else if (cmd == "box")    //axis aligned box
			{
				Vector minpoint, maxpoint;
				aaBox* box;

				file >> minpoint >> maxpoint;
				box = new aaBox(minpoint, maxpoint);
				if (material) box->SetMaterial(material);
				this->addObject((Object*)box);
			}
			else if (cmd == "p")  // Polygon: just accepts triangles for now
			{
				Vector P0, P1, P2;
				Triangle* triangle;
				unsigned total_vertices;

				file >> total_vertices;
				if (total_vertices == 3)
				{
					file >> P0 >> P1 >> P2;
					triangle = new Triangle(P0, P1, P2);
					if (material) triangle->SetMaterial(material);
					this->addObject((Object*)triangle);
				}
				else
				{
					cerr << "Unsupported number of vertices.\n";
					break;
				}
			}

			else if (cmd == "pl")  // General Plane
			{
				Vector P0, P1, P2;
				Plane* plane;

				file >> P0 >> P1 >> P2;
				plane = new Plane(P0, P1, P2);
				if (material) plane->SetMaterial(material);
				this->addObject((Object*)plane);
			}

			else if (cmd == "l")  // Need to check light color since by default is white
			{
				Vector pos;
				Color color;

				file >> pos >> color;

				this->addLight(new Light(pos, color));

			}
			else if (cmd == "v")
			{
				Vector up, from, at;
				float fov, hither;
				int xres, yres;
				Camera* camera;
				float focal_ratio; //ratio beteween the focal distance and the viewplane distance
				float aperture_ratio; // number of times to be multiplied by the size of a pixel

				next_token(file, token, "from");
				file >> from;

				next_token(file, token, "at");
				file >> at;

				next_token(file, token, "up");
				file >> up;

				next_token(file, token, "angle");
				file >> fov;

				next_token(file, token, "hither");
				file >> hither;

				next_token(file, token, "resolution");
				file >> xres >> yres;

				next_token(file, token, "aperture");
				file >> aperture_ratio;

				next_token(file, token, "focal");
				file >> focal_ratio;
				// Create Camera
				camera = new Camera(from, at, up, fov, hither, 100.0 * hither, xres, yres, aperture_ratio, focal_ratio);
				this->SetCamera(camera);
			}

			else if (cmd == "bclr")   //Background color
			{
				Color bgcolor;
				file >> bgcolor;
				this->SetBackgroundColor(bgcolor);
			}

			else if (cmd == "env")
			{
				file >> token;

				this->LoadSkybox(token);
				this->SetSkyBoxFlg(true);
			} 
			else if (cmd == "fire")
			{
			file >> token;

			this->LoadFirebox(token);
			this->SetSkyBoxFlg(true);
			}
			else if (cmd[0] == '#')
			{
				file.ignore(lineSize, '\n');
			}
			else
			{
				cerr << "unknown command '" << cmd << "'.\n";
				break;
			}
			if (!(file >> cmd))
				break;
		}
	}

	file.close();
	return true;
};
