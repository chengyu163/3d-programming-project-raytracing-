///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2019 by João Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "grid.h"
#include "maths.h"
#include "sampler.h"


#define CAPTION "Whitted Ray-Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define MAX_DEPTH 4
#define POINTOFAREA 3
#define N 8


bool antialiasing = false;
bool depthOfField = false;
bool withGrid = true;
bool softShadows = false;

//Enable OpenGL drawing.  
bool drawModeEnabled = false;

//Draw Mode: 0 - point by point; 1 - line by line; 2 - full frame at once
int draw_mode = 1;

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float* colors;
float* vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t* img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;
Grid* grid = NULL;
int RES_X, RES_Y;

int WindowHandle = 0;


bool shadowRayTracing(Ray ray) {
	float t;
	if (!withGrid) {
		for (int i = 0; i < scene->getNumObjects(); i++) {
			if (scene->getObject(i)->intercepts(ray, t)) {
				return true;
			}
		}
		return false;
	}
	else {
		Vector intersectionPoint;
		Vector normalIntersection;
		return grid->Traverse(ray);
	}
}

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	bool intersection = false;
	float auxDistance;
	float minDistance = INFINITY;
	int closestIntercept = -1;
	Vector intersectionPoint;
	Vector normalVectorOfPoint;
	if (!withGrid) {
		Material* MaterialIntersect;
		for (int i = 0; i < scene->getNumObjects(); i++) {
			if (scene->getObject(i)->intercepts(ray, auxDistance)) {
				if (auxDistance < minDistance) {
					closestIntercept = i;
					minDistance = auxDistance;
				}
			}
		}
		if (closestIntercept == -1) {
			if (scene->GetSkyBoxFlg())
				return scene->GetSkyboxColor(ray);
			return scene->GetBackgroundColor();
		}
		MaterialIntersect = scene->getObject(closestIntercept)->GetMaterial();

		// Ray tracing starts here 
		Color color;

		intersectionPoint = ray.origin + ray.direction * minDistance;
		normalVectorOfPoint = (scene->getObject(closestIntercept)->getNormal(intersectionPoint)).normalize();
		bool inside = (ray.direction * normalVectorOfPoint) > 0;
		Vector bias = normalVectorOfPoint * 0.001f;

		for (int i = 0; i < scene->getNumLights(); i++) {
			Light* light = scene->getLight(i);
			if (antialiasing) {
				Vector direction;
				if (!softShadows) {
					direction = (light->position - intersectionPoint).normalize();
				}
				else {
					direction = (light->position + Vector((rand_float() / 2), (rand_float() / 2), (rand_float() / 2)) - intersectionPoint).normalize();
				}
				float diffuse = direction * normalVectorOfPoint;
				if (diffuse > 0) {
					Ray shadowRay = Ray(intersectionPoint + bias, direction);
					if (!shadowRayTracing(shadowRay)) {
						color = color + scene->getLight(i)->color * MaterialIntersect->GetDiffuse() * MaterialIntersect->GetDiffColor() * diffuse;
						if (MaterialIntersect->GetSpecular() > 0) {
							Vector half = (direction - ray.direction).normalize();
							color = color + scene->getLight(i)->color * MaterialIntersect->GetSpecColor() * MaterialIntersect->GetSpecular() * pow(max(0, half * normalVectorOfPoint), MaterialIntersect->GetShine());
						}
					}
				}
			}
			else {
				if (softShadows) {
					for (int k = 0; k < POINTOFAREA; k++) {
						Vector direction = (light->position + Vector(k / 10.0, k / 10.0, k / 10.0) - intersectionPoint).normalize();
						float diffuse = direction * normalVectorOfPoint;
						if (diffuse > 0) {
							Ray shadowRay = Ray(intersectionPoint + bias, direction);
							if (!shadowRayTracing(shadowRay)) {
								float intensity = 1.0 / POINTOFAREA;
								color = color + scene->getLight(i)->color * intensity * MaterialIntersect->GetDiffuse() * MaterialIntersect->GetDiffColor() * diffuse;
								if (MaterialIntersect->GetSpecular() > 0) {
									Vector half = (direction - ray.direction).normalize();
									color = color + scene->getLight(i)->color * intensity * MaterialIntersect->GetSpecColor() * MaterialIntersect->GetSpecular() * pow(max(0, half * normalVectorOfPoint), MaterialIntersect->GetShine());
								}

							}
						}
					}
				}
				else {
					Vector direction = (light->position - intersectionPoint).normalize();
					float diffuse = direction * normalVectorOfPoint;
					if (diffuse > 0) {
						Ray shadowRay = Ray(intersectionPoint + bias, direction);
						if (!shadowRayTracing(shadowRay)) {
							color = color + scene->getLight(i)->color * MaterialIntersect->GetDiffuse() * MaterialIntersect->GetDiffColor() * diffuse;
							if (MaterialIntersect->GetSpecular() > 0) {
								Vector half = (direction - ray.direction).normalize();
								color = color + scene->getLight(i)->color * MaterialIntersect->GetSpecColor() * MaterialIntersect->GetSpecular() * pow(max(0, half * normalVectorOfPoint), MaterialIntersect->GetShine());
							}
						}
					}
				}
			}
		}

		if (depth >= MAX_DEPTH) return color.clamp();



		if (MaterialIntersect->GetReflection() > 0) {
			Vector reflectRayDirection = ray.direction - normalVectorOfPoint * (normalVectorOfPoint * ray.direction) * 2;
			reflectRayDirection = reflectRayDirection.normalize();
			Ray* reflectRay = new Ray(intersectionPoint + bias, reflectRayDirection);
			Color reflectColor = rayTracing(*reflectRay, depth + 1, ior_1) * MaterialIntersect->GetReflection() * MaterialIntersect->GetSpecColor();
			color = color + reflectColor;
		}


		if (MaterialIntersect->GetTransmittance()) {
			float eta;

			float ior_2 = MaterialIntersect->GetRefrIndex();

			if (inside) {
				normalVectorOfPoint = normalVectorOfPoint * (-1);
				ior_2 = 1.0;
			}

			eta = ior_1 / ior_2;

			Vector t, r;
			float sinT, cosT;
			Vector negDiretcion = ray.direction * (-1);
			t = normalVectorOfPoint * (negDiretcion * normalVectorOfPoint) - negDiretcion;
			sinT = t.length() * eta;
			float total_reflection = sinT * sinT;

			if (total_reflection <= 1.0f) {
				cosT = sqrt(max(0.0f, 1.0f - (double)total_reflection));
				t = t / t.length(); ;
				r = t * sinT + (normalVectorOfPoint * (-1)) * cosT;
				Vector pi = inside ? intersectionPoint + bias : intersectionPoint - bias;
				Ray refratRay = Ray(pi, r);
				Color refratColor = rayTracing(refratRay, depth + 1, ior_2);
				color = color + refratColor * MaterialIntersect->GetTransmittance();
			}
		}

		return color.clamp();
	} //end of !withgrid
	else {  //start of withgrid
		Material* MaterialIntersect;
		Object* obj = grid->rayIntersection(ray, &intersectionPoint, &normalVectorOfPoint);
		if (obj) {
			MaterialIntersect = obj->GetMaterial();

			// Ray tracing starts here 
			Color color;

			bool inside = (ray.direction * normalVectorOfPoint) > 0;
			Vector bias = normalVectorOfPoint * 0.001f;

			for (int i = 0; i < scene->getNumLights(); i++) {
				Light* light = scene->getLight(i);
				if (antialiasing) {
					Vector direction;
					if (!softShadows) {
						direction = (light->position - intersectionPoint).normalize();
					}
					else {
						direction = (light->position + Vector((rand_float() / 2), (rand_float() / 2), (rand_float() / 2)) - intersectionPoint).normalize();
					}
					float diffuse = direction * normalVectorOfPoint;
					if (diffuse > 0) {
						Ray shadowRay = Ray(intersectionPoint + bias, direction);
						if (!shadowRayTracing(shadowRay)) {
							color = color + scene->getLight(i)->color * MaterialIntersect->GetDiffuse() * MaterialIntersect->GetDiffColor() * diffuse;
							if (MaterialIntersect->GetSpecular() > 0) {
								Vector half = (direction - ray.direction).normalize();
								color = color + scene->getLight(i)->color * MaterialIntersect->GetSpecColor() * MaterialIntersect->GetSpecular() * pow(max(0, half * normalVectorOfPoint), MaterialIntersect->GetShine());
							}
						}
					}
				}
				else {
					if (softShadows) {
						for (int k = 0; k < POINTOFAREA; k++) {
							Vector direction = (light->position + Vector(k / 10.0, k / 10.0, k / 10.0) - intersectionPoint).normalize();
							float diffuse = direction * normalVectorOfPoint;
							if (diffuse > 0) {
								Ray shadowRay = Ray(intersectionPoint + bias, direction);
								if (!shadowRayTracing(shadowRay)) {
									float intensity = 1.0 / POINTOFAREA;
									color = color + scene->getLight(i)->color * intensity * MaterialIntersect->GetDiffuse() * MaterialIntersect->GetDiffColor() * diffuse;
									if (MaterialIntersect->GetSpecular() > 0) {
										Vector half = (direction - ray.direction).normalize();
										color = color + scene->getLight(i)->color * intensity * MaterialIntersect->GetSpecColor() * MaterialIntersect->GetSpecular() * pow(max(0, half * normalVectorOfPoint), MaterialIntersect->GetShine());
									}

								}
							}
						}
					}
					else {
						Vector direction = (light->position - intersectionPoint).normalize();
						float diffuse = direction * normalVectorOfPoint;
						if (diffuse > 0) {
							Ray shadowRay = Ray(intersectionPoint + bias, direction);
							if (!shadowRayTracing(shadowRay)) {
								color = color + scene->getLight(i)->color * MaterialIntersect->GetDiffuse() * MaterialIntersect->GetDiffColor() * diffuse;
								if (MaterialIntersect->GetSpecular() > 0) {
									Vector half = (direction - ray.direction).normalize();
									color = color + scene->getLight(i)->color * MaterialIntersect->GetSpecColor() * MaterialIntersect->GetSpecular() * pow(max(0, half * normalVectorOfPoint), MaterialIntersect->GetShine());
								}
							}
						}
					}
				}
			}


			if (depth >= MAX_DEPTH) return color.clamp();



			if (MaterialIntersect->GetReflection() > 0) {
				Vector reflectRayDirection = ray.direction - normalVectorOfPoint * (normalVectorOfPoint * ray.direction) * 2;
				reflectRayDirection = reflectRayDirection.normalize();
				Ray* reflectRay = new Ray(intersectionPoint + bias, reflectRayDirection);
				Color reflectColor = rayTracing(*reflectRay, depth + 1, ior_1) * MaterialIntersect->GetReflection() * MaterialIntersect->GetSpecColor();
				color = color + reflectColor;
			}


			if (MaterialIntersect->GetTransmittance()) {
				float eta;

				float ior_2 = MaterialIntersect->GetRefrIndex();

				if (inside) {
					normalVectorOfPoint = normalVectorOfPoint * (-1);
					ior_2 = 1.0;
				}

				eta = ior_1 / ior_2;

				Vector t, r;
				float sinT, cosT;
				Vector negDiretcion = ray.direction * (-1);
				t = normalVectorOfPoint * (negDiretcion * normalVectorOfPoint) - negDiretcion;
				sinT = t.length() * eta;
				float total_reflection = sinT * sinT;

				if (total_reflection <= 1.0f) {
					cosT = sqrt(max(0.0f, 1.0f - (double)total_reflection));
					t = t / t.length(); ;
					r = t * sinT + (normalVectorOfPoint * (-1)) * cosT;
					Vector pi = inside ? intersectionPoint + bias : intersectionPoint - bias;
					Ray refratRay = Ray(pi, r);
					Color refratColor = rayTracing(refratRay, depth + 1, ior_2);
					color = color + refratColor * MaterialIntersect->GetTransmittance();
				}
			}

			return color.clamp();
		}
		else {
			if (scene->GetSkyBoxFlg())
				return scene->GetSkyboxColor(ray);
			return scene->GetBackgroundColor();
		}
	}	
}

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte* errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if (isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");

	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs


void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* Só se faz a alocação dos arrays glBufferData (NULL), e o envio dos pontos para a placa gráfica
	é feito na drawPoints com GlBufferSubData em tempo de execução pois os arrays são GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);

	// unbind the VAO
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
	//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);

	if (draw_mode == 0) glDrawArrays(GL_POINTS, 0, 1);
	else if (draw_mode == 1) glDrawArrays(GL_POINTS, 0, RES_X);
	else glDrawArrays(GL_POINTS, 0, RES_X * RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char* filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (withGrid) {
		grid = new Grid(scene->getObejcts());
	}

	set_rand_seed(time(NULL) * time(NULL));
	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;
			Vector pixel;  //viewport coordinates
			pixel.x = x + 0.5f;
			pixel.y = y + 0.5f;


			if (antialiasing) {
				for (int p = 0; p < N; p++) {
					for (int q = 0; q < N; q++) {
						float random = rand_float(); // random number between 0 and 1 
						Vector pixelPoint;  //viewport coordinates
						pixelPoint.x = pixel.x + (p + random) / N;
						pixelPoint.y = pixel.y + (q + random) / N;
						if (!depthOfField) {
							Ray ray = scene->GetCamera()->PrimaryRay(pixelPoint);
							color += rayTracing(ray, 1, 1.0);
						}
						else {
							float randomL = rand_float();
							Vector pixelL = Vector(randomL * scene->GetCamera()->GetAperture(), randomL * scene->GetCamera()->GetAperture(),0.0f );
							Ray ray = scene->GetCamera()->PrimaryRay(pixelL, pixelPoint);
							color += rayTracing(ray, 1, 1.0);
						}
					}
				}
				color.r(color.r() / pow(N, 2));
				color.g(color.g() / pow(N, 2));
				color.b(color.b() / pow(N, 2));
			}
			else {
				Ray ray = scene->GetCamera()->PrimaryRay(pixel);
				color = rayTracing(ray, 1, 1.0);
			}


			//color = scene->GetBackgroundColor(); //just for the template

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();


				if (draw_mode == 0) {  // drawing point by point
					drawPoints();
					index_pos = 0;
					index_col = 0;
				}
			}
		}
		if (draw_mode == 1 && drawModeEnabled) {  // drawing line by line
			drawPoints();
			index_pos = 0;
			index_col = 0;
		}
	}
	if (draw_mode == 2 && drawModeEnabled)        //full frame at once
		drawPoints();

	printf("Drawing finished!\n");

	if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
		printf("Error saving Image file\n");
		exit(0);
	}
	printf("Image file created\n");
	glFlush();
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top,
	float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void printValues() {
	if (antialiasing) {
		cout << "Antialising is ON\n";
		if (depthOfField) {
			cout << "Depth of Field is ON\n";
		}
		else {
			cout << "Depth of Field is OFF\n";
		}
	}
	else {
		cout << "Antialising is OFF\n";
		cout << "Depth of Field is OFF\n";
	}
	if (withGrid) {
		cout << "Grid is ON\n";
	}
	else {
		cout << "Grid is OFF\n";
	}
	if (softShadows) {
		cout << "Softshadows is ON\n";
	}
	else {
		cout << "Softshadows is OFF\n";
	}
}


void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {
	case 97: //a -switch antialiasing on/off
		antialiasing = !antialiasing;
		break;

	case 115: //s -switch depth of field on/off
		softShadows = !softShadows;
		break;

	case 100: //d -switch grid on/off
		depthOfField = !depthOfField;
		if (depthOfField == true) {
			antialiasing = true;
		}
		break;

	case 103: //g -switch softshadows on/off
		withGrid = !withGrid;
		break;

	case 27:
		glutLeaveMainLoop();
		break;

	}
	printValues();
	renderScene();
}


/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
}

void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit();
	if (result != GLEW_OK) {
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	}
	GLenum err_code = glGetError();
	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
	printf("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	glutInitWindowPosition(640, 100);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if (WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


void init(int argc, char* argv[])
{
	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();

}



void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	while (true) {
		cout << "Input the Scene Name: ";
		cin >> input_user;
		strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
		strcat_s(scene_name, sizeof(scene_name), input_user);

		ifstream file(scene_name, ios::in);
		if (file.fail()) {
			printf("\nError opening P3F file.\n");
		}
		else
			break;
	}

	scene = new Scene();
	scene->load_p3f(scene_name);
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X * RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);
}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	int ch2;
	if (!drawModeEnabled) {
	
		printValues();
		do {
			init_scene();
			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
			if (toupper(ch) == 'Y') {
				do {
					printValues();
					cout << "\nPress A if you want to switch on/off antialising!\n";
					cout << "Press S if you want to switch on/off soft shadows!\n";
					cout << "Press D if you want to switch on/off depth of field!\n";
					cout << "Press G if you want to switch on/off grid!\n";
					cout << "\nPress Q if you do not whish to change any of those!\n";
					ch2 = _getch();
					if (toupper(ch2) == 'A') {
						antialiasing = !antialiasing;
					}
					else if (toupper(ch2) == 'S') {
						softShadows = !softShadows;
					}
					else if (toupper(ch2) == 'D') {
						depthOfField = !depthOfField;
						if (depthOfField == true) {
							antialiasing = true;
						}
					}
					else if (toupper(ch2) == 'G') {
						withGrid = !withGrid;
					}
				} while ((toupper(ch2) != 'Q'));
			}
		} while ((toupper(ch) == 'Y'));
	}

	else {   //Use OpenGL to draw image in the screen
		init_scene();
		if (draw_mode == 0) { // draw image point by point
			size_vertices = 2 * sizeof(float);
			size_colors = 3 * sizeof(float);
			printf("DRAWING MODE: POINT BY POINT\n\n");
		}
		else if (draw_mode == 1) { // draw image line by line
			size_vertices = 2 * RES_X * sizeof(float);
			size_colors = 3 * RES_X * sizeof(float);
			printf("DRAWING MODE: LINE BY LINE\n\n");
		}
		else if (draw_mode == 2) { // draw full frame at once
			size_vertices = 2 * RES_X * RES_Y * sizeof(float);
			size_colors = 3 * RES_X * RES_Y * sizeof(float);
			printf("DRAWING MODE: FULL IMAGE\n\n");
		}
		else {
			printf("Draw mode not valid \n");
			exit(0);
		}
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);

		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);

		/* Setup GLUT and GLEW */
		printValues();
		cout << "\nPress A if you want to switch on/off antialising!\n";
		cout << "Press S if you want to switch on/off soft shadows!\n";
		cout << "Press D if you want to switch on/off depth of field!\n";
		cout << "Press G if you want to switch on/off grid!\n";
		cout << "\nPress Q if you do not whish to change any of those!\n";
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////
