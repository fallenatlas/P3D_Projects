 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>
#include <limits>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"
	
//Enable OpenGL drawing.  
bool drawModeEnabled = true;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 4  //number of bounces
#define ROUGHNESS 0.3 //roughness parameter for fuzzy reflection

#define AREA_LIGHT_LIGHTS 16  //number of lights in the area light source

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;


unsigned int spp = 1;
bool antialiasing = false;
bool dof = false;
bool softLights = false;



/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
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

	/* S�Ese faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	�Efeito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
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
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
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

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
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

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

bool pointInShadow(Vector origin, Vector direction, float distanceToLight) {
	Ray ray = Ray(origin, direction);
	int numObjects = scene->getNumObjects();
	for (int i = 0; i < numObjects; i++) {
		Object* object = scene->getObject(i);
		float distance = 0;
		if (object->intercepts(ray, distance)) {
			if (distanceToLight > distance) {
				return true;
			}
		}
	}
	return false;
}

Color softShadowLight(Light* light, Vector pointOfContact, Ray ray, Material* material, Vector normal) {
	bool inShadow = false;
	Vector lightDirection = light->position - pointOfContact;
	float distanceToLight = lightDirection.length();
	lightDirection = lightDirection.normalize();

	if (Accel_Struct == accelerator::GRID_ACC) {
		inShadow = grid_ptr->Traverse(Ray(pointOfContact, light->position - pointOfContact));
	}
	else if (Accel_Struct == accelerator::BVH_ACC) {
		inShadow = bvh_ptr->Traverse(Ray(pointOfContact, light->position - pointOfContact));
	}
	else {
		inShadow = pointInShadow(pointOfContact, lightDirection, distanceToLight);
	}
			

	if (!inShadow) { //trace shadow ray
		Vector h = (lightDirection - ray.direction).normalize();
		Vector lightColor = Vector(light->color.r(), light->color.g(), light->color.b());
		Vector diffColor = Vector(material->GetDiffColor().r(), material->GetDiffColor().g(), material->GetDiffColor().b());
		Vector specColor = Vector(material->GetSpecColor().r(), material->GetSpecColor().g(), material->GetSpecColor().b());
		Vector diffuse = Vector(lightColor.x * diffColor.x, lightColor.y * diffColor.y, lightColor.z * diffColor.z) * max(normal * lightDirection, 0.0F) * material->GetDiffuse();
		Vector specular = Vector(lightColor.x * specColor.x, lightColor.y * specColor.y, lightColor.z * specColor.z) * powf(max(h * normal, 0.0F), material->GetShine()) * material->GetSpecular();

		Vector blinnPhong = diffuse + specular;
		return Color(blinnPhong.x, blinnPhong.y, blinnPhong.z);
	}
	else {
		//Return Shadow
		return Color(0, 0, 0);
	}
}


Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	Color color;

	float smallestDistance = std::numeric_limits<float>::infinity();
	Object* closestObject = NULL;
	
	Vector hitPoint;

	if (Accel_Struct == GRID_ACC) {
		if (!grid_ptr->Traverse(ray, &closestObject, hitPoint)) {
			if (scene->GetSkyBoxFlg()) {
				return scene->GetSkyboxColor(ray).clamp();
			}
			else {
				return scene->GetBackgroundColor().clamp();
			}
		}
	}
	else if (Accel_Struct == BVH_ACC) {
		if (!bvh_ptr->Traverse(ray, &closestObject, hitPoint)) {
			if (scene->GetSkyBoxFlg()) {
				return scene->GetSkyboxColor(ray).clamp();
			}
			else {
				return scene->GetBackgroundColor().clamp();
			}
		}
	}
	else {
		int numObjects = scene->getNumObjects();
		for (int i = 0; i < numObjects; i++) {
			Object* object = scene->getObject(i);
			float distance = 0;
			if (object->intercepts(ray, distance)) {
				if (distance < smallestDistance) {
					closestObject = object;
					smallestDistance = distance;
				}
			}
		}
	}

	if (closestObject == NULL) {
		return scene->GetBackgroundColor();
	}

	hitPoint = Accel_Struct == accelerator::NONE ? ray.origin + ray.direction * smallestDistance : hitPoint;  //Accel_Struct != accelerator::NONE ? hitPoint :
	Vector normal = closestObject->getNormal(hitPoint);  // still missing  // error here?
	bool inside = (ray.direction * normal) > 0;
	float bias = 0.001F;
	Vector pointOfContact = hitPoint + normal * EPSILON;
	Material* material = closestObject->GetMaterial();

	if (!inside) {
		int numLights = scene->getNumLights();
		for (int i = 0; i < numLights; i++) {
			Light* light = scene->getLight(i);

			if (softLights) {
				if (antialiasing) {
							// for dof
							/*
							Vector pixel = Vector(light->position.x - 1.0f, light->position.y - 1.0f, light->position.z);
							pixel = pixel + Vector(2.0f, 0.0f, 0.0f) * rand_float();
							pixel = pixel + Vector(0.0f, 2.0f, 0.0f) * rand_float();
							*/
							Vector pixel = Vector(light->position.x - 3.0f, light->position.y, light->position.z - 3.0f);
							pixel = pixel + Vector(6.0f, 0.0f, 0.0f) * rand_float();
							pixel = pixel + Vector(0.0f, 0.0f, 6.0f) * rand_float();
							
							Light* NLight = new Light(pixel, light->color);
							color += softShadowLight(NLight, pointOfContact, ray, material, normal);
				}
				else {
					float spacing = 1.0f / sqrtf(AREA_LIGHT_LIGHTS);
					Color brightness = light->color * (1.0f / AREA_LIGHT_LIGHTS);
					float initial_offset = -(1.0f / 2.0f) + (spacing / 2);

					for (int j = 0; j < sqrtf(AREA_LIGHT_LIGHTS); j++) {
						for (int k = 0; k < sqrtf(AREA_LIGHT_LIGHTS); k++) {
							//Original point light will be in a corner of the area light source
							Light* Nlight = new Light(light->position + Vector(initial_offset + (j * spacing), 0.0, initial_offset + (k * spacing)), brightness);
							color += softShadowLight(Nlight, pointOfContact, ray, material, normal);
						}
					}

				}
			}
			else {
				color += softShadowLight(light, pointOfContact, ray, material, normal);
			}		
		}
	}

	if (depth > MAX_DEPTH) {
		return color.clamp();
	}

	normal = inside ? normal * -1 : normal;
	Vector V = ray.direction * -1;

	Color rColor;
	Color tColor;

	float reflection = closestObject->GetMaterial()->GetReflection();
	bool reflective = reflection > 0.0F;
	if (reflective) {

		Vector reflectionDirection = (normal * (2 * (normal * V)) - V).normalize();
		Vector fuzzyReflectionDirection = (reflectionDirection + ((rnd_unit_sphere() * ROUGHNESS))).normalize();

		Ray rRay = Ray(pointOfContact, (fuzzyReflectionDirection * normal) > 0.0F ? fuzzyReflectionDirection : reflectionDirection);
		rColor = rayTracing(rRay, depth + 1, ior_1); // * reflection
	}

	float refraction = closestObject->GetMaterial()->GetRefrIndex();
	float transmittance = closestObject->GetMaterial()->GetTransmittance();
	bool transparent = transmittance == 1.0F;

	float Kr = reflection;

	if (transparent) {
		float nextIor = !inside ? refraction : 1.0F;
		float n = ior_1 / nextIor;

		Vector Vt = (normal * (normal * V)) - V;
		float sinI = Vt.length();
		float sinT = n * sinI;
		if (1 - (sinT * sinT) >= 0) {
			float cosT = sqrtf(1 - (sinT * sinT));
			Vector t = Vt.normalize();

			float r0 = powf((ior_1 - nextIor) / (ior_1 + nextIor), 2);
			float cosI = inside ? cosT : sqrtf(1 - (sinI * sinI));
			Kr = r0 + ((1 - r0) * powf(1 - cosI, 5));

			Vector refractionDirection = (t * sinT + normal * -cosT).normalize();
			Vector pointOfTransmitance = hitPoint + normal * -EPSILON;
			Ray rRay = Ray(pointOfTransmitance, refractionDirection);

			tColor = rayTracing(rRay, depth + 1, nextIor);
		}
	}

	color += rColor * Kr + tColor * (1 - Kr);

	return color.clamp();
}



// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	set_rand_seed(time(NULL) * time(NULL));

	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;

			Vector pixel;  //viewport coordinates
			
			if (antialiasing) {
				for (int p = 0; p < spp; p++) {
					for (int q = 0; q < spp; q++) {
						pixel.x = x + (p + rand_float()) / spp;
						pixel.y = y + (q + rand_float()) / spp;
						Ray ray = Ray(Vector(0.0F, 0.0F, 0.0F), Vector(0.0F, 0.0F, 0.0F));
						if (dof) {
							Vector lens_sample = rnd_unit_disk();
							ray = scene->GetCamera()->PrimaryRay(lens_sample, pixel);
						}
						else {
							ray = scene->GetCamera()->PrimaryRay(pixel);   //function from camera.h
						}
						color += rayTracing(ray, 1, 1.0).clamp();
					}
				}

				color = color * (1 / pow(spp, 2));
			}
			else {
				
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;

				//YOUR 2 FUNTIONS:
				Ray ray = scene->GetCamera()->PrimaryRay(pixel);   //function from camera.h

				color = rayTracing(ray, 1, 1.0).clamp();
			
			}
			
			
			//color = scene->GetBackgroundColor(); //TO CHANGE - just for the template

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}

	}
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

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

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	spp = scene->GetSamplesPerPixel();
	if (spp == 0) {
		antialiasing = false;
		//spp = 1;
		printf("Whitted Ray-Tracing\n");
	}
	else {
		antialiasing = true;
		printf("Distribution Ray-Tracing\n");
	}

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

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////