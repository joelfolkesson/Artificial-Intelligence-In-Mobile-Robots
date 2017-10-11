//=============================================================================
//map.h
//=============================================================================
#ifndef maps_h_DEFINED
#define maps_h_DEFINED
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//=============================================================================


//=============================================================================
//Defines
//=============================================================================
//Tracing
#define TraceErrors 1

//Default map parameters
#define DEFAULT_MAX_VALUE   255
#define DEFAULT_GRAD_LENGTH 512

//Values in grid (and .map file)
#define MAP_GOAL      0
#define MAP_START    -1
#define MAP_UNKNOWN  -2
#define MAP_OBSTACLE -3
#define MAP_BORDER   -4
#define MAP_ERROR    -5


//Map file stuff:

//Symbol file conversions
#define SYM2MAP_GOAL     'G'
#define SYM2MAP_START    'S'
#define SYM2MAP_OBSTACLE '#'
#define SYM2MAP_BORDER   '@'
#define SYM2MAP_ERROR    '?'
#define SYM2MAP_UNKNOWN  "   "

#define MAP2SYM_GOAL     " G "
#define MAP2SYM_START    " S "
#define MAP2SYM_OBSTACLE "###"
#define MAP2SYM_BORDER   "@@@"
#define MAP2SYM_ERROR    "???"
#define MAP2SYM_UNKNOWN  "   "

//Image file conversions
//Goal
#define R_GL 255
#define G_GL 0
#define B_GL 0
//Start
#define R_ST 0
#define G_ST 255
#define B_ST 0
//Unknown
#define R_UN 255
#define G_UN 255
#define B_UN 255
//Obstacle
#define R_OB 99
#define G_OB 99
#define B_OB 99
//Border
#define R_BO 0
#define G_BO 0
#define B_BO 0
//Error value
#define R_ER 0
#define G_ER 0
#define B_ER 0

//Map file constraints
#define MAX_LINE_LEN  1024 //Max length of header lines

//Filetypes
#define SYM 0  //Symbol file (useful for printing)
#define MAP 1  //Map file (internal representation)
#define IMG 2  //Image file (useful to view maps)

//File names
#define MAX_FILENAME_SIZE 64
#define DEF_RD_IMG_FILE   "def_map.ppm"
#define DEF_WR_IMG_FILE   "new_map.ppm"
#define DEF_RD_MAP_FILE   "def_map.map"
#define DEF_WR_MAP_FILE   "new_map.map"
#define DEF_RD_SYM_FILE   "def_map.txt"
#define DEF_WR_SYM_FILE   "new_map.txt"

//Special chars
#define CR 0x0D
#define LF 0x0A
//=============================================================================


//=============================================================================
//Structures
//=============================================================================
//Map position
typedef struct
{
	int i;
	int j;
}Coordinates;

//Pixel in image file
typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
}Pixel;

//Map struct
typedef struct
{
	int** map;
	int height;
	int width;
	int max_value;
	int gradient_length;
	Pixel* gradient;
} Map;
//=============================================================================


//=============================================================================
//Function prototypes
//=============================================================================

//Create an empty map with the properties specified in the file "file"
//This should usually only be called by CreateMapFromFile()
Map* ProcessHeaders(FILE* file);

//Create a map with the properties specified in the default file of type "type"
Map* CreateMapFromDefaultFile(int type);

//Create a map with the properties specified in the file called "filename"
Map* CreateMapFromFile(int type, char* filename);

//Save map to default file of type "type"
void SaveMapToDefaultFile(Map* map, int type);

//Save map to file called "filename", of type "type"
void SaveMapToFile(Map* map, int type, char* filename);

//Create an empty map, with width, height, max_value (-1 for default max_value)
Map* CreateEmptyMap(int width, int height, int max_value);

//Create a bordered map, with width, height, max_value (-1 for default max_value)
Map* CreateBorderedMap(int width, int height, int max_value);

//Remove all but borders and obstacles from map
void ClearMap(Map* map);

//Release memory associated with map
void FreeMap(Map* map);

//Change the value in cell [i][j] of map to value "value"
void ChangeCellState(Map* map, int i, int j, int value);

//Get value in cell [i][j] of map
int GetCellState(Map* map, int i, int j);

//Print map to standard output
void PrintMap(Map* map);

//Return [i][j] coordinates of first position found to have start value
Coordinates FindStart(Map* map);

//Return [i][j] coordinates of first position found to have goal value
Coordinates FindGoal(Map* map);

//Return [i][j] coordinates of first position found to have the max value in map
Coordinates FindMax(Map* map);

//Return [i][j] coordinates of first position found to have the min value in map
Coordinates FindMin(Map* map);

//Return [i][j] coordinates of first position found to have the value "value" in map
Coordinates FindValue(Map* map, int value);

//Return the highest value found in map
int GetMaxValue(Map* map);

//Return the lowest value found in map
int GetMinValue(Map* map);

//Fill the gradient array (used to draw a gradient path: G->B->R)
void FillGradient(Map* map, int length);

//Value conversions for converting between various file types
int ImageToMap(Map* map, Pixel pix);
Pixel MapToImage(Map* map, int num);
void MapToSymbol(int num, char* str);
int SymbolToMap(char* str);
//=============================================================================


//=============================================================================
#endif //#ifndef maps_h_DEFINED
//=============================================================================
