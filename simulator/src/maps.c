//=============================================================================
//map.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "maps.h"
//=============================================================================


//=============================================================================
//Functions
//=============================================================================
Map* CreateBorderedMap(int width, int height, int max_value)
{
  int i,j;

  Map* map = CreateEmptyMap(width, height, max_value);

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return NULL;
  }

  for (i=0;i<map->height;i++)
  for (j=0;j<map->width;j++)
  {
    if ((i == 0) || (i == map->height-1))
      map->map[i][j] = MAP_BORDER;
    else if ((j == 0) || (j == map->width-1))
      map->map[i][j] = MAP_BORDER;
  }

  return map;
}
//=============================================================================
Map* CreateEmptyMap(int width, int height, int max_value)
{
  int i,j;
  Map* map = (Map*)malloc(sizeof(Map));

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return NULL;
  }

  map->width = width;
  map->height = height;
  map->gradient_length = 0;
  map->gradient = NULL;

  if (max_value < 0)
    map->max_value = DEFAULT_MAX_VALUE;
  else
    map->max_value = max_value;

  map->map = (int**)malloc(height*sizeof(int*));
  for (i=0;i<map->height;i++)
  {
    map->map[i] = (int*)malloc(width*sizeof(int));
    for (j=0;j<map->width;j++)
      map->map[i][j] = MAP_UNKNOWN;
  }
  return map;
}
//=============================================================================
void FreeMap(Map* map)
{
  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  free(map->map);
  free(map->gradient);
  free(map);
  map = NULL;
}
//=============================================================================
void ClearMap(Map* map)
{
  int i,j;

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  for (i=0; i<map->height; i++)
  for (j=0; j<map->width; j++)
  {
    switch(map->map[i][j])
    {
      case MAP_UNKNOWN:
      case MAP_OBSTACLE:
      case MAP_BORDER:
        break;
      default:
        map->map[i][j] = MAP_UNKNOWN;
    }
  }
}
//=============================================================================
void ChangeCellState(Map* map, int i, int j, int value)
{
  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  map->map[i][j] = value;
}
//=============================================================================
int GetCellState(Map* map, int i, int j)
{
  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return -1;
  }

  return map->map[i][j];
}
//=============================================================================
void PrintMap(Map* map)
{
  int i,j;
  char str[12];

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  printf("\n");

  //Print info
  for (i=0; i<map->height; i++)
  {
    for (j=0; j<map->width; j++)
    {
      MapToSymbol(map->map[i][j],str);
      if (j < map->width-1)
       printf("%s ",str);
      else
        printf("%s",str);
    }
    printf("\n");
  }
  printf("Width: %3d  Height: %3d\n", map->width, map->height);
}
//=============================================================================
Coordinates FindStart(Map* map)
{
  Coordinates ret = {-1,-1};

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return ret;
  }

  return FindValue(map, MAP_START);
}
//=============================================================================
Coordinates FindGoal(Map* map)
{
  Coordinates temp = {-1,-1};

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return temp;
  }

  return FindValue(map, MAP_GOAL);
}
//=============================================================================
Coordinates FindMax(Map* map)
{
  int i,j;
  int max = -1;
  Coordinates ret = {-1,-1};

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return ret;
  }

  for (i=0;i<map->height;i++)
  for (j=0;j<map->width;j++)
  {
    if (map->map[i][j] > max)
    {
      max = map->map[i][j];
      ret.i = i;
      ret.j = j;
    }
  }
  return ret;
}
//=============================================================================
Coordinates FindMin(Map* map)
{
  int i,j;
  int min = map->max_value;
  Coordinates ret = {-1,-1};

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return ret;
  }

  for (i=0;i<map->height;i++)
  for (j=0;j<map->width;j++)
  {
    if (map->map[i][j] < min)
    {
      min = map->map[i][j];
      ret.i = i;
      ret.j = j;
    }
  }
  return ret;
}
//=============================================================================
int GetMaxValue(Map* map)
{
  int i,j;
  int max = -1;

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return -1;
  }

  for (i=0;i<map->height;i++)
  for (j=0;j<map->width;j++)
  {
    if (map->map[i][j] > max)
      max = map->map[i][j];
  }
  return max;
}
//=============================================================================
int GetMinValue(Map* map)
{
  int i,j;
  int min = map->max_value;

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return -1;
  }

  for (i=0;i<map->height;i++)
  for (j=0;j<map->width;j++)
  {
    if (map->map[i][j] < min)
      min = map->map[i][j];
  }
  return min;
}
//=============================================================================
Coordinates FindValue(Map* map, int value)
{
  Coordinates temp = {-1,-1};

  int i,j;
  int count = 0;

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return temp;
  }


  for (i=0;i<map->height;i++)
  for (j=0;j<map->width;j++)
  {
    if (map->map[i][j] == value)
    {
      temp.i = i;
      temp.j = j;
      count++;
    }
  }

  switch(count)
  {
    case 0:
      #if TraceErrors
      printf("Value not found!\n");
      #endif
      break;
    case 1:
      break;
    default:
      #if TraceErrors
      printf("Several cells have value - sending last one!\n");
      #endif
  }

  return temp;

}
//=============================================================================
Map* ProcessHeaders(FILE* file)
{
  Map* map;
  int num_read, max_value, width, height;
  char line[MAX_LINE_LEN];

  //Ignore first two lines
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    #if TraceErrors
    printf("Error reading first line from file!\n");
    #endif
    fclose(file);
    return NULL;
  }

  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    #if TraceErrors
    printf("Error reading second line from file!\n");
    #endif
    fclose(file);
    return NULL;
  }

  //Read number of rows and columns
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    #if TraceErrors
    printf("Error reading third line from file!\n");
    #endif
    fclose(file);
    return NULL;
  }

  num_read = sscanf(line,"%d %d",&width,&height);
  if (num_read != 2)
  {
    #if TraceErrors
    printf("Error reading number of rows and columns!\n");
    #endif
    fclose(file);
    return NULL;
  }

  //Read max value
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    #if TraceErrors
    printf("Error reading max value line from file!\n");
    #endif
    fclose(file);
    return NULL;
  }

  num_read = sscanf(line,"%d",&max_value);
  if (num_read != 1)
  {
    #if TraceErrors
    printf("Error reading max value!\n");
    #endif
    fclose(file);
    return NULL;
  }

  //Create map
  map = CreateEmptyMap(width, height, max_value);

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return NULL;
  }

  map->max_value = max_value;

  return map;
}
//=============================================================================
Map* CreateMapFromDefaultFile(int type)
{
  Map* map;

  switch(type)
  {
    case SYM:
      map = CreateMapFromFile(type, DEF_RD_SYM_FILE);
      break;
    case MAP:
      map = CreateMapFromFile(type, DEF_RD_MAP_FILE);
      break;
    case IMG:
      map = CreateMapFromFile(type, DEF_RD_IMG_FILE);
      break;
    default:
      #if TraceErrors
      printf("maps: LoadMapFromDefaultFile: invalid type!\n");
      #endif
      return NULL;
  }
  return map;
}
//=============================================================================
Map* CreateMapFromFile(int type, char* filename)
{
  Map* map;
  FILE* file;
  int i,j,r,g,b;
  int num_read;
  char str[12];
  Pixel pix;

  // Open a file
  file = fopen(filename, "r");
  if (file == NULL)
  {
    #if TraceErrors
    printf("Could not open file '%s'!\n", filename);
    #endif
    return NULL;
  }

  map = ProcessHeaders(file);

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error processing headers!\n");
    #endif
    return NULL;
  }

  //Read data from file
  for(i=0;i<map->height;i++)
  for(j=0;j<map->width;j++)
  {
    if (type == MAP)
    {
      //Read one value at a time
      num_read = fscanf(file,"%d",&map->map[i][j]);
      if (num_read != 1)
      {
        #if TraceErrors
        printf("Error extracting data from file!\n");
        #endif
        fclose(file);
        return NULL;
      }
    }
    else if (type == IMG)
    {
      //Read one value at a time
      num_read = fscanf(file,"%d %d %d",&r,&g,&b);
      if (num_read != 3)
      {
        #if TraceErrors
        printf("Error extracting data from file!\n");
        #endif
        fclose(file);
        return NULL;
      }

      pix.r = (unsigned char)r;
      pix.g = (unsigned char)g;
      pix.b = (unsigned char)b;

      //Convert to "map" format
      map->map[i][j] = ImageToMap(map, pix);
    }
    else if (type == SYM)
    {
      //Read string
      num_read = fscanf(file,"%c",&str[0]);
      if (num_read != 1)
      {
        #if TraceErrors
        printf("Error extracting data from file!\n");
        #endif
        fclose(file);
        return NULL;
      }

      //Check for lf or cr - if found, don't count this char
      if ((str[0] == LF) || (str[0] == CR))
        j--;
      else
      {
        num_read = fscanf(file,"%c%c%*c",&str[1],&str[2]);
        str[3] = '\0';
        if (num_read != 2)
        {
          #if TraceErrors
          printf("Error extracting data from file!\n");
          #endif
          fclose(file);
          return NULL;
        }

        //Convert char to map...
        map->map[i][j] = SymbolToMap(str);
      }
    }
    else
    {
      #if TraceErrors
      printf("maps: LoadMapFromFile: invalid type!\n");
      #endif
    }
  }
  //Close file
  fclose(file);

  return map;
}
//=============================================================================
void SaveMapToDefaultFile(Map* map, int type)
{
  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  switch(type)
  {
    case SYM:
      SaveMapToFile(map, type, DEF_WR_SYM_FILE);
      break;
    case MAP:
      SaveMapToFile(map, type, DEF_WR_MAP_FILE);
      break;
    case IMG:
      SaveMapToFile(map, type, DEF_WR_IMG_FILE);
      break;
    default:
      #if TraceErrors
      printf("maps: SaveMapToDefaultFile: invalid type!\n");
      #endif
  }
}
//=============================================================================
void SaveMapToFile(Map* map, int type, char* filename)
{
  FILE* file;
  int i,j;
  char str[12];
  Pixel pix;


  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  //Open file
  file = fopen(filename, "w");
  if (file == NULL)
  {
    #if TraceErrors
    printf("Could not open file '%s'!\n", filename);
    #endif
    return;
  }

  //Print info to file
  if (type == IMG)
  {
    //Fill gradient
    FillGradient(map, GetMaxValue(map));

    fprintf(file,"P3\n# .ppm file: %s\n%d %d\n%d\n",
            filename, map->width, map->height, map->max_value);
    for (i=0;i<map->height;i++)
    {
      for (j=0;j<map->width;j++)
      {
        pix = MapToImage(map, map->map[i][j]);
        fprintf(file, " %4d%4d%4d", pix.r,pix.g,pix.b);
      }
      fprintf (file, "\n");
    }
  }
  else if (type == MAP)
  {
    fprintf(file,"PX\n# .map file: %s\n%d %d\n%d\n",
            filename, map->width, map->height, map->max_value);
    for (i=0;i<map->height;i++)
    {
      for (j=0;j<map->width;j++)
      {
        fprintf(file, "%5d", map->map[i][j]);
      }
      fprintf (file, "\n");
    }
  }
  else if (type == SYM)
  {
    //Print header lines
    fprintf(file,"PX\n# .txt file: %s\n%d %d\n%d\n",
            filename, map->width, map->height, map->max_value);

    //Print info to file
    for (i=0; i<map->height; i++)
    {
      for (j=0; j<map->width; j++)
      {
        MapToSymbol(map->map[i][j],str);
        if (j < map->width-1)
         fprintf(file,"%s ",str);
        else
          fprintf(file,"%s",str);
      }
      fprintf(file,"\n");
    }
    fprintf(file,"\n");
  }
  else
  {
    #if TraceErrors
    printf("maps: LoadMapFromDefaultFile: invalid type!\n");
    #endif
  }

  //Close file
  fclose(file);
}
//=============================================================================
void FillGradient(Map* map, int length)
{
  int i;
  double ratio = ((double)(map->max_value))/((double)(length>>1));

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return;
  }

  map->gradient_length = length;
  map->gradient = (Pixel*)malloc(length*sizeof(Pixel));

  //Initialise length elements in the gradient
  //Fill [0-length/2]
  for (i=0;i<=(length>>1);i++)
  {
    map->gradient[i].r = (unsigned char)(((length>>1)-i)*ratio);
    map->gradient[i].g = 0;
    map->gradient[i].b = (unsigned char)(i*ratio);
  }

  //Fill [length/2-length]
  for (i=1;i<(length>>1);i++)
  {
    map->gradient[i+(length>>1)].r = 0;
    map->gradient[i+(length>>1)].g = (unsigned char)(i*ratio);
    map->gradient[i+(length>>1)].b = (unsigned char)(((length>>1)-i)*ratio);
  }
}
//=============================================================================
int ImageToMap(Map* map, Pixel pix)
{
  int i;

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return -1;
  }

  if ((pix.r == R_GL) && (pix.g == G_GL) && (pix.b == B_GL))
    return MAP_GOAL;
  if ((pix.r == R_ST) && (pix.g == G_ST) && (pix.b == B_ST))
    return MAP_START;
  if ((pix.r == R_UN) && (pix.g == G_UN) && (pix.b == B_UN))
    return MAP_UNKNOWN;
  if ((pix.r == R_OB) && (pix.g == G_OB) && (pix.b == B_OB))
    return MAP_OBSTACLE;
  if ((pix.r == R_BO) && (pix.g == G_BO) && (pix.b == B_BO))
    return MAP_BORDER;
  if ((pix.r == R_ER) && (pix.g == G_ER) && (pix.b == B_ER))
    return MAP_ERROR;

  FillGradient(map, GetMaxValue(map));

  //Extract values from gradient
  for (i=0;i<map->gradient_length;i++)
  {
    if ((pix.r == map->gradient[i].r) &&
        (pix.g == map->gradient[i].g) &&
        (pix.b == map->gradient[i].b))
      return i;
  }

  #if TraceErrors
  printf("Unknown value in map...\n");
  #endif

  return MAP_ERROR;
}
//=============================================================================
Pixel MapToImage(Map* map, int num)
{
  Pixel temp = {0,0,0};

  if (map == NULL)
  {
    #if TraceErrors
    printf("Error: map not initialised!\n");
    #endif
    return temp;
  }

  switch(num)
  {
    case MAP_GOAL:
      temp.r = R_GL;
      temp.g = G_GL;
      temp.b = B_GL;
      break;
    case MAP_START:
      temp.r = R_ST;
      temp.g = G_ST;
      temp.b = B_ST;
      break;
    case MAP_UNKNOWN:
      temp.r = R_UN;
      temp.g = G_UN;
      temp.b = B_UN;
      break;
    case MAP_OBSTACLE:
      temp.r = R_OB;
      temp.g = G_OB;
      temp.b = B_OB;
      break;
    case MAP_BORDER:
      temp.r = R_BO;
      temp.g = G_BO;
      temp.b = B_BO;
      break;
    default:
      if (num > map->gradient_length)
      {
        #if TracErrors
        printf("Unknown value!");
        #endif
        temp.r = R_ER;
        temp.r = G_ER;
        temp.r = B_ER;
      }
      else
      {
        temp.r = map->gradient[num].r;
        temp.g = map->gradient[num].g;
        temp.b = map->gradient[num].b;
      }
  }

  return temp;

}
//=============================================================================
void MapToSymbol(int num, char* str)
{
  switch(num)
  {
    case MAP_GOAL:
      strcpy(str,MAP2SYM_GOAL);
      break;
    case MAP_START:
      strcpy(str,MAP2SYM_START);
      break;
    case MAP_OBSTACLE:
      strcpy(str,MAP2SYM_OBSTACLE);
      break;
    case MAP_BORDER:
      strcpy(str,MAP2SYM_BORDER);
      break;
    case MAP_UNKNOWN:
      strcpy(str,MAP2SYM_UNKNOWN);
      break;
    case MAP_ERROR:
      strcpy(str,MAP2SYM_ERROR);
      break;
    default:
      if (num > 0)
        sprintf(str, "%3d", num);
      else
        strcpy(str,"???");
  }
}
//=============================================================================
int SymbolToMap(char* str)
{
  int temp;
  switch(str[1])
  {
    case SYM2MAP_GOAL:
      return MAP_GOAL;
    case SYM2MAP_START:
      return MAP_START;
    case SYM2MAP_OBSTACLE:
      return MAP_OBSTACLE;
    case SYM2MAP_BORDER:
      return MAP_BORDER;
    case SYM2MAP_ERROR:
      return MAP_ERROR;
  }

  if (strcmp(SYM2MAP_UNKNOWN,str) == 0)
    return MAP_UNKNOWN;

  sscanf(str, "%d", &temp);
  if (temp < 0)
    return MAP_ERROR;
  else
    return temp;
}
//=============================================================================
