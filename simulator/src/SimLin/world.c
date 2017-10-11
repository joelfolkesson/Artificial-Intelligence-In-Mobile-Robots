/*****************************************************************************/
/* File:        world.c (Khepera Simulator)                                  */
/* Author:      Olivier MICHEL <om@alto.unice.fr>                            */
/* Date:        Thu Sep 21 14:39:05 1995                                     */
/* Description: world management                                             */
/*                                                                           */
/* Copyright (c) 1995                                                        */
/* Olivier MICHEL                                                            */
/* MAGE team, i3S laboratory,                                                */
/* CNRS, University of Nice - Sophia Antipolis, FRANCE                       */
/*                                                                           */
/* Permission is hereby granted to copy this package for free distribution.  */
/* The author's name and this copyright notice must be included in any copy. */
/* Commercial use is forbidden.                                              */
/*****************************************************************************/

#include "header.h"
#include "world.h"
#include "sim.h"

void AddObject(struct World  *world,struct Object *object)
{
  struct Object **obj;
  int i,j,shift;
  double brick_xmin, brick_xmax, brick_ymin, brick_ymax; //--KL
  u_long xmin_val, xmax_val;

  /* LAMPS are added first, other objects are added last */

  if (object->Type == LAMP)
  {
    object->Next = world->Objects;
    world->Objects = object;
  }
  else
  {
    obj = &(world->Objects);
    while(*obj) obj = &((*obj)->Next);
    *obj = object;
  }

  //--KL - if bricks are added, world.Image[] should be modified
  //For now only use approximate bounding box
  if (object->Type == BRICK)
  { 
    //printf("\fBrick position:\n");
    //printf("  brick_x: %d\n", object->X);
    //printf("  brick_y: %d\n", object->Y);
    //printf("  brick_a: %lf\n", object->Alpha);
    
    //Find exact brick position in image coordinates (0.98125 is a calibration factor)
    brick_xmin = MIN(MAX( (object->X*0.98125 - MAX(0.5*BRICKWidth*fabs(sin(object->Alpha)),0.5*BRICKHeight*fabs(cos(object->Alpha))))  ,0),WORLD_W);
    brick_xmax = MIN(MAX( (object->X*0.98125 + MAX(0.5*BRICKWidth*fabs(sin(object->Alpha)),0.5*BRICKHeight*fabs(cos(object->Alpha))))  ,0),WORLD_W);
    brick_ymin = MIN(MAX( (object->Y - MAX(0.5*BRICKWidth*fabs(cos(object->Alpha)),0.5*BRICKHeight*fabs(sin(object->Alpha))))  ,0),WORLD_H);
    brick_ymax = MIN(MAX( (object->Y + MAX(0.5*BRICKWidth*fabs(cos(object->Alpha)),0.5*BRICKHeight*fabs(sin(object->Alpha))))  ,0),WORLD_H);
    
    //printf("Image coordinates:\n");
    //printf("  brick_xmin: %lf\n", brick_xmin);
    //printf("  brick_xmax: %lf\n", brick_xmax);
    //printf("  brick_ymin: %lf\n", brick_ymin);
    //printf("  brick_ymax: %lf\n", brick_ymax);
    
    //Translate these to array coordinates
    brick_xmin = MAX(0,MIN(15.99,brick_xmin*16.0/WORLD_W));
    brick_xmax = MAX(0,MIN(15.99,brick_xmax*16.0/WORLD_W));
    brick_ymin = MAX(0,MIN(499.99,(int)((brick_ymin*500.0/WORLD_H)+0.5)  ));
    brick_ymax = MAX(0,MIN(499.99,(int)((brick_ymax*500.0/WORLD_H)+0.5)  ));

    //printf("\nArray coordinates:\n");
    //printf("  brick_xmin: %lf\n", brick_xmin);
    //printf("  brick_xmax: %lf\n", brick_xmax);
    //printf("  brick_ymin: %lf\n", brick_ymin);
    //printf("  brick_ymax: %lf\n", brick_ymax);


    //Put 1's in the border positions
    //Use shift and check for >= 32 to workaround gcc/g++ bug
    if (floor(brick_xmin) != floor(brick_xmax))
    {
    
      shift = MAX(0,(int)rint((brick_xmin - (double)floor(brick_xmin))*32.0) - 1);
      if (shift >= 32)
        xmin_val = 0;
      else
        xmin_val = 0xffffffff << shift;
      
      shift = (int)rint((1.0-(brick_xmax - (double)floor(brick_xmax)))*32.0) + 1;
      if (shift >= 32)
        xmax_val = 0;
      else
        xmax_val = 0xffffffff >> shift;
    }
    else
    {
      shift = MAX(0,(int)rint((brick_xmin - (double)floor(brick_xmin))*32.0) - 1);
      if (shift >= 32)
        xmin_val = 0;
      else
        xmin_val = 0xffffffff << shift;

      shift = (int)rint((1.0-(brick_xmax - (double)floor(brick_xmax)))*32.0) + 1;
      if (shift >= 32)
        xmin_val = 0;
      else
      {
        xmin_val <<= shift;
        xmin_val >>= shift;
      }
      xmax_val = xmin_val;
    }
    
    //printf("\nxmin_val = %08lx\n",xmin_val);
    //printf("xmax_val = %08lx\n",xmax_val);

    //Put values in image
    for (i=(int)floor(brick_xmin);i<=(int)floor(brick_xmax);i++)
    for (j=(int)brick_ymin;j<(int)brick_ymax;j++)
    {
      if (i == (int)floor(brick_xmin))
        world->Image[i][j] = xmin_val;
      else if (i == (int)floor(brick_xmax))
        world->Image[i][j] = xmax_val;
      else
        world->Image[i][j] = 0xffffffff;
      //printf("(%d,%d,%08lx) ",i,j,world->Image[i][j]);
    }
    //printf("\n\n");
  }
  
}

struct Object *CreateObject(u_char type,short int x,short int y,double alpha)
{
  struct Object *object;

  object              = (struct Object *)malloc(sizeof(struct Object));
  object->Type        = type;
  object->X           = x;
  object->Y           = y;
  object->Alpha       = alpha;
  object->Next        = NULL;
  return(object);
}

void FreeObjects(struct Object *object)
{
  if (object)
  { 
    FreeObjects(object->Next);
    FreeObject(object);
    
    //--KL should also modify world->image?!
  }
}

void CreateEmptyWorld(struct Context *context)
{
  short int i;
  //short int j,s;
  struct World *world;
  //FILE *file;
  //char text[256];

  world = context->World;
  FreeObjects(world->Objects);
  strcpy(world->Name,"new");
  world->Objects = (struct Object *)NULL;
  world->ObjectType = BRICK;
  world->BehindObject = NULL;
  world->BehindX = 0;
  world->BehindY = 0;
  for(i=0;i<N_OBJECTS;i++) world->ObjectAlpha[i]=0;
  strcpy(world->ObjectName[BRICK],"brick");
  strcpy(world->ObjectName[CORK],"cork");
  strcpy(world->ObjectName[LAMP],"lamp");
}

void CreateDefaultWorld(struct Context *context)
{
  short int i,j;
  struct World *world;

  CreateEmptyWorld(context);
  world = context->World;
  strcpy(world->Name,"default");
  for(i=25;i<WORLD_W;i+=50)
  {
    AddObject(world,CreateObject(BRICK,i,14,0.0));
    AddObject(world,CreateObject(BRICK,i,987,0.0));
    //--KL - Put bricks along bottom for 60x60cm field
    //for (j=650;j<WORLD_W;j+=20)
    //  AddObject(world,CreateObject(BRICK,i,j,0.0));
  }
  for(j=50;j<WORLD_H;j+=50)
  {
    AddObject(world,CreateObject(BRICK,10,j,M_PI/2));
    AddObject(world,CreateObject(BRICK,989,j,M_PI/2));

    //--KL - Put bricks along right side for 60cmx60cmfield
    //for (i=648;i<WORLD_H;i+=20)
    //  AddObject(world,CreateObject(BRICK,i,j,M_PI/2));
  }
  
  
  //--KL though AddObject should modify the Image, this is good to
  //do anyway, to "clear" the old image (rather than relying on free objects to
  //remove the objects from the image)

  //Draw top edge
  for(i=0;i<16;i++)
  for(j=0;j<13;j++)
    world->Image[i][j] = (u_long)0xffffffff;
  
  //Draw middle
  for(j=13;j<488;j++)
  {
    //Draw left edge
    world->Image[0][j] = (u_long)0x00007ff;
    for(i=1;i<15;i++)
    {
      world->Image[i][j] = (u_long)0x00000000;

      //--KL - Draw vertical edge from 640-1000
      //if (i > 9)
      //  world->Image[i][j] = (u_long)0xffffffff;
    }
    //Draw right edge
    world->Image[15][j] = (u_long)0x00ffe00;
  }
  
  //Draw bottom edge
  for(i=0;i<16;i++)
  for(j=488;j<500;j++)
    world->Image[i][j]=(u_long)0xffffffff;

  
  //--KL - Draw horizontal edge from 640-1000
  //for(i=0;i<16;i++)
  //for(j=320;j<500;j++)
  //  world->Image[i][j]=(u_long)0xffffffff;
}

void CreateChaoWorld(struct Context *context)
{
  short int i;
  //short int j;
  struct World *world;

  CreateDefaultWorld(context);
  world = context->World;
  strcpy(world->Name,"chao");
  for(i=0;i<90;i++)
  {
    AddObject(world,
     CreateObject(BRICK,Rnd(950)+25,Rnd(950)+25,(double)Rnd(1000)*M_PI/1000));
  }
}

void FreeWorld(struct Context *context)
{
  //short int i,j;
  struct World *world;

  world = context->World;
  FreeObjects(world->Objects);
  free(world);
}

void RemoveObject(struct World  *world,struct Object *object)
{
  struct Object **search;

  if (object)
  {
    search = &(world->Objects);
    while(*search)
    {
      if (*search == object)
      {
        *search = object->Next;
        FreeObject(object);
      }
      else search = &((*search)->Next);
    }
  }
  
  //--KL must also modify world.image!
  
}

struct Object *FindObject(struct World *world,short int x,short int y)
{
  struct Object *search,*found;

  search = world->Objects;
  found  = NULL;
  while (search)
  {
    if ((search->X - x < 7)&&(search->X - x > -7)&&
        (search->Y - y < 7)&&(search->Y - y > -7))
    {
      found  = search;
      search = NULL;
    }
    else search = search->Next;
  }
  return(found);
}

void WriteWorldToFile(struct World *world,FILE *file)
{
  u_char obj;
  short int i,j;
  struct Object *object;

  if (world->Objects) obj = 1; else obj = 0;
  fprintf(file,"%d,%d\n",obj,world->ObjectType);
  for(i=0;i<N_OBJECTS;i++)
   fprintf(file,"%s\n%d\n",world->ObjectName[i],world->ObjectAlpha[i]);
  for(i=0;i<16;i++)for(j=0;j<500;j++) fprintf(file,"%lx,",world->Image[i][j]);
  object = world->Objects;
  while(object)
  {
    if (object->Next) obj = 1; else obj = 0;
    fprintf(file,"%d,%d,%d,%d,%d\n",object->Type,object->X,object->Y,
                                     (int)(object->Alpha*180/M_PI),obj);
    object = object->Next;
  }
}

void ReadWorldFromFile(struct World *world,FILE *file)
{
  short int i,j,obj,type;
  int catch = 0;
  struct Object **object;

  catch = fscanf(file,"%hd,%hd\n",&obj,&type);
  world->ObjectType = (u_char)type;
  if (obj) world->Objects = (struct Object *)1; else world->Objects = NULL;
  for(i=0;i<N_OBJECTS;i++)
   catch = fscanf(file,"%s\n%hd\n",world->ObjectName[i],&(world->ObjectAlpha[i]));
  for(i=0;i<16;i++) for(j=0;j<500;j++)
   catch = fscanf(file,"%lx,",&(world->Image[i][j]));
  world->BehindX = 0;
  world->BehindY = 0;
  world->BehindObject = NULL;
  object = &(world->Objects);
  while(*object)
  {
    *object = (struct Object *)malloc(sizeof(struct Object));
    catch = fscanf(file,"%hd,%hd,%hd,%lf,%hd\n",&type,&((*object)->X),
                                   &((*object)->Y),&((*object)->Alpha),&obj);
    (*object)->Type = (u_char)type;
    (*object)->Alpha *= M_PI/180.0;
    if (obj) (*object)->Next = (struct Object *)1;
    else (*object)->Next = NULL;
    object = &((*object)->Next);
  }

//todo
if(catch) {}
}

void ChooseRandomPosition(struct World *world,double *x,double *y,
                          double *alpha)
{
  u_char    success;
  short int ix,iy;

  do
  {
    ix = (Rnd(900) + 50)/2;
    iy = (Rnd(900) + 50)/2;
    if ((world->Image[ix/32][iy]         & (1 << (ix%32)))||
        (world->Image[(ix+15)/32][iy]    & (1 << ((ix+15)%32)))||
        (world->Image[(ix-15)/32][iy]    & (1 << ((ix-15)%32)))||
        (world->Image[ix/32][iy+15]      & (1 << (ix%32)))||
        (world->Image[ix/32][iy-15]      & (1 << (ix%32)))||
        (world->Image[(ix+11)/32][iy+11] & (1 << ((ix+11)%32)))||
        (world->Image[(ix+11)/32][iy-11] & (1 << ((ix+11)%32)))||
        (world->Image[(ix-11)/32][iy+11] & (1 << ((ix-11)%32)))||
        (world->Image[(ix-11)/32][iy-11] & (1 << ((ix-11)%32))))
         success = FALSE; else success = TRUE;
  }
  while(success == FALSE);
  *x = (double)ix*2;
  *y = (double)iy*2;
  *alpha = (2.0*M_PI*Rnd(1000))/999.0;
}
