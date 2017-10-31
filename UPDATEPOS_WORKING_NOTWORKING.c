void UpdatePos     (){
  Posture pos;
	Steps s         = GetSteps();
  float distLeft  = s.l - lWheelGlobal;
  float distRight = s.r - rWheelGlobal;

  rWheelGlobal    = s.r;
  lWheelGlobal    = s.l;

  float lWheelDist    = distLeft*MM_PER_PULSE_L;
  float rWheelDist   = distRight*MM_PER_PULSE_R;

  float dist = (lWheelDist + rWheelDist)/2;
  float delta = (rWheelDist - lWheelDist)/ROBOT_DIAMETER;
  float dx = dist* cos(delta/2);
  float dy = dist* sin(delta/2);

  pos = GetPosture();

  pos.x = pos.x + dx*cos(pos.th) - dy*sin(pos.th);
  pos.y = pos.y + dx*sin(pos.th) + dy*cos(pos.th);
  pos.th = pos.th + delta;
  float th = pos.th;
      th = fmodf(th,2*PI);
  SetPosture((float)pos.x, (float)pos.y,(float)pos.th);
  //printf(" UPDATE TH: %f\n", DEG(th));
	// float rWheelDist= (s.r - rWheelGlobal) * MM_PER_PULSE; 							 //calc moved dist per wheel
	// float lWheelDist= (s.l - lWheelGlobal) * MM_PER_PULSE; 							 // -||-
	// float delta     = ((s.r * MM_PER_PULSE) - (s.l * MM_PER_PULSE)) /52.55; //moved dist unit
	// float dist      = ((rWheelDist + lWheelDist ) / 2.0); 								 //travelled dist
	// ClearSteps();
	// float dx        = dist * (cos(delta/2.0)); 														 //displacement x
	// float dy        = dist * (sin(delta/2.0));														 //displacement y
	// Posture p       = GetPosture(); 														 					 //get the current readings of robot pos
  //
	// float x         = p.x + (dx * cos(p.th)) - (dy * sin(p.th)); 					 //calc new x coords
	// float y         = p.y + (dx * sin(p.th)) + (dy * cos(p.th)); 					 //calc new y coords
	// float th        = p.th + delta; 														 					 // calc new th value
  //       th        = fmod(th,(PI*2)); 														 					 // conversion into mod
  //
	//  printf            ("\nupdate_pos p.x = %.4f, p.y = %.4f, p.th = %.4f DEG \n",p.x,p.y,DEG(th));
	// // printf            ("\nDistance = %f\n",dist);
	// //*prev           = GetSteps();
	// SetPosture        (x,y,th); 																					 //send away new calculated values for exec
	// p               = GetPosture();
}



void UpdatePos     (){
	Steps s         = GetSteps();
	// printf            ("\nPrev -> r %d\n",prev->r);
	// printf            ("\nPrev -> l %d\n",prev->l);

	float rWheelDist= ((float)s.r - (float)rWheelGlobal) * 0.129; 							 //calc moved dist per wheel
	float lWheelDist= ((float)s.l - (float)lWheelGlobal) * 0.129; 							 // -||-
	float delta     = (((float)s.r * 0.129) - ((float)s.l * 0.129)) /52.5; //moved dist unit
	float dist      = ((rWheelDist + lWheelDist ) / 2.0); 								 //travelled dist
	ClearSteps();
	float dx        = dist * (cos(delta/2.0)); 														 //displacement x
	float dy        = dist * (sin(delta/2.0));														 //displacement y
	Posture p       = GetPosture(); 														 					 //get the current readings of robot pos

	float x         = p.x + (dx * cos(p.th)) - (dy * sin(p.th)); 					 //calc new x coords
	float y         = p.y + (dx * sin(p.th)) + (dy * cos(p.th)); 					 //calc new y coords
	float th        = p.th + delta; 														 					 // calc new th value
        th        = fmod(th,PI*2); 														 					 // conversion into mod

	// printf            ("\nupdate_pos p.x = %.4f, p.y = %.4f, p.th = %.4f DEG \n",p.x,p.y,DEG(th));
	// printf            ("\nDistance = %f\n",dist);
	//*prev           = GetSteps();
	SetPosture        (x,y,th); 																					 //send away new calculated values for exec
	p               = GetPosture();
}
