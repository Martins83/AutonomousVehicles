/***
* Name: road
* Author: Annalisa Congiu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model road

import "./../models/Parameters.gaml"

//species that will represent the roads
species road skills: [skill_road] { 
	geometry geom_display;
	string oneway;
	string junction;
	float perim<-shape.perimeter;	
	int nb_agents<-length(all_agents) update: length(all_agents);
	float capacity <- 1+(shape.perimeter*lanes)/v_length;
	float speed_coeff<- 0.1 update: (length(all_agents)/capacity) min:0.1 max:((shape.perimeter < 1.0) ? 1.0 : 3.0);
	
	int ov_rgb<-150 update: 150-(150*int(speed_coeff-0.1)) min: 0 max:255; //0.1 ->150 // 1 e oltre -> 0
	int ov_rgbR<-150 update: 255*int(speed_coeff-0.1)  min: 150 max: 255; // 1 e oltre -> 255 // 0.1 -> 0
	rgb color<-rgb(127,127,127) update: rgb(ov_rgbR, ov_rgb, ov_rgb);
	
	aspect base {    
		draw shape color: color end_arrow: 6;
	} 
	aspect base3D {    
		draw geom_display color: #gray ;
	} 
}


