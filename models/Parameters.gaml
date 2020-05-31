/**
* Name: Parameters
* Author: Annalisa Congiu
* Description: parameters and variables for the simulations and common agents
* Tags: Tag1, Tag2, TagN
*/

model Parameters

/* List of parameters and global variables used in the simulations and common agents */

global{
	//represent the day time for the agent to inform them to go work or home
	float starting_current_hour <- 7.25;
	float current_hour <- starting_current_hour update: (((time+(starting_current_hour*3600#s))/#hour)-(g)*24);
	int h update: (time/#hour) mod 24; 
	int g update: int(time/#hour/24);
	
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads);
	
	graph the_graph;  
	graph road_network;  
	map road_weights;
	map graph_weights;
	bool not_reset<-true;
	
	float step <- 5 #s;	
	
	csv_file input_file <- csv_file("../includes/input_data.csv",",");
	matrix data <- matrix(input_file);
	

	int nb_people <- int(data[3,0]);
	int modstart <- ((nb_people mod int(data[1,0])) >0) ? int(nb_people/int(data[1,0])) + 1 : int(nb_people/int(data[1,0]));
	int nb_car <- int(data[0,0]);
	int nb_cars <-  int(data[2,0]);
	int max_passengers_per_AV<-int(data[1,0]); 

	
	//Variables to manage the minimal and maximal time to start working/go home
	float min_work_start <- 8.0; 
	float max_work_start <- 10.0; 
	float min_work_end <- 17.0; 
	float max_work_end <- 18.0;  
	
	float before_work_search<-0.5; //(30 minutes)
	float before_work_start	<- 0.15;
	float after_work_start<-0.5;
	
	//Manage the speed allowed in the model for the people agents
	float min_speed <- 5.0  #km / #h;
	float max_speed <- 20.0 #km / #h; 
	
	float cost_km <- float(data[4,0]);
	
	float v_length <- 5.0#m;
	
	float stats_grouping_time <-0.0;
	float stats_path_time <-0.0;
	int n_grouping;
	
	float start_simulation;
}