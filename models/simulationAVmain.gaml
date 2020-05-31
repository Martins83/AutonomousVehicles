/**
* Name: simulationAVmain
* Author: Annalisa Congiu
* Description: simulation of a AV Carpooling System
* Tags: 
*/

model simulationAVmain

import "./../models/building.gaml"
import "./../models/road.gaml"
import "./../models/intersection.gaml"
import "./../models/cars.gaml"
import "./../models/people.gaml"
import "./../models/Parameters.gaml"
import "./../models/decAV.gaml"


/* *
 * Experiments to create the map and the agents for the 
 * Decentralised simulation of a Autonomous Vehicles Carpooling system
 * */

global {
	int late <- 0;
	int were_late_with_lift <- late update: people count (each.late_at_work_with_lift);
	int were_late_no_lift <- late update: people count (each.late_at_work_no_lift);
	int init_were_passengers <- 0;
	int int_were_passengers <- init_were_passengers update: people count (each.got_lift);
	int gave_lift <- 0 update: decAV count (each.gave_lift);
	int init_were_not_passengers <- nb_people;
	int were_not_passengers <- init_were_not_passengers update: people count (each.got_lift=false);
	

	init {
		
		//create the intersection and check if there are traffic lights or not by looking the values inside the type column of the shapefile and linking
		// this column to the attribute is_traffic_signal. 
		create intersection from: shape_file_nodes with:[is_traffic_signal::(read("type") = "traffic_signals")];
		
		//create road agents using the shapefile and using the oneway column to check the orientation of the roads if there are directed
		create road from: shape_file_roads with:[lanes::int(read("lanes")), oneway::string(read("oneway")), junction::string(read("junction"))] {
			geom_display <- shape + (2.5 * lanes);
			maxspeed <- (lanes = 1 ? 30.0 : (lanes = 2 ? 50.0 : 70.0)) °km/°h;
			switch oneway {
				match "no" {
					if junction!='roundabout'{
						create road {
							lanes <- max([1, int (myself.lanes / 2.0)]);
							shape <- polyline(reverse(myself.shape.points));
							maxspeed <- myself.maxspeed;
							geom_display  <- myself.geom_display;
							linked_road <- myself;
							myself.linked_road <- self;
						}
						lanes <- int(lanes /2.0 + 0.5);
					}
				}
				match "-1" {
					shape <- polyline(reverse(shape.points));
				}	
			}
		}
		
//		create a new linked road for all oneway roads that are isolated and are not roudabouts
		ask road where ((each.linked_road)=nil and each.junction!='roundabout'){
			create road {
				lanes <- myself.lanes;
				shape <- polyline(reverse(myself.shape.points));
				maxspeed <- myself.maxspeed;
				geom_display  <- myself.geom_display;
				linked_road <- myself;
				myself.linked_road <- self;
			}
		}
		
//		create road weight for the road network 
		road_weights <- road as_map (each::(each.shape.perimeter));
		graph_weights <- road as_map (each::each.shape.perimeter);
		
		//creation of the road network using the road and intersection agents
		road_network <-  (as_driving_graph(road, intersection))  with_weights road_weights;
		the_graph<- as_edge_graph(road);
		
		//initialize the traffic light
		ask intersection {
			do initialize;
		}

//		creation of the builgings from the shape files and division between residential and work
		create building from: shape_file_buildings with:[type::string(read("type")), group::string(read("group"))]; 
		ask building{
			if group='residential'{
				color <- #slategray;
			} else{
				color <- #lightgrey;
			}
		}
		write "created buildings";
		list<building> living_buildings<- building where (each.group='residential');
		list<building> work_buildings <- building where (each.group='industrial');

//		creation of the people with random parameters
		create people number: nb_people { 
			speed <- min_speed + rnd (max_speed - min_speed) ;
			start_work <- min_work_start + rnd (max_work_start - min_work_start) ; 
			end_work <- rnd (min_work_end,max_work_end, 0.5);
			living_place <- one_of(living_buildings);
			working_place <- one_of(work_buildings);
			
			//Code for simulating only one working place
//			working_place <-  (building(31)); 
			
			//Code for simulating only two working place
//			if (flip(0.5)) {
//				working_place <- (building(21));
//			} else if (flip(0.5)) {
//				working_place <- (building(31));
//			} else {
//				working_place <- (building(31));
//			}
						
			location <- living_place.location;
		}
		
		write "created people";


		create decAV number: nb_car { 
			max_speed <- 160 °km/°h;
			vehicle_length <- v_length;
			right_side_driving <- true;
			proba_lane_change_up <- 0.1 + (rnd(500) / 500);
			proba_lane_change_down <- 0.5+ (rnd(500) / 500);
			location <- one_of(road).location;
			security_distance_coeff <- 5/9 * 3.6 * (1.5 - rnd(1000) / 1000);  
			proba_respect_priorities <- 1.0 - rnd(200/1000);
			proba_respect_stops <- [1.0];
			proba_block_node <- 0.0;
			proba_use_linked_road <- 0.0;
			max_acceleration <- 5/3.6;
			speed_coeff <- 1.2 - (rnd(400) / 1000);
			threshold_stucked <-int ( (1 + rnd(5))°mn);
			breakdown_probability <- 0.00001;
			max_passengers<-max_passengers_per_AV;
			}

//		creation of the other cars with random parameters
		create cars number: nb_cars { 
			max_speed <- 160 °km/°h;
			vehicle_length <- v_length;
			right_side_driving <- true;
			proba_lane_change_up <- 0.1 + (rnd(500) / 500);
			proba_lane_change_down <- 0.5+ (rnd(500) / 500);
			location <- one_of(intersection where empty(each.stop)).location;
			security_distance_coeff <- 5/9 * 3.6 * (1.5 - rnd(1000) / 1000);  
			proba_respect_priorities <- 1.0 - rnd(200/1000);
			proba_respect_stops <- [1.0];
			proba_block_node <- 0.0;
			proba_use_linked_road <- 0.0;
			max_acceleration <- 5/3.6;
			speed_coeff <- 1.2 - (rnd(400) / 1000);
			threshold_stucked <-int ( (1 + rnd(5))°mn);
			breakdown_probability <- 0.00001;
		}	
	}
	
	reflex update_road_speed {
		road_weights <- road as_map (each::(each.shape.perimeter * (each.speed_coeff)));
		road_network <- road_network with_weights road_weights;
	}
	
	int gotLiftAndLate <- 0 update: (people count (each.late_at_work_with_lift));
	int gotLiftAndNoLate <- 0 update: (people count (each.late_at_work_with_lift=false));
	
	int noLiftAndLate <- 0 update: (people count (each.late_at_work_no_lift));
	int noLiftAndNoLate <- 0 update: (people count (each.late_at_work_no_lift=false));
	
	action save_results {
			ask people {
				float time_late <- 0.0;
				if (start_work < arrival_time){
					time_late <- (arrival_time - start_work);
					time_late <- time_late with_precision 2;
				}
				if self.got_lift=false{
					waiting_time <- 0.0;
				} else {
				 	waiting_time <- waiting_time with_precision 2; //for those who got a lift
				}
				cost_proposed <- cost_proposed with_precision 3;
				// save the values of some of the people variables to the csv file
				save [name,arrival_time,start_work,time_late,waiting_time,got_lift,late_at_work_with_lift,cost_proposed] to: "people_data.csv" type: "csv" rewrite: false;
			}
			
			ask decAV {
				save [name,dist_covered_cars,gave_lift,cumulative_lifts_given] to: "AVs_data.csv" type: "csv" rewrite: false;
			}
		}
	
	//conditions to stop the simulation
	reflex stop_simulation when: remove_duplicates(people collect each.state)=['working'] and (length(list(people))=nb_people){
		list<people> were_passengers <- (people where (each.got_lift=true));
		list<float> costs_proposed <- were_passengers collect each.cost_proposed;
		list<float> w_times<- were_passengers collect each.waiting_time;
		were_late_with_lift<- (people count (each.late_at_work_with_lift));
		were_late_no_lift<- (people count (each.late_at_work_no_lift));
		write "passengers taken: " +length(were_passengers);
		write "passengers late at work with lift: " + were_late_with_lift;
		write "passengers late at work no lift: " + were_late_no_lift;
		write "cars used: " +length(decAV where (each.gave_lift=true));
		write 'cost proposed ('+(length(were_passengers)=length(costs_proposed))+' ' +length(costs_proposed)+'): ' + mean(costs_proposed);
		write costs_proposed;
		write 'waiting times ('+(length(were_passengers)=length(w_times))+' ' +length(w_times)+'): ' + mean(w_times);
		write w_times;
		write 'total_duration '+ total_duration + ' current_hour '+ current_hour;
		do save_results;
		do pause ;
   }
}


experiment Simulation type: gui {
	float minimum_cycle_duration <- 0.01;	
	
	parameter "People in the system" var: nb_people category: "Initial";
    parameter "AVs in the system" var:nb_car <- nb_car;
    parameter "Cars in the system" var:nb_cars <- nb_cars; 
    parameter "Max passengers per AV" var:max_passengers_per_AV; 
	
	parameter "Min. start hour: " var:min_work_start <- min_work_start category: "Hours";
	parameter "Max. start hour: "  var:max_work_start <- max_work_start category: "Hours";
	
	parameter "Minutes before work to search for lift: " var: before_work_search<-0.5 category: "Hours"; //(30 minuts)
	parameter "Minutes before work to go alone: " var: before_work_start <-0.25 category: "Hours"; // (15 minuts)
	
	output {
		monitor "Time" value: time;
		monitor "Current hour" value: current_hour;
		monitor "Late at work with lift" value: were_late_with_lift;
		
		display city_display {
			graphics "world" {
				draw world.shape.contour color:#black;
			}
			
			species building aspect: base refresh:false;
			species road aspect: base ;
			species intersection aspect: base;
			species decAV aspect: base;
			species cars aspect: base;
			species people aspect: base transparency: 0.2;
		}
		
		display chart refresh: every(100 # cycles)
		{
			chart "Runtime Simulation" type: series style: line size: {1,0.5} position: {0, 0} color: #black 
			{
				data "total_users" value: nb_people color: #black marker: false;
				data "total_AVs" value: nb_car color: #brown marker: false;
				data "users_served" value: int_were_passengers color: # green marker: true;
				data "AVs_used" value: gave_lift color: #blue marker: true;
				data "users_with_lift_late_at_work" value: were_late_with_lift color: #red marker: true;
			}
		}
	}	
}
