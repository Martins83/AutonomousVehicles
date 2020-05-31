/***
* Name: cars
* Author: Annalisa Congiu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model cars

import "./../models/Parameters.gaml"
import "./../models/road.gaml"
import "./../models/intersection.gaml"

//Car species (ensemble) that will move on the graph of roads to a target and using the skill advanced_driving
//parent of the species representing the autonomous vehicles
species cars skills: [advanced_driving]{ 

	rgb color <- rgb(rnd(255), rnd(255), rnd(255)) ; //random color for cars
	int counter_stucked <- 0; //the counter of stucked cars
	int threshold_stucked;
	bool breakdown <- false;
	float breakdown_probability ;
	intersection the_target;


	//it simulate the breakdown of a car with probability breakdown_probability
	reflex breakdown when: flip(breakdown_probability){
		breakdown <- true;
		max_speed <- 1 °km/°h;
	}
	// speed checking 
	reflex keep_in_check_real_speed{
		float rs_ma<-(real_speed + max_acceleration);
		float road_max;
		float confront_car_road ;
		if current_road!=nil{
			road_max <- (road(current_road).maxspeed * speed_coeff);
			confront_car_road <- (rs_ma < road_max ? rs_ma: road_max);
		}else{
			confront_car_road <- rs_ma;
		}
		real_speed <- (max_speed<confront_car_road) ? max_speed : confront_car_road;
	}
	
	reflex time_to_go when: final_target = nil {
		the_target <- one_of(intersection where not each.is_traffic_signal);
		current_path <- compute_path(graph: road_network, target: the_target); 
		if (current_path = nil ) {
			final_target <- nil;
		} 
	}
	
	reflex move when: current_path != nil and final_target != nil {
		do drive;
		if real_speed < 5°km/°h {
			counter_stucked <- counter_stucked + 1;
			if (counter_stucked mod threshold_stucked = 0) {
				proba_use_linked_road <- min([1.0,proba_use_linked_road + 0.1]);
			}
		} else {
			counter_stucked <- 0;
			proba_use_linked_road <- 0.0;
		}
	}
	
	aspect base { 
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{5#m,2.5#m} rotate: heading+180;
		draw breakdown ? square(15) : triangle(15) color: color rotate:heading + 90;
	} 
	aspect base3D {
		point loc <- calcul_loc();
		draw box(vehicle_length, 1,1) at: loc rotate:  heading color: color;
		
		draw triangle(0.5) depth: 1.5 at: loc rotate:  heading + 90 color: color;
		
		if (breakdown) {
			draw circle(2) at: loc color: #red;
		}
	} 
	
	point calcul_loc {
		if (current_road = nil) {
			return location;
		} else {
			float val <- (road(current_road).lanes - current_lane) + 0.5;
			val <- on_linked_road ? val * - 1 : val;
			if (val = 0) {
				return location; 
			} else {
				return (location + {cos(heading + 90) * val, sin(heading + 90) * val});
			}
		}
	}
} 


