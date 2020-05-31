/***
* Name: people
* Author: Annalisa Congiu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model people

import "./../models/Parameters.gaml"
import "./../models/building.gaml"
import "./../models/intersection.gaml"

//species representing a simulation of people that are looking for a lift to {go to/come back from} work 
species people skills:[moving] control: fsm {
	rgb color <- #royalblue;
	building living_place <- nil ;
	building working_place <- nil ;
	float start_work ;
	float end_work ;
	point the_target <- nil ;
	float dist<-0.0 ;
	float dist_covered_alone ;
	intersection origin;
	
	bool starting <- true;
	bool late <- false;
	float actual_time_in;
	
	map road_knowledge <- graph_weights update: graph_weights;
	path path_to_follow <- nil;
	float look_up <- 200.0; // 200 meters the distance to identify intersections around the user
	string next_state <- nil;
	bool got_lift <- false; 
	list<intersection> close_intersections <- nil // also append intersection closest_to self.location
		update: (intersection overlapping (self.shape + look_up)) ; //set up a list for each user to the closest intersection
	
	
	float distance_to_cover;
	float time_to_cover<-0.0;
	float time_needed<-0.0;
	float cost_to_cover<-0.0;
	float cost_proposed<-0.0;
	float departure_time<-0.0;
	float arrival_time<-0.0;
	float waiting_time<-0.0 update: ((state contains "search_lift") and got_lift) ? (current_hour - waiting_time) : waiting_time;
	bool set_waiting_time<-true;
	bool late_at_work_with_lift; //computed only for those passengers that got a lift
	bool late_at_work_no_lift; //computed only for those passengers that do not got a lift
	float time_trip <- 0.0;
	float total_time_needed <- 0.0;
	
//	STATES OF THE passengers FSM
	
	state resting initial:true{
		enter{
			color <- #lightblue;
			next_state<-'search_lift_work';
		}
		transition to: search_lift_work when: current_hour >= start_work-before_work_search; 
	}
	
	state search_lift_work{
		enter{
			the_target <- working_place.location;
			color<- #yellow;
			late<-false;
			next_state<-'go_work';
		}
		transition to: go_work when: current_hour >= start_work - before_work_start; 
		transition to: go_work when: self.got_lift = true;
	}
	
	state wait_for_lift{ 
		if current_hour>start_work and current_hour<end_work{
			late<-true;
		}else{
			late<-false;
		}
		color<-#gamagreen;
		transition to: working when: self.location = working_place.location;
		transition to: resting when: self.location = living_place.location;
	}
	
	state go_work{
		enter{
			the_target<-working_place.location;
			if current_hour>start_work{
				late<-true;
			} else{
				late<-false;
			}
			color<-#thistle; //pink
			next_state<-'working';
		}
		transition to: working when: current_hour >= start_work and self.location = working_place.location;
	}
	
	state working{
		enter{
			actual_time_in<-time/3600;
			location <- working_place.location;
			if current_hour>start_work{
				late<-true;
			} else{
				late<-false;
			}
			color <- #gamaorange;
			if arrival_time>start_work and self.got_lift=true { 
				late_at_work_with_lift<-true;
			}
			if arrival_time>start_work and self.got_lift=false { 
				late_at_work_no_lift<-true;
			}
			next_state<-'search_lift_home';
		}
		transition to: search_lift_home when: current_hour > end_work; 
		exit{
			if (self.got_lift = true){
				self.got_lift <- false;
			} 
		}
	}
	
	state search_lift_home{
		enter{
			the_target <- living_place.location;
			color<- #yellow;	
			late<-false;
			next_state<-'go_home';
		}
		transition to: go_home when: current_hour >= end_work+after_work_start; 
	}
	state go_home{
		enter{
			the_target<-living_place.location;
			next_state<-'resting';  
			color<-#red;
		}
		transition to: resting when: self.location = living_place.location;		
	}
	
	reflex start_waiting_time when: (state contains "search_lift") and got_lift=false and set_waiting_time{
		if set_waiting_time{
			waiting_time <- current_hour with_precision 2;
			set_waiting_time<-false;
		}
	}
	
//	the agent computes the path to reach its destination
	reflex search_path when: the_target!=nil and path_to_follow=nil and ((state contains "search_lift") or 
		(state contains'go' and got_lift=true and location!=the_target)){
		if (path_to_follow = nil) {
			//Find the shortest path using the agent's own weights to compute the shortest path
			path_to_follow <- path_between(the_graph with_weights road_knowledge, location, the_target);
			if path_to_follow!=nil{
				list<geometry> segments <- path_to_follow.segments;
				loop seg over:segments{
					dist <- (dist + seg.perimeter);
					time_needed <- (time_needed + (seg.perimeter/(speed)));					
				}
			}
		}
	}
	
	//the agent follows the path it computed but with the real weights of the graph
	reflex move when: path_to_follow != nil and (state contains 'go_'){
	
		if starting{
			departure_time <- current_hour;
			starting<-false;
		}
		do follow path:path_to_follow speed: 2.0#m/#s move_weights: graph_weights; 
		if the_target = location {
			arrival_time <- current_hour with_precision 2;
			time_to_cover <- arrival_time-departure_time;
			dist_covered_alone<-dist_covered_alone + dist;
			dist<-0.0;
			the_target <- nil ;
			time_needed<-0.0;
			path_to_follow<-nil;
			starting<-true;
		}
	}
	
	aspect base {
		draw circle(25) color: color border: #black;
	}
}


