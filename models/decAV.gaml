/**
* Name: decAV
* Author: Annalisa Congiu
* Description: species representing AV in the decentralised simulation
* Tags: 
*/

model decAV

import "./../models/building.gaml"
import "./../models/road.gaml"
import "./../models/intersection.gaml"
import "./../models/cars.gaml"
import "./../models/people.gaml"
import "./../models/Parameters.gaml"

/* Species representing Autonomous Vehicles in the decentralised simulation. */

//Car species (ensemble) that will move on the graph of roads to a target and using the skill advanced_driving
species decAV skills: [advanced_driving] control:fsm parent:cars{ 
	list<float> waiting_times;
	list<float> final_costs;
	
	int n_travel<-0;
	
	int total_passengers_travel<-0;
	int total_stops_travel<-0;
	int n_addition<-0;
	int cumulative_lifts_given <- 0;
	
	float stats_first_path_time<-0.0;

	float time_needed;
	float dist <-0.0;
	float dist_covered_cars<-0.0;
	
	path eventual_path;
	map<intersection, list<people>> people_destinations;
	list<people> first_group;
	map<intersection, list<people>> give_lift_to;
	map<intersection, list<string>> people_destinations_for_dropping;
	map<list<intersection>, list<float>> cost_legs;
	list<intersection> ordered_or_dest;
	list<intersection> int_targets <- nil;
	bool added_last;
	int added_people;
	int total_added_people;
	int origin_index;
	int index_dest;
	bool origin_exists<-false;
	bool update_costs_for_passengers<-false;
	bool gave_lift<-false;
	
	list<float> costs_passengers<-nil; // list of costs for passengers
	float mean_of_costs<-0.0;
	float max_of_costs<-0.0;
	float min_of_costs<-0.0;
	
	int max_passengers<-max_passengers_per_AV; 
	list<intersection> current_road_nodes;
	bool update<-true;
	float arrived_time<-0.0;
	float starting_time<-0.0;
	
	people first_p;
	list<people> possible_pass;
	intersection origin;
	
//	create a list of people that are close to the car and keep the list update
	list<people> people_near <- 
				(length(passenger) < max_passengers and current_road_nodes!=nil and state!='first_stop' and update) 
				? (people where ((each.state contains 'search_lift') and each.close_intersections contains_any self.current_road_nodes)) : nil
				update: (length(passenger) < max_passengers and current_road_nodes!=nil and state!='first_stop' and update) 
				? (people where ((each.state contains 'search_lift') and each.close_intersections contains_any self.current_road_nodes)) : nil; 
		
	state wander initial: true{
		enter{
			first_p<-nil;
			first_group<-nil;
			possible_pass<-nil;
			people_near<-nil;
			people_destinations<-nil;
			people_destinations_for_dropping<-nil;
			costs_passengers<-nil;
			cost_legs<-nil;
			int_targets<-nil;
			ordered_or_dest<-nil;
			mean_of_costs<-0.0;
			min_of_costs<-0.0;
			max_of_costs<-0.0;
			total_passengers_travel <- 0;
			total_stops_travel <- 0;
			waiting_times <- nil;
			final_costs <- nil;
		}
		}

	
	state first_stop{
		transition to: wander when: possible_pass = nil;
	}
	
	state moving{} 
	state stop{}
	
	
	//	reflex to update the road nodes 
	reflex update_road_nodes when: current_road!=nil and state!='first_stop'{
		if current_road_nodes contains intersection(road(current_road).target_node){
			update<-false;
			
		}else{
			current_road_nodes<-nil;
			add intersection(road(current_road).target_node) to:current_road_nodes;
			add intersection(road(current_road).source_node) to:current_road_nodes;	
			update<-true;
		}
	}
	
//	reflex to update the path to follow when the car is wandering
	reflex time_to_go when: final_target = nil{
		if state = 'wander'{
			the_target <- one_of(intersection where not each.is_traffic_signal);
			current_path <- compute_path(graph: road_network, target: the_target );
			if (current_path = nil) {
				final_target <- nil;
			}else{
				starting_time<- time;
			}	
		}
	}
	
//	reflex to choose a possible first passenger
	reflex choose_first_passenger when: !empty(people_near) and state='wander'{
		stats_first_path_time<- machine_time;
		state<-'first_stop';
		origin <- current_road_nodes[1];  //the current position of the car
		
		possible_pass<- people_near;
		people_near<-nil;
		if origin != nil {
			possible_pass >>- possible_pass select (each.the_target = nil);
			possible_pass <- possible_pass sort_by (each.the_target distance_to origin);
		}
		
		do identify_first_passenger;
	}


//	change the first passenger if it has already been taken by another car
	reflex change_first_passenger when: first_p=nil and state='first_stop'{
		origin <- current_road_nodes[1];  //the current position of the car
		do identify_first_passenger;
	}
	
	action identify_first_passenger {
		eventual_path<-nil;
		intersection first_p_target_inter;
		
		if eventual_path=nil{
			loop while: eventual_path=nil and !empty(possible_pass){
				first_p <- last(possible_pass);
				first_p_target_inter <-(intersection closest_to first_p.the_target); //look for the closest intersection to the passenger's target
				if (origin != nil and first_p_target_inter != nil){
					eventual_path<- road_network path_between (origin,first_p_target_inter); // path chosen for first passenger -> chose general path
				}
				if eventual_path != nil and eventual_path.edges=[]{
					first_p<-nil;
				}
				remove first_p from: possible_pass;
			}
			if eventual_path = nil{
				self.location<- current_road_nodes[1].location;
				possible_pass<-nil;
				people_near<-nil;
				self.state<- 'wander';
			}
		}	
	}
	
	//	notify other cars that the passenger has been taken
	reflex check_other_cars_first_p when: first_p!=nil and state='first_stop'{
		ask (decAV-self) where (each.state='first_stop' and each.first_p=first_p){
			if self.possible_pass=myself.possible_pass{
				remove self.first_p from: self.possible_pass;
				if empty(self.possible_pass){
					people_destinations_for_dropping<-nil;
					costs_passengers<-nil;
					mean_of_costs<-0.0;
					min_of_costs<-0.0;
					max_of_costs<-0.0;
					self.state<-'wander';
					write(self.state + " " + myself.state);
				}
			}
		}
	}
	
	
//	remove passengers that want to go towards a place in a complete different direction
	action remove_due_to_direction{
		list removed_p;
		loop p over: (possible_pass){
			if (p.the_target != nil) {
				if !dead(p) {
					if angle_between(last(int_targets).location, first(int_targets).location, p.the_target)>20.0{ 
												// 1st passenger destination, 1st passenger origin, p destination
						add p to: removed_p;
					} else {
						intersection cl_int <- (intersection closest_to p.the_target);
						if (people_destinations.keys contains cl_int){
							add p to:people_destinations[cl_int];
						} else {
							add cl_int::[p] to: people_destinations;
						}
					}
				} else{
					add p to: removed_p;
				}
			}	
		}
		if !empty(removed_p){
			remove all: removed_p from: possible_pass;
		}	
		// check in order to avoid to compute the current path again for other passengers that go to the same location;		
		if people_destinations.keys contains last(int_targets){
			first_group<-nil;
			add first_p to:first_group;
			add all:people_destinations[last(int_targets)] to: first_group;
			remove all:people_destinations[last(int_targets)] from: possible_pass; // are already going to be captured;
			remove key: last(int_targets) from: people_destinations;
		}
	}
	
//	compute first path
	reflex create_path when: !empty(possible_pass+first_p) and state='first_stop' and first_p!=nil{ 
		int total_available_people <- length(possible_pass);
	    origin<- current_road_nodes[1];
//		intersection origin<- current_road_nodes[1];
		first_group <- [first_p];
		
		intersection first_p_target_inter <- (intersection closest_to first_p.the_target);
		
		eventual_path <- road_network path_between (origin,first_p_target_inter);
		float time_for_eventual_path<-0.0;
		loop e over: (eventual_path.edges){
			time_for_eventual_path <- time_for_eventual_path + (road(e).shape.perimeter / road(e).maxspeed);
		}
		
		add origin to: int_targets; // formally add starting point
		add first_p_target_inter to: int_targets; // formally add ending point
		
		do remove_due_to_direction;

		int dest_length<- length(people_destinations.keys);
		if !empty(possible_pass){
			int i<-0;
			float length_ev_path;
			
			if length(people_destinations.keys)>0{
				length_ev_path <- eventual_path.shape.perimeter;
				
				loop dest over: (people_destinations.keys){
					path deviation1 <- nil;
					path deviation2 <- nil;
					float length_dev;
					
					deviation1 <- road_network path_between (int_targets[i], dest) ;
					deviation2 <- road_network path_between (dest, last(int_targets));
					
					if (deviation1.edges!=[]) and (deviation2.edges!=[]){
						length_dev <- deviation1.shape.perimeter + deviation2.shape.perimeter ;
					}else{
						length_dev <- length_ev_path + (length_ev_path/3) ;
					}
					
					if length_dev < (length_ev_path) {
						int at_i<- length(int_targets)-1;
						add dest at:at_i to: int_targets;
						length_ev_path <-0.0;
						int j<-0;
						loop inters over:(int_targets-first(int_targets)){
							path current_leg<-road_network path_between(int_targets[j], inters);						 
							length_ev_path <- length_ev_path+((current_leg).shape.perimeter) ;
							j<-j+1;
						}
						i<-i+1;
					} else{
						remove all:people_destinations[dest] from: possible_pass;
						remove key:dest from: people_destinations;
					}
				}
			}
		}
		
		add all:first_group to: possible_pass;		
		add origin::possible_pass to: give_lift_to;
		add first_p_target_inter::first_group to:people_destinations;
		
		
		//di seguito calcola i costi del primo gruppo
		//int total_added_people ;
		if !empty(possible_pass){
			people_destinations_for_dropping<-nil;			
			costs_passengers<-nil;
			int i <- 0;
			
			float sum_time_leg <- 0.0;
			float sum_distance_leg <- 0.0;
			float sum_cost_leg <- 0.0;
			int leg_passengers <- length(possible_pass);
			
			loop dest over: (int_targets-first(int_targets)){
				float time_leg <- 0.0;
				float cost_leg <- 0.0;
				
				path leg_path <- road_network path_between (int_targets[i], dest);
					
				loop e over: list<road>(leg_path.edges){
					time_leg <- time_leg + (e.shape.perimeter / e.maxspeed);
				}
	
				cost_leg <- ((leg_path.shape.perimeter/1000) * cost_km);
				sum_cost_leg <- sum_cost_leg + (cost_leg/leg_passengers); 
				sum_time_leg <- sum_time_leg + time_leg;
				add [(int_targets[i]), dest]::[cost_leg,leg_passengers, time_leg]  to: cost_legs;
				
				list<string> names<- nil;	
				loop p over: people_destinations[dest]{
					ask p{
						origin<- myself.current_road_nodes[1];
						got_lift<-true;
						cost_proposed<-sum_cost_leg;
					}
					add p.name to: names;
				}
				
				add dest::names to:people_destinations_for_dropping;
				
				loop times:length(names){
					add sum_cost_leg to: costs_passengers;
				}
				
				leg_passengers<- leg_passengers - length(people_destinations[dest]);
				i<-i+1;
			}
			add all:int_targets to: ordered_or_dest;
			
			mean_of_costs <- mean(costs_passengers);
			min_of_costs<- min(costs_passengers);
			max_of_costs<- max(costs_passengers); 
			ask decAV-self{
				remove all: possible_pass from: self.possible_pass;
			}
			total_added_people <- length(possible_pass);
			add all: (possible_pass collect each.waiting_time) to: waiting_times;
			capture possible_pass as:passenger{} 
			
			self.location<- origin.location;
			state<-'moving';
			first_p <- nil;
			possible_pass<-nil;
			final_target <- nil;
			the_target <- nil;
			current_path <- nil;
			people_near<-nil;
			gave_lift<-true;
		} else{
			possible_pass<-nil;
			people_near<-nil;
			self.state<- 'wander';
		}
		stats_first_path_time <- machine_time - stats_first_path_time;
		total_passengers_travel <- total_passengers_travel + total_added_people;
	}

//	compute a path when there are passengers on	
	reflex get_path_moving when: final_target = nil and state='moving'{
		loop while: the_target=nil{
			the_target<- first(int_targets-first(int_targets));
		}
		
		current_path <- compute_path(graph: road_network, target: the_target );
		
		if current_path!=nil{
			list<road> roads_in_path <- list<road>(current_path.edges);
			loop r over:roads_in_path{
				dist <- dist+r.shape.perimeter;
				time_needed<- (time_needed + (r.shape.perimeter/r.maxspeed));
			}

			starting_time<- time;
		}
	}
	
//	move when there is a path, both when wandering and with passengers on
	reflex move when: current_path != nil and final_target != nil{
		if (state='wander' or state='moving'){
			do drive;
			if real_speed < 5°km/°h {
				counter_stucked<- counter_stucked + 1;
				if (counter_stucked mod threshold_stucked = 0) {
					proba_use_linked_road <- min([1.0,proba_use_linked_road + 0.1]);
				}
			} else {
				counter_stucked<- 0;
				proba_use_linked_road <- 0.0;
			}
			
			if (self.location = the_target.location){
				arrived_time<-time;
				final_target<-nil;
				dist_covered_cars<-dist_covered_cars + dist;
				dist<-0.0;
				time_needed<-0.0;
				if state='moving'{
					state<-'stop';
				}
			}		
		}
	}
	
//	update the costs for the passengers: the new passengers get to know the costs for the trip
//	and already on board passengers get to know the updated price
	reflex update_costs_passengers when: update_costs_for_passengers=true {
		list<float> costs_for_on <- list(passenger) collect each.cost_proposed;
		list<intersection> origins_p <- list(passenger) collect each.origin;
		
		float prev_cost<-0.0;
		loop p over: list(passenger){
			bool next_too<-false;
			float cost<-0.0;
			loop key over: cost_legs.keys{
				if key[0]=p.origin{
					next_too<-true;
					cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
				}
				if next_too{
					cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
					if key[1]=(intersection closest_to p.the_target){break;}
				}
			}
			prev_cost <- p.cost_proposed;
			p.cost_proposed <- cost; 
		}
		
		costs_for_on <- list(passenger) collect each.cost_proposed;
		update_costs_for_passengers<-false;
	}
	
//	car recomputation of costs with the addition of passengers
	action update_costs{
		int difference_in_dest_or <- (index_dest - origin_index)-1;
		int stop_at;
		
		int i<-0;
		bool take_existing<-false;
		map<list<intersection>, list<float>> tmp <- cost_legs;
		cost_legs<-nil;
		
		loop key over: tmp.keys{
			if i<origin_index-1 or (origin_exists and i <origin_index){
				add key::tmp[key] to: cost_legs;
			}
			if  ((origin_exists and i=origin_index) or (origin_index=0 and i=0)){
				if difference_in_dest_or = 0 and key[1] != ordered_or_dest[index_dest]{
					path leg2 <- road_network path_between (ordered_or_dest[origin_index], ordered_or_dest[index_dest]);
					float time2<-0.0;
					loop e over: list<road>(leg2.edges){
						time2<- time2 + (e.shape.perimeter/e.maxspeed);
					}
					float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
					add [ordered_or_dest[origin_index], ordered_or_dest[index_dest]]::[cost2, int(tmp[key][1] + added_people), time2] to: cost_legs;
					
					path leg3 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
					float time3<-0.0;
					loop e over: list<road>(leg3.edges){
						time3<- time3 + (e.shape.perimeter/e.maxspeed);
					}
					float cost3 <- ((leg3.shape.perimeter/1000)*cost_km) ;
					add [ordered_or_dest[index_dest], key[1]]::[cost3, int(tmp[key][1]), time3] to: cost_legs;
				} else if difference_in_dest_or > 0{
					tmp[key][1]<- int(tmp[key][1] + added_people); 
					add key::tmp[key] to: cost_legs;
				}
			}
			if ( !origin_exists and origin_index>0 and i=origin_index-1) {
					path leg1 <- road_network path_between (key[0], ordered_or_dest[origin_index]);
					float time1<-0.0;
					loop e over: list<road>(leg1.edges){
						time1<- time1 + (e.shape.perimeter/e.maxspeed);
					}
					float cost1 <- ((leg1.shape.perimeter/1000)*cost_km);
					add [key[0], ordered_or_dest[origin_index]]::[cost1, int(tmp[key][1]), time1] to: cost_legs;
				if difference_in_dest_or = 0 and key[1] != ordered_or_dest[index_dest]{
					path leg2 <- road_network path_between (ordered_or_dest[origin_index], ordered_or_dest[index_dest]);
					float time2<-0.0;
					loop e over: list<road>(leg2.edges){
						time2<- time2 + (e.shape.perimeter/e.maxspeed);
					}
					float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
					add [ordered_or_dest[origin_index], ordered_or_dest[index_dest]]::[cost2, int(tmp[key][1] + added_people), time2] to: cost_legs;
					
					path leg3 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
					float time3<-0.0;
					loop e over: list<road>(leg3.edges){
						time3<- time3 + (e.shape.perimeter/e.maxspeed);
					}
					float cost3 <- ((leg3.shape.perimeter/1000)*cost_km) ;
					add [ordered_or_dest[index_dest], key[1]]::[cost3, int(tmp[key][1]), time3] to: cost_legs;
				}else if difference_in_dest_or > 0 or (difference_in_dest_or = 0 and key[1]= ordered_or_dest[index_dest] ){ 
					path leg2 <- road_network path_between (ordered_or_dest[origin_index], key[1]);
					float time2<-0.0;
					loop e over: list<road>(leg2.edges){
						time2<- time2 + (e.shape.perimeter/e.maxspeed);
					}
					if leg2.shape.perimeter != nil {
					float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
					add [ordered_or_dest[origin_index], key[1]]::[cost2, int(tmp[key][1]+added_people), time2] to: cost_legs;
					}
				}
			}
			if !added_last and ((origin_exists and i>origin_index and i<index_dest-1  and difference_in_dest_or>0)
				or (!origin_exists and i>=origin_index and i<index_dest-2 and origin_index>0 )){
				add key::[tmp[key][0], int(tmp[key][1]+ added_people), tmp[key][2]] to: cost_legs;
			}
			if !added_last and ((origin_exists and i = index_dest-1 and i>=origin_index and difference_in_dest_or>0)
				or (!origin_exists and i = index_dest-2 and i>=origin_index and origin_index>0)){
					if key[1]= ordered_or_dest[index_dest]{
						add key::[tmp[key][0], int(tmp[key][1]+ added_people), tmp[key][2]] to: cost_legs;
					} else{
						path leg1 <- road_network path_between (key[0], ordered_or_dest[index_dest]);
						float time1<-0.0;
						loop e over: list<road>(leg1.edges){
							time1<- time1 + (e.shape.perimeter/e.maxspeed);
						}
						float cost1 <- ((leg1.shape.perimeter/1000)*cost_km);
						add [key[0], ordered_or_dest[index_dest]]::[cost1, int(tmp[key][1]+added_people), time1] to: cost_legs;
						
						path leg2 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
						float time2<-0.0;
						loop e over: list<road>(leg2.edges){
							time2<- time2 + (e.shape.perimeter/e.maxspeed);
						}
						float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
						add [ordered_or_dest[index_dest], key[1]]::[cost2, int(tmp[key][1]), time2] to: cost_legs;
					}
			} 
			if !added_last and (
				(origin_exists and i >= index_dest and difference_in_dest_or = 0)
				or (!origin_exists and i >= index_dest-1) ){
				add key::tmp[key] to: cost_legs;
			}
			if added_last and ((i>=origin_index and !origin_exists) or i>origin_index){
				add key::[tmp[key][0], int(tmp[key][1]+ added_people), tmp[key][2]] to: cost_legs;
			}
			
			i<-i+1;
			
		}
		if added_last{
			intersection key0 <- ordered_or_dest[length(ordered_or_dest)-2];
			intersection key1 <- last(ordered_or_dest);
			path leg1 <- road_network path_between (key0, key1);
			float time1<-0.0;
			loop e over: list<road>(leg1.edges){
				time1<- time1 + (e.shape.perimeter/e.maxspeed);
			}
			float cost1 <- ((leg1.shape.perimeter/1000)*cost_km);
			add [key0, key1]::[cost1, int(added_people), time1] to: cost_legs;
			added_last<-false;
		}
	}

//	look for new passengers on the road and check if the constraints are respected in order to actually pick them up
	reflex add_on_road when: !empty(people_near) and (state='moving') and length(passenger)< max_passengers{
		stats_first_path_time <- machine_time;
		state<-'first_stop';
		possible_pass<-people_near;
		do remove_due_to_direction;
		int total_available_people<-length(possible_pass);
		int dest_length <- 0;
		int initial_stops <-0;
		if !empty(possible_pass){
			list<people> to_remove;
			loop p over: possible_pass {
				if (closest_to(intersection, p.the_target) = nil) {
					add p to: to_remove;
				}
			}
			remove all: to_remove from: possible_pass;
			map<intersection, list<people>> people_on_road <- group_by(possible_pass, (closest_to(intersection, each.the_target)));
			loop k over: people_on_road.keys {
				if (k = nil) {
					remove key:k from: people_on_road;
				}
			}
			
			int inted <- ((ordered_or_dest index_of the_target));
			list<intersection> going_back<- copy_between(ordered_or_dest, 0, inted);
			list<intersection> common_back <- people_on_road.keys inter going_back;
			add current_road_nodes[1] to: common_back;
			
			if !empty(common_back){
				loop k over: common_back{
					remove key:k from: people_on_road;
				}
			}
			
			common_back<-nil;
			
			if !empty(people_on_road){
				initial_stops <- length(ordered_or_dest);
				
				origin_exists<-false;
				origin_index<- inted-1;	
				
				if ordered_or_dest[inted-1]!=current_road_nodes[1]{
					add current_road_nodes[1] at:inted to: ordered_or_dest;
					origin_index <- inted;
				} else{
					origin_exists<-true;
				}
				
				passenger first_still_on <-  first(list(passenger));
				float first_remaining_time <-  (first_still_on.start_work - current_hour)* 3600;
				intersection first_remaining_target;
				float start_to_first <- 0.0;
				
				loop dest over: people_destinations_for_dropping.keys{
					if people_destinations_for_dropping[dest] contains first_still_on.name{
						first_remaining_target<-dest;
						break;
					}
				}
				
				int j <- 0;
				int n <- (int_targets index_of first_remaining_target);

				float normal_time<-0.0;
				loop times: n{
					path leg;
					if j = 0{
						leg <- (road_network path_between (location, int_targets[j+1]));
					} else{
						leg<- (road_network path_between (int_targets[j], int_targets[j+1]));
					}
					start_to_first <- start_to_first + (leg.shape.perimeter / 8.3) ;
					loop e over: leg.edges{
						normal_time<- normal_time + (road(e).shape.perimeter / road(e).maxspeed);
					}
					j <- j+1;
				}
				
				
				int remaining_places <- max_passengers - length(list(passenger));
				dest_length<- length(people_on_road.keys);
				loop dest over: people_on_road.keys{
					path target_to_dest ;
					float time_t2dest<-0.0;
					path target_to_next;
					float time_t2next<-0.0;
					float time_dest2next<-0.0;
					bool added<- false;
					int i<-1;
					int index_max <- length(int_targets)-1;
					added_people<-0;
					
					if remaining_places=0{
						break;
					}
					if remaining_places>0{
						list<people> toadd <- (people_on_road[dest]);
						if length(toadd) > remaining_places{
							toadd <- people_on_road[dest] copy_between(0, remaining_places-1);
						}
						if length(toadd) = 0{
							break;
						} 
						
						if int_targets contains dest{
							index_dest<- ordered_or_dest index_of dest;
								
							if people_destinations_for_dropping.keys contains dest{ 
								add all:list<string>(toadd collect each.name) to: people_destinations_for_dropping[dest];
							} 
							added_people<- length(toadd)+ added_people; 
							remaining_places <- remaining_places - length(toadd);
							total_added_people  <- total_added_people + added_people;
							
							do update_costs;
							origin_exists<-true;
							bool next_too<-false;
							
							ask toadd{
								got_lift<-true;
								self.origin <- myself.current_road_nodes[1];
							}
		
							add all: (toadd collect each.waiting_time) to: waiting_times;
							capture toadd as: passenger{
							}
							added<-true;
							update_costs_for_passengers<-true;					
						} 
						else{				
							loop while: added=false{
								if i=1 { //and dest != nil
									if (location = nil) {
										write("LOCATION NIL");
									}
									if (dest = nil) {
										write("DEST NIL");
									}
									target_to_dest <- road_network path_between(location, dest);
									target_to_next <- road_network path_between(location, int_targets[i]);
									time_t2dest <- (target_to_dest.edges=[])? 0 : target_to_dest.shape.perimeter /8.3;
									time_t2next <- target_to_next.shape.perimeter /8.3; 
								}else if i< index_max { 
									target_to_dest <- road_network path_between(int_targets[i-1], dest);
									target_to_next <- road_network path_between(int_targets[i-1], int_targets[i]);
									time_t2dest <- (target_to_dest.edges=[])? 0 : target_to_dest.shape.perimeter /8.3;
									time_t2next <- target_to_next.shape.perimeter /8.3; 
								}
								//target_to_dest != nil
								if i<=index_max and target_to_dest.shape != nil and target_to_next.shape != nil{
									if target_to_dest.shape.perimeter < target_to_next.shape.perimeter{
										path dest_to_target <- road_network path_between (dest, int_targets[i]);
										if dest_to_target.edges!=[]{
											time_dest2next <- dest_to_target.shape.perimeter /8.3;
											float with_change <- (start_to_first - time_t2next) + (time_t2dest + time_dest2next);
											
											if with_change < (start_to_first + (start_to_first/2)) and first_remaining_time > (with_change*8.3/13.8){
												if !((int_targets-int_targets[0]) contains dest){
													index_dest<- ordered_or_dest index_of int_targets[i]; 
													add dest at:(index_dest) to: ordered_or_dest; 
													add dest at:i to: int_targets;
												}
												if people_destinations_for_dropping contains dest{
													add all:list<string>(toadd collect each.name) to: people_destinations_for_dropping[dest];
												}else{
													add dest::list<string>(toadd collect each.name) to:people_destinations_for_dropping;
												} 
												added_people<- length(toadd)+ added_people;
												total_added_people  <- total_added_people + added_people;
												remaining_places <- remaining_places - length(toadd);
												
												do update_costs;
												origin_exists<-true;
												
												ask toadd{
													got_lift<-true;
													self.origin <- myself.current_road_nodes[1];
												}
												add all: (toadd collect each.waiting_time) to: waiting_times;
												capture toadd as: passenger{}						
												
												if i=1{
													the_target<-nil;
													final_target<-nil;
													current_target<-nil;	
												}
												added<-true;
												update_costs_for_passengers<-true;
											} else {
												added<-true;
											}
										}
									}
								}
								if i=index_max and !added{
									target_to_dest <- road_network path_between(last(int_targets), dest);
									if target_to_dest.edges!=[]{
										if !((int_targets-int_targets[0]) contains dest){
											index_dest<- length(ordered_or_dest);
											add dest to: int_targets;
											add dest to: ordered_or_dest;
										}
										if people_destinations_for_dropping contains dest{
											add all:list<string>(toadd collect each.name) to: people_destinations_for_dropping[dest];
										}else{
											add dest::list<string>(toadd collect each.name) to:people_destinations_for_dropping;
										} 
										added_people<- length(toadd)+ added_people;
										total_added_people  <- total_added_people + added_people;
										remaining_places <- remaining_places - length(toadd);
										added_last<-true;
										
										do update_costs;
										origin_exists<-true;
																			
										ask toadd{
											got_lift<-true;
											self.origin <- myself.current_road_nodes[1];
										}
										add all: (toadd collect each.waiting_time) to: waiting_times;
										
										capture toadd as: passenger{}
										added<-true;
										update_costs_for_passengers<-true;
									}
									break;
								}
//								}
								i<-i+1;	
							} // end while
							remove all:people_on_road[dest] from: possible_pass;
						}
					}
				}
				if total_added_people=0{ 
					if (ordered_or_dest index_of current_road_nodes[1]) !=0{
						remove current_road_nodes[1] from: ordered_or_dest;
					}
				} else{
					write self.name + " ("+ total_added_people+ ") updated costs "+ cost_legs;
				}
				stats_first_path_time <- machine_time - stats_first_path_time;
				n_addition <- n_addition + 1;
			}
		}
		state<-'moving';
		people_near<-nil;
		possible_pass<-nil;
		self.location<- current_road_nodes[1].location;
		total_passengers_travel <- total_passengers_travel + total_added_people; 
	}

//	reflex to drop the passengers that have reached their destination
	reflex drop_people when:!empty(passenger) and state='stop'{
		list<string>names;
		list<string> states_dropped;
		list<people>dropped;
		string substitute_state;

		loop p over: (passenger){
			if people_destinations_for_dropping[the_target] contains p.name and self.location=the_target.location{
				add p.cost_proposed to: final_costs; 
				add p to: dropped;
				cumulative_lifts_given <- cumulative_lifts_given + 1;
				add p.name to: names;
				if p.the_target!=location{
					if (p.next_state contains 'work') {
						substitute_state<-'go_work';
					}
					if (p.next_state='resting' or p.next_state='go_home') {
						substitute_state<-'go_home';
					}
				}else{
					substitute_state<-p.next_state;
				}
				add substitute_state to: states_dropped;
				ask p{
					state<- substitute_state;
					location<-myself.location;
					path_to_follow<-nil;
				}
				release p in:world as:people{}
			}
		}
//		if !empty(dropped){
//			//write string(self.name) +' ('+current_hour+') dropped '+(names) + ' with states: '+ states_dropped; //+' @ '+ location +' with: '+p_targets+ ' - ' +state;
//		}
		
		remove the_target from: int_targets;
		remove key:the_target from: people_destinations_for_dropping;
		the_target<-nil;
		
		if empty(passenger){
			n_travel<-n_travel+1;
			self.state<- 'wander';
		} else{state<-'moving';}
	}
	
	species passenger parent: people{
		aspect default{
		}
	}

	aspect base {
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{5#m,2.5#m} rotate: heading+180;
		draw breakdown ? square(15) : triangle(30) color: color rotate:heading + 90;
	} 
	aspect base3D {
		point loc <- calcul_loc();
		draw box(vehicle_length, 1,1) at: loc rotate:  heading color: color;
		
		draw triangle(0.5) depth: 1.5 at: loc rotate:  heading + 90 color: color;
		
		if (breakdown) {
			draw circle(2) at: loc color: #red;
		}
	} 
} 


