/***
* Name: autonomousCars
* Author: Annalisa Congiu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model autonomousCars 
global{
	//map used to filter the object to build from the OSM file according to attributes. for an exhaustive list, see: http://wiki.openstreetmap.org/wiki/Map_Features
	map filtering <- map(["highway"::["primary", "secondary", "tertiary", "motorway", "residential", "unclassified", "road"], 
		"building"::[]
	]);
	
	//OSM file to load
	file<geometry> osmfile <-  file<geometry>(osm_file("../includes/map.osm", filtering))  ;
	
	geometry shape <- envelope(osmfile);
	map<point, intersection> nodes_map;
	graph road_network; 

	//Manage the speed allowed in the model for the people agents
	float stop_speed <- 0.0   #km / #h;
	float min_speed <- 20.0  #km / #h;
	float max_speed <- 30.0 #km / #h; 	
	
	init {
		write "OSM file loaded: " + length(osmfile) + " geometries";
		
		//from the OSM file, creation of the selected agents
		loop geom over: osmfile {
			string building_str <- string(geom get ("building"));
			
			if (shape covers geom) {				
				string highway_str <- string(geom get ("highway"));	
				if (length(geom.points) = 1) {
					if ( highway_str != nil  and building_str=nil ) {
						string crossing <- string(geom get ("crossing"));
						if highway_str='traffic_signals'{
							crossing<- 'traffic_signals';
						}
						create intersection with: [shape ::geom, type:: highway_str, crossing::crossing] {
							nodes_map[location] <- self;
						}
					}
				} else 	
				if (highway_str !=nil  and building_str=nil){
					string junction <- string(geom get("junction"));
					
					string oneway <- string(geom get ("oneway"));
					if oneway=nil{
						oneway<-'no';
					}
					float maxspeed_val <- float(geom get ("maxspeed"));
					if maxspeed_val = 0 {maxspeed_val <- 50.0;}
					string lanes_str <- string(geom get ("lanes"));
					int lanes_val <- empty(lanes_str) ? 1 : ((length(lanes_str) > 1) ? int(first(lanes_str)) : int(lanes_str));
					if lanes_val<1 {
						lanes_val <- 1;
					}
					create road with: [shape ::(geom), type::highway_str, oneway::oneway, maxspeed::maxspeed_val, lanes::lanes_val, junction::junction];			
				}
			}
		}


		ask road[1739]{ write "die "+ self.name ; do die; }
		ask road[1457]{ write "die "+ self.name ; do die; }
		ask road[1456]{ write "die "+ self.name ; do die; }
		ask road[1455]{ write "die "+ self.name ; do die; }
		ask road[1422]{ write "die "+ self.name ; do die; }
		ask road[1408]{ write "die "+ self.name ; do die; }
		ask road[1407]{ write "die "+ self.name ; do die; }
		ask road[1105]{ write "die "+ self.name ; do die; }
		ask road[1074]{ write "die "+ self.name ; do die; }
		ask road[1005]{ write "die "+ self.name ; do die; }



		write "Road and node agents created";
		
//		create intersection agent at end/beginning of road when one does not already exist
		ask road {
			point ptF <- first(shape.points);
			if (not(ptF in nodes_map.keys)) {
				create intersection with:[location::ptF] {
					nodes_map[location] <- self;
				}	
			}
			point ptL <- last(shape.points);
			if (not(ptL in nodes_map.keys)) {
				create intersection with:[location::ptL] {
					nodes_map[location] <- self;
				}
			}
		}
		
		write "Supplementary node agents created";
		
//		kill intersection agents that do not overlap any road
		ask intersection {
			if (empty (road overlapping (self))) {
				do die;
			}
		}
		ask intersection where (each.crossing='island'){
			do die;
		}
		write "node agents filtered";
		
//		create building species agents from the data in the OSM file
		create osm_agent from:osmfile with: [building_str::string(read("building"))];
		write "Building agent created";
		
//		on the base of the string characterising the building the agent 
//		is assigned to either the residential or the industrial cathegory  
		ask osm_agent {
			if (length(shape.points) = 1) {}
			if(length(shape.points)>1){
				if(building_str = "residential" or building_str ="apartments" or building_str = "cabin" or building_str = "dormitory" or 
					building_str = "house" or building_str = "hut" or building_str = "detached" or building_str = "casa_di_riposo" or building_str = "convent" ){
					create building with: [shape ::shape, type::building_str]{
						residential<-true;
						group<- 'residential';
						color <- #lightblue;
					}
				} 
				else 
				if(building_str ="bunker" or building_str ="cathedral"
				or building_str ="chapel" or building_str ="church" or building_str ="civic" or building_str ="commercial"
				or building_str ="government" or building_str ="greenhouse" or building_str ="gym" or building_str ="hangar"
				or building_str ="hospital" or building_str ="industrial" or building_str ="kindergarten" or building_str ="kiosk"
				or building_str ="manufacture" or building_str ="office" or building_str ="retail" or building_str ="school"
				or building_str ="train_station" or building_str ="university")
				{
					create building with:[shape::shape, type::building_str]{
						industrial <-true;
						group<-'industrial';
						color<-#blue;
					}
				}else if(building_str ="FBK_Fondazione_Bruno_Kessler"){
					create building with:[shape::shape, type::building_str]{
						industrial <-true;
						group<-'industrial';
						color<-#red;
					}
				}else 
				if (
					building_str = "unclassifed" or building_str = "undefined" 
							or building_str = "0" or building_str = "1" or building_str = "2" or building_str = "3"  
							or building_str = "4" or building_str = "5" or building_str = "6" or building_str = "7" 
							or building_str = "8" or building_str = "9" or building_str = "10" or building_str = "13" 
							or building_str = "14" or building_str = "40"
							){
					create building with:[shape::shape, type::building_str]{
						industrial <- flip(0.4);
						if (industrial = true) {
							group<-'industrial';
							color <- #blue;
						} else{
							residential <-true;
							group<- 'residential';
							color <- #lightblue;
						}
					}
				} else if (building_str != nil){
					create building with:[shape::shape, type::building_str]{
						industrial <- flip(0.4);
						if (industrial = true) {
							group<-'industrial';
							color <- #blue;
						} else{
							residential <-true;
							group<- 'residential';
							color <- #lightblue;
						}
						}
				}
			}
		}
		write "building agents filtered";
		
//		the agents extracted and their main/useful characterists are stored, on the base of their species in .shp file, so not to load the osm file everytime
		save road type:"shp" to:"../includes/roads.shp" with:[lanes::"lanes",maxspeed::"maxspeed", oneway::"oneway", junction::"junction"] ;
		save intersection type:"shp" to:"../includes/nodes.shp" with:[type::"type", crossing::"crossing"] ;
		save building type:"shp" to:"../includes/buildings.shp" with:[type::"type", group::"group"] ;
		road_network <-  (as_driving_graph(road, intersection));
			
	}
}

species road skills:[skill_road]{
	rgb color <- rgb(150,150,150);
	string type;
	string oneway;
	float maxspeed;
	int lanes;
	string junction;

	aspect base_ligne {
		draw shape color: color end_arrow:1; 
	}
} 
	
species intersection skills:[skill_road_node]{
	string type;
	string crossing;
	aspect base { 
		draw square(1) color: #mediumaquamarine ;
	}
} 

species osm_agent {
	string building_str;
} 

species building{
	string type;
	bool residential;
	bool industrial;
	string group;
	rgb color<- rgb(200,200,200);
	aspect base{
		draw shape color: color border: #black;
	}
}

experiment createMap type: gui {
	output {
		display map type: opengl {
			graphics "world" {
				draw world.shape.contour;
			}
			species road aspect: base_ligne refresh:false;
			species intersection aspect: base refresh:false;
			species building aspect: base refresh:false;
		}
	}
}