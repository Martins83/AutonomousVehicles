/***
* Name: building
* Author: Annalisa Congiu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model building

//species that will represent the buildings
species building{
	string type;
	string group;
	rgb color<- rgb(200,200,200);

	aspect base{
		draw shape color: color border: #gray;
	}
}

