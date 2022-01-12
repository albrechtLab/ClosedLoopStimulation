// Prior to experiment, find the window of interest within a 25% scaled image of your arena for analysis
x=13;
y=37;
w=224
h=177
makeRectangle(x, y, w, h);
run("Crop");
//

range=3; //edit if frames of interest is altered in main beanshell script
rename("main");

setOption("BlackBackground", false);
run("Make Binary", "method=Otsu background=Light calculate");
run("Analyze Particles...", "size=100-500 circularity=0.00-1.00 show=Masks display clear stack");
animalsize=getResult("Area",0);
selectWindow("main");
close();
selectWindow("Mask of main");
rename("main");

if(nResults>(range-1)){
	run("Duplicate...", "title=Current duplicate range="+range);
	imageCalculator("Difference create stack", "main","Current");
	selectWindow("Result of main");
	run("Clear Results");
	run("Analyze Particles...", "size=20-Infinity show=Masks clear stack");
	run("Clear Results");
	threshold=(animalsize*255/2)/(w*h)*(1/8);
	//threshold set to approximately 1/8th body size movement for a 0-255 grayscale
	run("Profile Plot Options...", "width=450 height=200 minimum=0 maximum=5 fixed interpolate draw");
	run("Plot Z-axis Profile");
	state=0;
for(o=0; o<range; o++){
	value=getResult("Mean",o);
	if (value>threshold){
		state=1;
	}
}
selectWindow("Mask of Result of main");
close();
selectWindow("Mask of Result of main-0-0");
close();
selectWindow("Result of main");
close();
selectWindow("Current");
close();
selectWindow("main");
close();
if (state==1){
	return "Awake";
}
else{
	return "Sleep";
}
}
else{
	selectWindow("main");
	close();
	return "Awake";
}
