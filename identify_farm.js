// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();
// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('error').style.display = 'inline';
  console.log(error);
});
// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('error').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('connected').style.display = 'inline';
});
ros.on('close', function() {
  console.log('Connection closed.');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'inline';
});
// Create a connection to the rosbridge WebSocket server.
//ros.connect('ws://127.0.0.1:9090');
//ros.connect('ws://192.168.7.1:9090');
//ros.connect('ws://192.168.43.103:9090');
ros.connect('ws://192.168.1.101:9090');

var myIcon = L.icon({
    iconUrl: 'red_dot.png',
    iconSize: [10, 10]
});

var robot_marker
var botPosition
var offsetLat, offsetLng
var datum_set = false;
function offsetGPS(){
  console.log(robot_marker._latlng)
  console.log(botPosition)
  offsetLat = robot_marker._latlng.lat - botPosition[0]
  offsetLng = robot_marker._latlng.lng - botPosition[1]
  datum_set = true;
  //console.log(offsetLat,offsetLng)
}

var datum = new ROSLIB.Param({
  ros : ros,
  name : 'datum'
});
function goto_robot(){
  datum.get(function(value) {
    var val = [value[0],value[1]]
    console.log(val);
    robot_marker = L.marker(val, {draggable: true, icon: myIcon}).addTo(map).on('moveend', offsetGPS);
    botPosition = val
    map.setView(robot_marker.getLatLng(),21);
  });
}

var goalPublisher = new ROSLIB.Topic({
  ros : ros,
  name : '/GPS_points',
  messageType : 'geometry_msgs/Point'
});

var GPSgoal = new ROSLIB.Message({
    x : 0,
    y : 0,
    z : 0
});

function sendGoal(point){
  GPSgoal.x = point[1] - offsetLat;
  GPSgoal.y = point[0] - offsetLng;
  console.log(GPSgoal)
  goalPublisher.publish(GPSgoal);
}

//var current_robot_location = [18.954724,72.815926];
var current_robot_location = [18.954639,72.815924];
var map = L.map('mapid').setView(current_robot_location, 21);
L.tileLayer('http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',{
    maxZoom: 20,
    subdomains:['mt0','mt1','mt2','mt3']
				}).addTo(map);

//Create a topic robot's GPS Position
var robotGPS = new ROSLIB.Topic({
  ros : ros,
  name : '/fix',
  messageType : 'sensor_msgs/NavSatFix'
});

//Listener to recieve Robot's GPS position, and update that marker on the map.
robotGPS.subscribe(function(message) {
  current_robot_location[0] = message.latitude;
  current_robot_location[1] = message.longitude;
  if(datum_set){
    current_robot_location[0] = current_robot_location[0] + offsetLat;
    current_robot_location[1] = current_robot_location[1] + offsetLng;
    robot_marker.setLatLng(current_robot_location);
    console.log(current_robot_location);
  }
});

var drawnItems = new L.FeatureGroup();
map.addLayer(drawnItems);
var drawControl = new L.Control.Draw({
	draw : {
		polyline : true,
		circle : false,
		marker : false,
		polygon : false,
		circlemarker : false,
		rectangle : false,
	},

	edit : {
		featureGroup : drawnItems
	}
});
map.addControl(drawControl);

var geojson

// Currently, we are not managing any layers. Assumption is that only a single farm
// will be identified in a single setting, thus we are allowing only ONE layer, and
// treating that layer as the farm.

map.on(L.Draw.Event.CREATED, function(event) {
  drawnItems.eachLayer(function (oldLayer) {
    drawnItems.removeLayer(oldLayer);
  });
  var layer = event.layer;
	geojson = layer.toGeoJSON();
  drawnItems.addLayer(layer);
  if(layer.editing._marker){
    sendGoal(geojson.geometry.coordinates)
    console.log(geojson.geometry.coordinates)
  }
  else{
		document.getElementById("feedback").innerHTML = 'Waypoints identified';
		document.getElementById("readytosend").innerHTML = '<button onclick="sendCoordinates()">Send Waypoints</button>';
		console.log(geojson.geometry.coordinates)
  }
});

map.on(L.Draw.Event.EDITED, function (e) {
    var layers = e.layers;
    layers.eachLayer(function (layer) {
    	geojson = layer.toGeoJSON();
    	console.log(geojson);
    });
});

function publish_limits(data) {
  var list_of_msgs =[]
  for (x in data){
    list_of_msgs.push(new ROSLIB.Message({
    data : data[x]}))
  };
  var points = new ROSLIB.Message({
    point : list_of_msgs
  });

  // Create topic and publish the data to that topic.
  var farmLimits = new ROSLIB.Topic({
    ros : ros,
    name : '/waypoints',
    messageType : 'farm_limits/FarmLimits'
  });
  farmLimits.publish(points);
}

function sendCoordinates() {
	var features = geojson['geometry']['coordinates'];
	publish_limits(features);
	document.getElementById("feedback").innerHTML = 'Waypoints sent to bot';
	document.getElementById("readytosend").innerHTML = '';
}
