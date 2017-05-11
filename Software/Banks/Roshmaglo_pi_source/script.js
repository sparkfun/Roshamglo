var SerialPort = require('/home/pi/node_modules/serialport');
 
var badgeReader = new SerialPort('/dev/Infrared', {
  parser: SerialPort.parsers.readline('\r\n'),
  baudRate: 9600
});

var scoreboard = new SerialPort('/dev/Scoreboard', {
  parser: SerialPort.parsers.readline('\r'),
  baudRate: 9600
});

var badgeID = "";
var points = 0;
var playerName = 0;

badgeReader.on('data', function (data) {
  badgeID = data.split(',')[0];
  points = parseInt(data.split(',')[1], 16);

  // Catch 0xFF error from badges
  if ( points == 255 ) {
    console.log("0xFF error. Resetting points to 0.");
    points = 0;
  }

  console.log(badgeID);
  console.log(points);

  scoreboard.write("N,"+badgeID+"\r");

});

scoreboard.on('data', function (data) {

  playerName = data;

  if(playerName != 0){

	var scoreString = "S,"+badgeID+","+points+","+playerName+"\r";
	scoreboard.write(scoreString);
	$(".result").show();
	$(".result").html("<p>Welcome Back</p><p>"+playerName+"!<br>You've Deposited "+points+" Points!");
	window.setTimeout(welcomeScreen, 5000);
  } else {
 	
	$(".result").hide();
	$("#container").show();

  }


});

function welcomeScreen(){
$(".result").html("<p> >  >  > </p><p> Fire your badge! </p><p> >  >  > </p>");
}



