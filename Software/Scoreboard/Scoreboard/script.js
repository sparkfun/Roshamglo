var SerialPort = require('C:\\Users\\sparkfun_large\\AppData\\Roaming\\npm\\node_modules\\serialport');

var Twit = require('C:\\Users\\sparkfun_large\\AppData\\Roaming\\npm\\node_modules\\twit');

var T = new Twit({
  consumer_key:         'WD8e2gPJ4Dek2BVSuo8aoQ1xE',
  consumer_secret:      'xA9qcdXQioEjS4rTVEBZsdGc5Ykoxe3JSkIYYaOm3YmmNyKmyc',
  access_token:         '811262965391519744-jQl42XJI3kdEWpTAdph9pWnl4XrgG54',
  access_token_secret:  'YdQ2SqNlnmnF8oy9CETf5jYKx0UJqctwscC9W3DMAeifw',
  timeout_ms:           60*1000,  // optional HTTP request timeout to apply to all requests.
})

var middleBank = new SerialPort('COM5', {
  parser: SerialPort.parsers.readline('\r'),
  baudRate: 9600
});

var leftBank = new SerialPort('COM8', {
  parser: SerialPort.parsers.readline('\r'),
  baudRate: 9600
});

var rightBank = new SerialPort('COM4', {
  parser: SerialPort.parsers.readline('\r'),
  baudRate: 9600
});

middleBank.on('data', function (data) {
  
  var msgType = data.split(",")[0];
  
  switch (msgType) {
	  
	  case "S":
  
		  var newID = data.split(",")[1];
		  var newScore = data.split(",")[2];
		  var playerName = data.split(",")[3];
		  
		  console.log(data);
		  
		  if ( newScore == "FF") {
			  console.log("Middle FF error");
			  newScore = "0";
		  }
		  
		  if(getScoreByID(newID)){
			setScoreByID(newID, parseInt(getScoreByID(newID))+parseInt(newScore));
		  }else{
			var tableRecord = "<tr><td>100000</td><td style=\"display: none;\">" + newID + "</td><td>" + playerName + "</td><td>" + newScore + "</td></tr>";
			$( "#leaderBoard" ).append(tableRecord);
		  }

			sortAndRank();
			break;
			
	 	case "N":

			var searchID = data.split(",")[1];
			middleBank.write(getNameByID(searchID) + "\r");
			break;
			
  }
  
});

leftBank.on('data', function (data) {
  
  var msgType = data.split(",")[0];
  
  switch (msgType) {
	  
	  case "S":
  
		  var newID = data.split(",")[1];
		  var newScore = data.split(",")[2];
		  var playerName = data.split(",")[3];
		  
		  console.log(data);
		  
		  if ( newScore == "FF") {
			  console.log("Left FF error");
			  newScore = "0";
		  }
		  
		  if(getScoreByID(newID)){
			setScoreByID(newID, parseInt(getScoreByID(newID))+parseInt(newScore));
		  }else{
			var tableRecord = "<tr><td>100000</td><td style=\"display: none;\">" + newID + "</td><td>" + playerName + "</td><td>" + newScore + "</td></tr>";
			$( "#leaderBoard" ).append(tableRecord);
		  }

			sortAndRank();
			break;
			
	 	case "N":

			var searchID = data.split(",")[1];
			leftBank.write(getNameByID(searchID) + "\r");
			break;
			
  }
  
});

rightBank.on('data', function (data) {
  
  var msgType = data.split(",")[0];
  
  switch (msgType) {
	  
	  case "S":
  
		  var newID = data.split(",")[1];
		  var newScore = data.split(",")[2];
		  var playerName = data.split(",")[3];
		  
		  console.log(data);
		  
		  if ( newScore == "FF") {
			  console.log("Right FF error");
			  newScore = "0";
		  }
		  
		  if(getScoreByID(newID)){
			setScoreByID(newID, parseInt(getScoreByID(newID))+parseInt(newScore));
		  }else{
			var tableRecord = "<tr><td>100000</td><td style=\"display: none;\">" + newID + "</td><td>" + playerName + "</td><td>" + newScore + "</td></tr>";
			$( "#leaderBoard" ).append(tableRecord);
		  }

			sortAndRank();
			break;
			
	 	case "N":

			var searchID = data.split(",")[1];
			rightBank.write(getNameByID(searchID) + "\r");
			break;
			
  }
  
});

function getScoreByID(id){
	
  var table = document.getElementById("leaderBoard");
  var trows = table.getElementsByTagName("tr");
  var tdata;
  
  for (i = 0; i < trows.length; i++){
	  tdata = trows[i].getElementsByTagName("td")[1];
	  if (tdata) {
		if (tdata.innerHTML.indexOf(id) > -1){
			return trows[i].getElementsByTagName("td")[3].innerHTML;
		}		
	 }	  
  }
  
  return false;
	
}

function setScoreByID(id, score){
	
  var table = document.getElementById("leaderBoard");
  var trows = table.getElementsByTagName("tr");
  var tdata;
  
  for (i = 0; i < trows.length; i++){
	  tdata = trows[i].getElementsByTagName("td")[1];
	  if (tdata) {
		if (tdata.innerHTML.indexOf(id) > -1){
			console.log("attempt to update score to " + score + ".");
			if ( isNaN(score) || (score == "FF") )  {
				trows[i].getElementsByTagName("td")[3].innerHTML = 0;
			} else {
				trows[i].getElementsByTagName("td")[3].innerHTML = score;
			}
			return true;
		}		
	 }	  
  }
  
  return false;
	
}

function sortAndRank(){
	
  var leader = document.getElementById("leaderBoard").getElementsByTagName("tr")[1].getElementsByTagName("td")[2].innerHTML;
  var table = document.getElementById("leaderBoard");
  var trows;
  var tdata;
  var sorting = true;
  var x;
  var y;
  
  while (sorting) {
	  
	  sorting = false;
	  trows = table.getElementsByTagName("tr");
	  
	  for (i = 1; i < (trows.length - 1); i++) {
		  
		x = trows[i].getElementsByTagName("td")[3];  
		y = trows[i + 1].getElementsByTagName("td")[3]; 

		if ( parseInt(x.innerHTML) < parseInt(y.innerHTML) ){
			trows[i].parentNode.insertBefore(trows[i+1], trows[i]);
			sorting = true;
		}
		  
	  }
	  
  }
  
  for (i = 1; i < trows.length; i++) {
	  
	  trows[i].getElementsByTagName("td")[0].innerHTML = i;
	  
  }
  
    var newLeader = document.getElementById("leaderBoard").getElementsByTagName("tr")[1].getElementsByTagName("td")[2].innerHTML;

	if ( leader != newLeader ) {
		
	var dt = new Date();	
		
	var msg = "[" + dt.getTime() + "] " + newLeader + " just overtook " + leader + " for first place! #PlayHardware #SXSW";
		console.log(msg);	
		T.post('statuses/update', { status: msg }, function(err, data, response) {
  console.log(data)
})
		
	}	
}

function getNameByID(id){
	
  var table = document.getElementById("leaderBoard");
  var trows = table.getElementsByTagName("tr");
  var tdata;
  
  for (i = 0; i < trows.length; i++){
	  tdata = trows[i].getElementsByTagName("td")[1];
	  if (tdata) {
		if (tdata.innerHTML.indexOf(id) > -1){
			return trows[i].getElementsByTagName("td")[2].innerHTML;
		}		
	 }	  
  }
  
  return 0;
	
}

setInterval(function(){
	
	$('html, body').animate({ scrollTop: $(document).height() }, 10000);
	$('html, body').animate({ scrollTop: 0 }, 10000);
	
}, 60000);

setInterval(function(){
	
	$('#banner').fadeIn();
	setTimeout(function(){$('#banner').fadeOut();}, 10000);
	
}, 600000);

setInterval(function(){
	var newLeader = document.getElementById("leaderBoard").getElementsByTagName("tr")[1].getElementsByTagName("td")[2].innerHTML;
	var newRunner = document.getElementById("leaderBoard").getElementsByTagName("tr")[2].getElementsByTagName("td")[2].innerHTML;
	var dt = new Date();	
	var msg = "[" + dt.getTime() + "] Hourly Update! " + newLeader + " is our current leader!! " + newRunner + " is runner-up! #PlayHardware #SXSW";
		console.log(msg);	
		T.post('statuses/update', { status: msg }, function(err, data, response) {
  console.log(data)
})
}, 3600000);






