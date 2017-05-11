$(function(){
    var $write = $('#write'),
        shift = false,
        capslock = false;
     
    $('#keyboard li').click(function(){
        var $this = $(this),
            character = $this.html(); // If it's a lowercase letter, nothing happens to this variable
         
        // Shift keys
        if ($this.hasClass('left-shift') || $this.hasClass('right-shift')) {
            $('.letter').toggleClass('uppercase');
            $('.symbol span').toggle();
             
            shift = (shift === true) ? false : true;
            capslock = false;
            return false;
        }
         
        // Caps lock
        if ($this.hasClass('capslock')) {
            $('.letter').toggleClass('uppercase');
            capslock = true;
            return false;
        }
         
        // Delete
        if ($this.hasClass('delete')) {
            var html = $write.html();
             
            $write.html(html.substr(0, html.length - 1));
            return false;
        }
         
        // Special characters
        if ($this.hasClass('symbol')) character = $('span:visible', $this).html();
        if ($this.hasClass('space')) character = ' ';
        if ($this.hasClass('tab')) character = "\t";
        if ($this.hasClass('return')) {
	
	playerName = $write.html();
	$write.html("");

	var scoreString = "S,"+badgeID+","+points+","+playerName+"\r";
	scoreboard.write(scoreString);

	$("#container").hide();
	$(".result").show();
	$(".result").html("<p>Welcome to the Game</p><p>"+playerName+"!<br>You've Deposited "+points+" Points!");
	window.setTimeout(welcomeScreen, 5000);

	}
         
        // Uppercase letter
        if ($this.hasClass('uppercase')) character = character.toUpperCase();
         
        // Remove shift once a key is clicked.
        if (shift === true) {
            $('.symbol span').toggle();
            if (capslock === false) $('.letter').toggleClass('uppercase');
             
            shift = false;
        }
         
        // Add the character
	if(character!="That's Me!"){
        $write.html($write.html() + character);
        }
            var nameString = $write.html();
    if( nameString.length > 20 ){
    $write.html(nameString.substring(0, 20));
  };
  
  
    });
    

});