$(document).ready(function(){
    //connect to the socket server.
    var socket = io.connect('http://' + document.domain + ':' + location.port + '/test');
    var messages_received = [];

    //receive details from server
    socket.on('emailmsg', function(msg) {
        console.log("Received Message " + msg.data);
        messages_received.push(msg.data);
        message_string = '';
        for (var i = 0; i < messages_received.length; i++){
            message_string = my_string + '<p>' + messages_received[i] + '</p>';
        }
        $('#log').html(message_string);
    });

});