(function() {

    // ROS

    // ROS connection
    var ros = new ROSLIB.Ros();

    // Error event
    ros.on("error", function(error) {
        document.getElementById("connecting").style.display = "none";
        document.getElementById("connected").style.display = "none";
        document.getElementById("closed").style.display = "none";
        document.getElementById("error").style.display = "inline";
        console.log(error);
    });

    // Connection event
    ros.on("connection", function() {
        console.log("Connection made!");
        document.getElementById("connecting").style.display = "none";
        document.getElementById("error").style.display = "none";
        document.getElementById("closed").style.display = "none";
        document.getElementById("connected").style.display = "inline";
    });

    // Connection closed event
    ros.on("close", function() {
        console.log("Connection closed.");
        document.getElementById("connecting").style.display = "none";
        document.getElementById("connected").style.display = "none";
        document.getElementById("closed").style.display = "inline";
    });

    function rosConnect() {
       ros.connect("ws://localhost:9090"); // Create a connection to the rosbridge WebSocket server.
    }

    // Canvas
    var canvasWidth = 500;
    var canvasHeight = 500;
    var canvasContext;

    // Turtlesim config
    var ttlesimXMin = 0;
    var ttlesimXMax = 11;
    var ttlesimYMin = 0;
    var ttlesimYMax = 11;

    var ttlesimBackground = {};
    ttlesimBackground.r = 69;
    ttlesimBackground.g = 86;
    ttlesimBackground.b = 255;

    var turtlesData = {};

    function setupCanvas() {
        var canvas = document.getElementsByTagName("canvas")[0];
        canvas.setAttribute("width", canvasWidth);
        canvas.setAttribute("height", canvasHeight);
        canvasContext = canvas.getContext("2d");

        $("#clean").click(function() {
            canvasContext.clearRect(0, 0, canvasWidth, canvasHeight);
        });
    }

    function toCanvasCoords(val, coord) {
        if (coord === "x") {
            return ((val - ttlesimXMin) / ttlesimXMax) * canvasWidth;
        } else { // Assume "y"
            return ((val - ttlesimYMin) / ttlesimYMax) * canvasHeight;
        }
    }

    function drawPoint(x, y) {
        canvasContext.beginPath();
        canvasContext.arc(x, y, 2, 0, 2 * Math.PI, false); // A full circle
        canvasContext.fillStyle = "black";
        canvasContext.fill();
    }

    function subscribe(turtle_name) {
        var color_sub = new ROSLIB.Topic({
            ros : ros,
            name : "/" + turtle_name + "/color_sensor",
            messageType : "turtlesim/Color"
        });

        color_sub.subscribe(function(message) {
            turtlesData[turtle_name] = {};
            turtlesData[turtle_name].r = message.r;
            turtlesData[turtle_name].g = message.g;
            turtlesData[turtle_name].b = message.b;
        });

        var pose_sub = new ROSLIB.Topic({
            ros : ros,
            name : "/" + turtle_name + "/pose",
            messageType : "turtlesim/Pose"
        });

        pose_sub.subscribe(function(message) {
            if ((turtlesData[turtle_name].r != ttlesimBackground.r) || (turtlesData[turtle_name].g != ttlesimBackground.g) || (turtlesData[turtle_name].b != ttlesimBackground.b)) {
                drawPoint(toCanvasCoords(message.x, "x"), toCanvasCoords(message.y, "y"));
            }
        });
    }

    function main() {
        rosConnect();
        setupCanvas();

        subscribe("turtle2");
    }

    $(document).ready(function() {
        main();
    });

})();