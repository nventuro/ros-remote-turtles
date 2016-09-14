(function() {

    // ROS setup

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

    // Turtles
    var turtles = {};

    function turtle() {
        // These properties will be set by the topic subscriber's callbacks,
        // but we need to create them so that they can be accessed later on
        this.pos = {};
        this.pos.x = 0;
        this.pos.y = 0;

        this.color = {};
        this.color.r = ttlesimBackground.r;
        this.color.g = ttlesimBackground.g;
        this.color.b = ttlesimBackground.b;
    }

    function drawTurtle(turtle) {
        if ((turtle.color.r != ttlesimBackground.r) || (turtle.color.g != ttlesimBackground.g) || (turtle.color.b != ttlesimBackground.b)) {
            drawPoint(toCanvasCoords(turtle.pos.x, "x"), toCanvasCoords(turtle.pos.y, "y"), turtle.color);
        }
    }

    function toCanvasCoords(val, coord) {
        if (coord === "x") {
            return ((val - ttlesimXMin) / ttlesimXMax) * canvasWidth;
        } else { // Assume "y"
            return ((val - ttlesimYMin) / ttlesimYMax) * canvasHeight;
        }
    }

    function drawPoint(x, y, color) {
        canvasContext.beginPath();
        canvasContext.arc(x, y, 2, 0, 2 * Math.PI, false); // A full circle
        canvasContext.fillStyle = "rgb(" + color.r + ", " + color.g + ", " + color.b + ")";;
        canvasContext.fill();
    }

    function setupCanvas() {
        var canvas = document.getElementsByTagName("canvas")[0];
        canvas.setAttribute("width", canvasWidth);
        canvas.setAttribute("height", canvasHeight);
        canvasContext = canvas.getContext("2d");

        clearCanvas();
        $("#clean").click(clearCanvas);
    }

    function clearCanvas() {
        canvasContext.clearRect(0, 0, canvasWidth, canvasHeight);
        canvasContext.fillStyle = "rgb(" + ttlesimBackground.r + ", " + ttlesimBackground.g + ", " + ttlesimBackground.b + ")";
        canvasContext.fillRect(0, 0, canvasWidth, canvasHeight);
    }

    function updateAliveTurtles() {
        // We find which turtles are alive by requesting the list of ROS topics,
        // and parsing that looking for turtle names
        var topicsClient = new ROSLIB.Service({
            ros : ros,
            name : '/rosapi/topics',
            serviceType : 'rosapi/Topics'
        });

        var request = new ROSLIB.ServiceRequest();

        topicsClient.callService(request, function(result) {
            // We could keep the turtles that are still alive, add the new
            // ones and delete the dead ones, but we simply clear them all
            // and then add all of the alive ones.

            turtles = {}
            _.each(result.topics, function(topic) {
                // We assume the turtle names are of the form "turtle%u" (where %u is an unsigned integer). While
                // parsing the list, we also specifically check for the "pose" topic, to avoid duplicated entries
                if ((topic.indexOf("/turtle") != -1) && (topic.indexOf("/pose") != -1)) {
                    var turtle_name = "turtle" + parseInt(topic.substring(topic.indexOf("turtle") + "turtle".length));
                    turtles[turtle_name] = new turtle();
                    subscribe(turtle_name);
                }
            });
        });
    }

    function subscribe(turtle_name) {
        // We need to subscribe to both the "color_sensor" and "pose" topics,
        // which will update the values of the turtle object associated with
        // each topic

        var color_sub = new ROSLIB.Topic({
            ros : ros,
            name : "/" + turtle_name + "/color_sensor",
            messageType : "turtlesim/Color"
        });

        color_sub.subscribe(function(message) {
            turtles[turtle_name].color.r = message.r;
            turtles[turtle_name].color.g = message.g;
            turtles[turtle_name].color.b = message.b;
        });

        var pose_sub = new ROSLIB.Topic({
            ros : ros,
            name : "/" + turtle_name + "/pose",
            messageType : "turtlesim/Pose"
        });

        pose_sub.subscribe(function(message) {
            turtles[turtle_name].pos.x = message.x;
            turtles[turtle_name].pos.y = message.y;

            // Turtles are redrawn each time their position is udpdated
            drawTurtle(turtles[turtle_name]);
        });
    }

    function main() {
        rosConnect();
        setupCanvas();

        $("#refresh").click(updateAliveTurtles);
    }

    $(document).ready(function() {
        main();
    });

})();
