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

    var turtlesData = {};

    function setupCanvas() {
        var canvas = document.getElementsByTagName("canvas")[0];
        canvas.setAttribute("width", canvasWidth);
        canvas.setAttribute("height", canvasHeight);
        canvasContext = canvas.getContext("2d");

        clearCanvas();
        $("#clean").click(clearCanvas);

        $("#topics").click(getTopics);
    }

    function clearCanvas() {
        canvasContext.clearRect(0, 0, canvasWidth, canvasHeight);
        canvasContext.fillStyle = "rgb(" + ttlesimBackground.r + ", " + ttlesimBackground.g + ", " + ttlesimBackground.b + ")";
        canvasContext.fillRect(0, 0, canvasWidth, canvasHeight);
    }

    function getTopics() {
        var topicsClient = new ROSLIB.Service({
            ros : ros,
            name : '/rosapi/topics',
            serviceType : 'rosapi/Topics'
        });

        var request = new ROSLIB.ServiceRequest();

        topicsClient.callService(request, function(result) {
            console.log("Getting topics...");

            var turtle_names = [];
            _.each(result.topics, function(topic) {
                // We assume the turtle names start with "turtle". We also specifically check
                // for the "pose" topic, to avoid duplicated entries
                if ((topic.indexOf("/turtle") != -1) && (topic.indexOf("/pose") != -1)) {
                    turtle_names.push("turtle" + parseInt(topic.substring(topic.indexOf("turtle") + "turtle".length)));
                }
            });

            console.log(turtle_names);
        });
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

    function subscribe(turtle_name) {
        // We need to subscribe to both the "color_sensor" and "pose" topics

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
                drawPoint(toCanvasCoords(message.x, "x"), toCanvasCoords(message.y, "y"), turtlesData[turtle_name]);
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
