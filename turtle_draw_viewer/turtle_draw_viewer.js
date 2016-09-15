(function() {
    var refresh_interval = 1 * 1000; // We check for new turtles every second

    // ROS setup

    // ROS connection
    var ros = new ROSLIB.Ros();

    // Error event
    ros.on("error", function(error) {
        $("#connecting").hide();
        $("#connected").hide();
        $("#closed").hide();
        $("#error").show();
        console.log(error);
    });

    // Connection event
    ros.on("connection", function() {
        console.log("Connection made!");
        $("#connecting").hide();
        $("#error").hide();
        $("#closed").hide();
        $("#connected").show();
    });

    // Connection closed event
    ros.on("close", function() {
        console.log("Connection closed.");
        $("#connecting").hide();
        $("#connected").hide();
        $("#closed").show();
    });

    function rosConnect() {
       ros.connect("ws://localhost:9090"); // Create a connection to the rosbridge WebSocket server.
    }

    // Canvas
    var canvas_cfg = {
        width : 500,
        height : 500
    };

    // Turtlesim config
    var ttlesim_cfg = {
        x : {
            min : 0,
            max : 11
        },
        y : {
            min : 0,
            max : 11
        },
        background : {
            r : 69,
            g : 86,
            b : 255
        }
    };

    // Turtles
    var turtles = {};

    function turtle(name) {
        this.name = name;

        // These properties will be set by the topic subscriber's callbacks,
        // but we need to create them so that they can be accessed later on
        this.pos = {
            x : 0,
            y : 0
        }

        this.color = {
            r : ttlesim_cfg.background.r,
            g : ttlesim_cfg.background.g,
            b : ttlesim_cfg.background.b
        };

        this.listeners = [];
    }

    function updateAliveTurtles() {
        // We find which turtles are alive by requesting the list of ROS topics,
        // and parsing that looking for turtle names
        var topics_client = new ROSLIB.Service({
            ros : ros,
            name : '/rosapi/topics',
            serviceType : 'rosapi/Topics'
        });

        var request = new ROSLIB.ServiceRequest();

        topics_client.callService(request, function(result) {
            // We keep the turtles that are still alive to avoid unsubscribing
            // and then subscribing again, and delete all of the dead turtles

            var current_turtle_names = [];
            _.each(result.topics, function(topic) {
                // We assume the turtle names start with "turtle". While parsing the list, we also specifically
                // check for the "pose" topic, to avoid duplicated entries
                if ((topic.indexOf("/turtle") != -1) && (topic.indexOf("/pose") != -1)) {
                    current_turtle_names.push(topic.substring(topic.indexOf("turtle"), topic.indexOf("/pose")));
                }
            });

            // Delete dead turtles
            _.each(turtles, function(turtle) {
                if ($.inArray(turtle.name, current_turtle_names) === (-1)) {
                    _.each(turtle.listeners, function(listener) {
                        listener.unsubscribe();
                    });
                    turtles[turtle.name] = null;
                }
            })

            _.each(current_turtle_names, function(turtle_name) {
                if (!(turtle_name in turtles)) {
                    // Add the newly created turtle

                    turtles[turtle_name] = new turtle(turtle_name);

                    // We can't subscribe until the new turtle has been created and added to the turtle dictionary,
                    // because the subscription callback will attempt to access the corresponding entry, and we'd
                    // have a race condition
                    turtles[turtle_name].listeners = subscribeTurtle(turtle_name);
                }
            });
        });
    }

    function subscribeTurtle(turtle_name) {
        // We need to subscribe to both the "color_sensor" and "pose" topics,
        // which will update the values of the turtle object associated with
        // each topic

        var color_listener = new ROSLIB.Topic({
            ros : ros,
            name : "/" + turtle_name + "/color_sensor",
            messageType : "turtlesim/Color"
        });

        color_listener.subscribe(function(message) {
            turtles[turtle_name].color = {
                r : message.r,
                g : message.g,
                b : message.b
            };
        });

        var pose_listener = new ROSLIB.Topic({
            ros : ros,
            name : "/" + turtle_name + "/pose",
            messageType : "turtlesim/Pose"
        });

        pose_listener.subscribe(function(message) {
            turtles[turtle_name].pos = {
                x : message.x,
                y : message.y
            }

            // Turtles are redrawn each time their position is udpdated
            drawTurtle(turtles[turtle_name]);
        });

        return [color_listener, pose_listener];
    }

    function drawTurtle(turtle) {
        if ((turtle.color.r != ttlesim_cfg.background.r) || (turtle.color.g != ttlesim_cfg.background.g) || (turtle.color.b != ttlesim_cfg.background.b)) {
            // We only draw the turtle if it's also drawing (that is, its pen isn't off). Since our background
            // color matches turtlesim's, though, this doesn't really matter that much.
            drawPoint(toCanvasXCoords(turtle.pos.x), toCanvasYCoords(turtle.pos.y), turtle.color);
        }
    }

    function toCanvasXCoords(val) {
        return ((val - ttlesim_cfg.x.min) / ttlesim_cfg.x.max) * canvas_cfg.width;
    }

    function toCanvasYCoords(val) {
        return ((val - ttlesim_cfg.y.min) / ttlesim_cfg.y.max) * canvas_cfg.height;
    }

    function drawPoint(x, y, color) {
        canvas_cfg.context.beginPath();
        canvas_cfg.context.arc(x, y, 2, 0, 2 * Math.PI, false); // A full circle
        canvas_cfg.context.fillStyle = "rgb(" + color.r + ", " + color.g + ", " + color.b + ")";;
        canvas_cfg.context.fill();
    }

    function setupCanvas() {
        var canvas = document.getElementsByTagName("canvas")[0];
        canvas.setAttribute("width", canvas_cfg.width);
        canvas.setAttribute("height", canvas_cfg.width);

        canvas_cfg.context = canvas.getContext("2d");

        clearCanvas();
        $("#clean").click(clearCanvas);
    }

    function clearCanvas() {
        canvas_cfg.context.clearRect(0, 0, canvas_cfg.width, canvas_cfg.height);
        canvas_cfg.context.fillStyle = "rgb(" + ttlesim_cfg.background.r + ", " + ttlesim_cfg.background.g + ", " + ttlesim_cfg.background.b + ")";
        canvas_cfg.context.fillRect(0, 0, canvas_cfg.width, canvas_cfg.height);
    }

    function main() {
        rosConnect();
        setupCanvas();

        updateAliveTurtles();
        setInterval(function() {
            updateAliveTurtles();
        }, refresh_interval);
    }

    $(document).ready(function() {
        main();
    });

})();
