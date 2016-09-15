package org.ros.android.turtle_draw_control;

import android.os.Bundle;
import android.view.View;
import android.widget.CheckBox;

import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.android.turtle_draw_control.Pauser;

public class MainActivity extends RosActivity {

private RosTextView<std_msgs.Bool> rosTextView;
private Pauser pauser;
private String topic_name;
private CheckBox checkbox;

public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("Turtle Draw Control", "Turtle Draw Control");
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        topic_name = "/turtle_draw/run";

        // Configure the checkbox
        checkbox = (CheckBox) findViewById(R.id.pause_checkbox);
        checkbox.setChecked(true); // Initially paused
        checkbox.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (checkbox.isChecked()) {
                    pauser.pause();
                } else {
                    pauser.unpause();
                }
            }
        });

        // Configure the textview to show the paused/unpaused state
        rosTextView = (RosTextView<std_msgs.Bool>) findViewById(R.id.text);
        rosTextView.setTopicName(topic_name);
        rosTextView.setMessageType(std_msgs.Bool._TYPE);
        rosTextView.setMessageToStringCallable(new MessageCallable<String, std_msgs.Bool>() {
            @Override
            public String call(std_msgs.Bool message) {
                return message.getData() ? "Running" : "Paused";
            }
        });
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        pauser = new Pauser(topic_name, true); // Initially paused

        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.

        // The user can easily use the selected ROS Hostname in the master chooser
        // activity.
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(pauser, nodeConfiguration);
        // The RosTextView is also a NodeMain that must be executed in order to
        // start displaying incoming messages.
        nodeMainExecutor.execute(rosTextView, nodeConfiguration);
    }
}
