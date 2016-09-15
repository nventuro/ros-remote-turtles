package org.ros.android.turtle_draw_control;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.concurrent.atomic.AtomicBoolean;

public class Pauser extends AbstractNodeMain {
    private String topic_name;
    private AtomicBoolean should_pause;

    public Pauser() {
        this.topic_name = "/turtle_draw/run";
        this.should_pause = new AtomicBoolean(true);
    }

    public Pauser(String topic_name, boolean should_pause) {
        this.topic_name = topic_name;
        this.should_pause = new AtomicBoolean(should_pause);
    }

    public void pause() {
        should_pause.set(true);
    }

    public void unpause() {
        should_pause.set(false);
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("turtle_draw_control/pauser");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Publisher publisher = connectedNode.newPublisher(this.topic_name, "std_msgs/Bool");
        final AtomicBoolean should_pause = this.should_pause;
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            protected void setup() {

            }

            protected void loop() throws InterruptedException {
                std_msgs.Bool bool = (std_msgs.Bool) publisher.newMessage();
                bool.setData(!should_pause.get()); // The 'run' topic pauses on False (not run) messages
                publisher.publish(bool);

                Thread.sleep(100L);
            }
        });
    }
}
