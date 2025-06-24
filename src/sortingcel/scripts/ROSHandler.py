#!/usr/bin/env python3
import rospy
from threading import Lock

# RosHandler class to manage publishers and subscribers
class ROSHandler:
    def __init__(self, rospy):
        self.publishers = {}
        self.subscribers = {}
        self.callbacks = {}
        self.locks = {}

    def create_publisher(self, topic_name, msg_type, queue_size=10):
        """
        Registers a publisher on a given topic.
        """
        if topic_name not in self.publishers:
            pub = rospy.Publisher(topic_name, msg_type, queue_size=queue_size)
            self.publishers[topic_name] = pub
            rospy.loginfo(f"Publisher created: {topic_name}")
        return self.publishers[topic_name]

    def create_subscriber(self, topic_name, msg_type, callback, queue_size=10):
        """
        Registers a subscriber on a given topic with a custom callback.
        """
        if topic_name not in self.subscribers:
            lock = Lock()
            wrapped_callback = self._wrap_callback(callback, lock)
            sub = rospy.Subscriber(topic_name, msg_type, wrapped_callback, queue_size=queue_size)
            self.subscribers[topic_name] = sub
            self.callbacks[topic_name] = wrapped_callback
            self.locks[topic_name] = lock
            rospy.loginfo(f"Subscriber created: {topic_name}")
        return self.subscribers[topic_name]

    def _wrap_callback(self, callback, lock):
        """
        Wraps the callback with a thread lock.
        """
        def wrapped(msg):
            with lock:
                callback(msg)
        return wrapped

    def publish(self, topic_name, msg):
        """
        Publishes a message to a given topic.
        """
        if topic_name in self.publishers:
            self.publishers[topic_name].publish(msg)
        else:
            rospy.logwarn(f"Tried to publish to unknown topic: {topic_name}")

    def shutdown(self):
        """
        Cleanly shuts down all connections.
        """
        rospy.loginfo("Shutting down ROSHandler.")
        for topic, pub in self.publishers.items():
            pub.unregister()
        for topic, sub in self.subscribers.items():
            sub.unregister()

