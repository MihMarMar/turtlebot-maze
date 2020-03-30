#! /usr/bin/env python
import sys
import rospy
import cv2
from pyzbar import pyzbar
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

should_scan = False


class QRScanner:
    def __init__(self, publisher=None):
        self.publisher = publisher
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        global should_scan
        try:

                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                im_resized = cv2.resize(cv_image, (480, 270))
                barcodes = pyzbar.decode(im_resized)
                if should_scan:
                    for barcode in barcodes:
                        (x, y, w, h) = barcode.rect
                        cv2.rectangle(im_resized, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        bcData = barcode.data.decode("utf-8")
                        bcType = barcode.type
                        text = "{} ({})".format(bcData, bcType)
                        cv2.putText(im_resized, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        rospy.loginfo("barcode read " + bcData)
                        if self.publisher:
                            self.publisher.publish(bcData)
                    should_scan = False
                cv2.imshow("Image window", im_resized)
                cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)


class MessagePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.pub = rospy.Publisher(topic, String, queue_size=10)

    def publish(self, msg):
        if not rospy.is_shutdown():
            rospy.loginfo(msg)
            self.pub.publish(msg)


def scan_cb(data):
    global should_scan
    should_scan = True


def main(args):
    pub = MessagePublisher("/turtlekin/qr")
    sub = rospy.Subscriber("/turtlekin/scan", String, scan_cb)
    QRScanner(publisher=pub)
    rospy.init_node('qr_reader', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
