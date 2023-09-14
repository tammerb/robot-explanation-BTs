import os
import rospy
import rospkg
import functools


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtCore import Qt

from explain_bt.srv import Explain, ExplainRequest
from std_srvs.srv import Empty, EmptyRequest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ExplainBTPlugin(Plugin):
    def __init__(self, context):
        super(ExplainBTPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ExplainBTPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_explain_bt'), 'resource', 'ExplainBTPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ExplainBTPlugin')

        self._widget.button_1.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_ARE_YOU_DOING))
        self._widget.button_2.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHY_ARE_YOU_DOING_THIS))
        self._widget.button_3.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_IS_YOUR_SUBGOAL))
        self._widget.button_4.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.HOW_DO_YOU_ACHIEVE_YOUR_SUBGOAL))
        self._widget.button_5.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_IS_YOUR_GOAL))
        self._widget.button_6.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.HOW_DO_YOU_ACHIEVE_YOUR_GOAL))
        self._widget.button_7.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_WENT_WRONG))  
        self._widget.button_8.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_IS_NEXT_ACTION_IF_SUCCESS))
        self._widget.button_9.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_IS_NEXT_ACTION_IF_FAIL)) 
        self._widget.button_10.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_ARE_CURRENT_PRE_CONDITIONS)) 
        self._widget.button_11.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question=ExplainRequest.WHAT_ARE_CURRENT_POST_CONDITIONS)) 

        self._widget.button_start.clicked[bool].connect(functools.partial(self._handle_empty_service, service_name='/start_tree'))
        self._widget.button_stop.clicked[bool].connect(functools.partial(self._handle_empty_service, service_name='/stop_tree'))
        self._widget.button_reset.clicked[bool].connect(functools.partial(self._handle_empty_service, service_name='/reset_tree'))

        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber('/tag_detections_image', Image, self._handle_image_update)

        self._image_label_width = self._widget.image_label.width()
        self._image_label_height = self._widget.image_label.height()

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


    def _handle_explain_bt_service(self, question):
        try:
            rospy.wait_for_service('/explain_tree', timeout=0.1)
            explain_srv_client = rospy.ServiceProxy('/explain_tree', Explain)
            answer = explain_srv_client(question).answer
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

        self._widget.text_browser.append(answer + "\n")

    def _handle_empty_service(self, service_name):
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
            srv_client = rospy.ServiceProxy(service_name, Empty)
            srv_client(EmptyRequest())
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def _handle_image_update(self, img_msg):
        cv_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        qt_img = self.convert_cv_qt(cv_img)
        self._widget.image_label.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        # rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        rgb_image = cv_img
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self._image_label_width, self._image_label_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog