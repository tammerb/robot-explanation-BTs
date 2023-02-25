import os
import rospy
import rospkg
import functools

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from explain_bt.srv import Explain, ExplainRequest


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

        self._widget.button_1.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question="What are you doing?"))
        self._widget.button_2.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question="Why are you doing this?"))
        self._widget.button_3.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question="How do you achieve your goal?"))
        self._widget.button_4.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question="What is your subgoal?"))
        self._widget.button_5.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question="What are the steps for your subgoal?"))
        self._widget.button_6.clicked[bool].connect(functools.partial(self._handle_explain_bt_service, question="What went wrong?"))

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
            add_two_ints = rospy.ServiceProxy('/explainable_bt', Explain)
            reply = add_two_ints(question).reply
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return


        self._widget.text_browser.append(reply)


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