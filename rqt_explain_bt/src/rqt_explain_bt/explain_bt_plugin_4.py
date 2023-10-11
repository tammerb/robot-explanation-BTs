import rospy
import functools

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QGridLayout, QTextBrowser, QLabel, QHBoxLayout
from python_qt_binding.QtCore import QObject, pyqtSignal, Qt
from python_qt_binding.QtGui import QFont

from std_srvs.srv import Empty, EmptyRequest
from explain_bt.msg import Explanations

class UpdateTextSignal(QObject):
    update_text_signal = pyqtSignal(str)

class ExplainBTPlugin(Plugin):
    def __init__(self, context):
        super(ExplainBTPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ExplainBTPlugin')

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName('ExplainBTPlugin')

        # Create a vertical layout for the entire widget
        layout = QVBoxLayout(self._widget)

        # Create a horizontal layout for the three buttons
        button_layout = QHBoxLayout()
        button_layout.setAlignment(Qt.AlignLeft)
        # start, stop, reset button
        button_names = ['Start', 'Stop', 'Reset']
        service_names = ['/start_tree', '/stop_tree', '/reset_tree']
        for button_name, service_name in zip(button_names, service_names):
            button = QPushButton(button_name)

            button.setFont(QFont("Arial", 14))
            button_style = "QPushButton { padding: 10px; min-width: 75px; }"
            button.setStyleSheet(button_style)

            button_layout.addWidget(button)
            button.clicked.connect(functools.partial(self._handle_empty_service, service_name=service_name))

        # Create a grid layout for the 4x3 grid
        questions = [
            "What are you doing?",
            "Why are you doing this?",
            "What is your subgoal?",
            "How do you achieve your subgoal?",
            "What is your goal?",
            "How do you achieve your goal?",
            "What went wrong?",
            "What is your next action if you succeed?",
            "What is your next action if you fail?",
            "What are your current pre-conditions?",
            "What are your current post-conditions?",
        ]
        self.update_text_signals = []

        grid_layout = QGridLayout()

        # Add labels and text browsers to each grid square
        for i, question in enumerate(questions):
            label = QLabel(question)
            label.setFont(QFont("Arial", 14))
            label.setContentsMargins(0, 20, 0, 10) 
            text_browser = QTextBrowser()
            text_browser.setFont(QFont("Arial", 14))
            update_text_signal = UpdateTextSignal()
            update_text_signal.update_text_signal.connect(functools.partial(self._handle_update_text_browser, text_browser=text_browser))
            self.update_text_signals.append(update_text_signal)
            # Add the label and text browser to a vertical layout
            vertical_layout = QVBoxLayout()
            vertical_layout.addWidget(label)
            vertical_layout.addWidget(text_browser)
            # Add the vertical layout to the grid
            grid_layout.addLayout(vertical_layout, i // 4, i % 4)

        # Add the button layout and grid layout to the main vertical layout
        layout.addLayout(button_layout)
        layout.addLayout(grid_layout)

        # Add the main layout to the widget
        self._widget.setLayout(layout)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.explanations_sub = rospy.Subscriber('/explanations', Explanations, self._handle_explanations_callback)

    def _handle_explanations_callback(self, explanations_msg):
        self.update_text_signals[0].update_text_signal.emit(explanations_msg.what_are_you_doing)
        self.update_text_signals[1].update_text_signal.emit(explanations_msg.why_are_you_doing_this)
        self.update_text_signals[2].update_text_signal.emit(explanations_msg.what_is_your_subgoal)
        self.update_text_signals[3].update_text_signal.emit(explanations_msg.how_do_you_achieve_your_subgoal)
        self.update_text_signals[4].update_text_signal.emit(explanations_msg.what_is_your_goal)
        self.update_text_signals[5].update_text_signal.emit(explanations_msg.how_do_you_achieve_your_goal)
        self.update_text_signals[6].update_text_signal.emit(explanations_msg.what_went_wrong)
        self.update_text_signals[7].update_text_signal.emit(explanations_msg.what_is_next_action_if_success)
        self.update_text_signals[8].update_text_signal.emit(explanations_msg.what_is_next_action_if_fail)
        self.update_text_signals[9].update_text_signal.emit(explanations_msg.what_are_current_pre_conditions)
        self.update_text_signals[10].update_text_signal.emit(explanations_msg.what_are_current_post_conditions)

    def _handle_update_text_browser(self, text, text_browser):
        text_browser.setPlainText(text)

    def _handle_empty_service(self, service_name):
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
            srv_client = rospy.ServiceProxy(service_name, Empty)
            srv_client(EmptyRequest())
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def shutdown_plugin(self):
        self.explanations_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass