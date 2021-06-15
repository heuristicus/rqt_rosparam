import os
import rospy
import rospkg
import rosparam

from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTreeView, QLineEdit
from python_qt_binding.QtGui import QStandardItemModel

class RqtRosParam(Plugin):
    def __init__(self, context):
        super(RqtRosParam, self).__init__(context)
        self.setObjectName("rqt_rosparam")

        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument(
            "-q",
            "--quiet",
            action="store_true",
            dest="quiet",
            help="Put plugin in silent mode",
        )
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print("arguments: ", args)
            print("unknowns: ", unknowns)

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("rqt_rosparam"), "resource", "RosParam.ui"
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("rqt_rosparam")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )

        self._param_tree = self._widget.findChildren(QTreeView, "paramTree")[0]
        self._filter_box = self._widget.findChildren(QLineEdit, "filterEntry")[0]
        self._model = QStandardItemModel()
        self._model.setHorizontalHeaderLabels(['Key', 'Value'])
        # There are only two sections so they can't be moved anyway
        self._param_tree.header().setSectionsMovable(False)
        self._param_tree.header().setDefaultSectionSize(200)
        self._param_tree.setModel(self._model)
        yaml_tree = rosparam.get_param("/")

        print(yaml_tree)



        context.add_widget(self._widget)

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

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
