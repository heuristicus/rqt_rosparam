import os
import rospy
import rospkg
import rosparam
import yaml

from collections.abc import Mapping
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTreeView, QLineEdit, QPushButton
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtCore import QSortFilterProxyModel


class RqtRosParam(Plugin):
    def __init__(self, context):
        super(RqtRosParam, self).__init__(context)
        self.setObjectName("rqt_rosparam")

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

        self._widget.findChildren(QPushButton, "refreshButton")[0].clicked.connect(
            self.refresh
        )
        self._param_tree = self._widget.findChildren(QTreeView, "paramTree")[0]
        self._filter_box = self._widget.findChildren(QLineEdit, "filterEntry")[0]
        self._model = QStandardItemModel()

        # Set up ability to sort the fields
        self._sort_model = QSortFilterProxyModel()
        self._sort_model.setSourceModel(self._model)
        self._param_tree.setModel(self._sort_model)

        self._model.setHorizontalHeaderLabels(["Key", "Value"])
        # There are only two sections so they can't be moved anyway
        self._param_tree.header().setSectionsMovable(False)
        self._param_tree.header().setDefaultSectionSize(200)
        self._param_tree.setSortingEnabled(True)

        self.refresh()

        context.add_widget(self._widget)
        self._model.itemChanged.connect(self.model_item_changed)

    def refresh(self):
        # Clear all the rows
        self._model.removeRows(0, self._model.rowCount())
        # Get the ros params
        yaml_params = rosparam.get_param("/")

        root = self._model.invisibleRootItem()

        self._add_dict_to_tree(yaml_params, root)

    def _add_dict_to_tree(self, dict, parent_item):
        """
        Add a dict to a model item, recursively
        :param dict: A dictionary
        :param parent_item: The model item to add the dict to
        :return:
        """
        for key, value in dict.items():
            # The value is a dict, so add the key and recurse
            if isinstance(value, Mapping):
                parent_item.appendRow(QStandardItem(key))
                # Get the row we just added so we can use it as a parent for the dictionary
                new_parent = parent_item.child(parent_item.rowCount() - 1)
                self._add_dict_to_tree(value, new_parent)
            else:
                # Can't handle lists so convert to string
                data_type = type(value)
                value = str(value)
                parent_item.appendRow([QStandardItem(key), QStandardItem(value)])
                # Try to indicate that the actual content of this object is a list, so we can convert it back later
                row = parent_item.child(parent_item.rowCount() - 1)
                row.data_type = data_type

    def model_item_changed(self, item):
        print(item)

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
