import os
import rospy
import rospkg
import rosparam
import yaml

from collections.abc import Mapping
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QTreeView,
    QLineEdit,
    QPushButton,
    QLabel,
)
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtCore import QSortFilterProxyModel
import python_qt_binding.QtCore as QtCore


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

        self._model = QStandardItemModel()

        # Set up the model used for sorting and filtering the fields
        self._sort_model = QSortFilterProxyModel()
        self._sort_model.setSourceModel(self._model)
        # If a child matches, show the parent. TODO: Must also add something to show all children when a parent matches
        self._sort_model.setRecursiveFilteringEnabled(True)

        self._param_tree = self._widget.findChildren(QTreeView, "paramTree")[0]
        self._param_tree.setModel(self._sort_model)

        self._widget.findChildren(QLineEdit, "filterEntry")[0].textChanged.connect(
            self._sort_model.setFilterFixedString
        )

        self._model.setHorizontalHeaderLabels(["Parameter", "Value"])
        # There are only two sections so they can't be moved anyway
        self._param_tree.header().setSectionsMovable(False)
        self._param_tree.header().setDefaultSectionSize(200)
        self._param_tree.setSortingEnabled(True)

        self._widget.findChildren(QPushButton, "refreshButton")[0].clicked.connect(
            self.refresh
        )
        self._widget.findChildren(QPushButton, "setParamButton")[0].clicked.connect(
            self._set_param_from_button
        )
        self._widget.findChildren(QPushButton, "collapseButton")[0].clicked.connect(
            self._param_tree.collapseAll
        )
        self._widget.findChildren(QPushButton, "expandButton")[0].clicked.connect(
            self._param_tree.expandAll
        )
        self._param_name_edit = self._widget.findChildren(QLineEdit, "setParamKeyEdit")[
            0
        ]
        self._param_value_edit = self._widget.findChildren(
            QLineEdit, "setParamValueEdit"
        )[0]
        self._feedback_label = self._widget.findChildren(QLabel, "feedbackLabel")[0]

        self.refresh()

        context.add_widget(self._widget)
        self._model.itemChanged.connect(self._model_item_changed)

    def refresh(self):
        """Refresh the model display by getting yaml parameters from the parameter server"""
        # Clear all the rows
        self._model.removeRows(0, self._model.rowCount())
        # Get the ros params
        yaml_params = rosparam.get_param("/")

        root = self._model.invisibleRootItem()

        self._add_dict_to_tree(yaml_params, root)

    def _add_dict_to_tree(self, value_dict, parent_item):
        """
        Add a dict to a model item, recursively
        :param value_dict: A dictionary
        :param parent_item: The model item to add the dict to
        :return:
        """
        for key, value in value_dict.items():
            # The value is a dict, so add the key and recurse
            if isinstance(value, Mapping):
                new_row = QStandardItem(key)
                parent_item.appendRow(new_row)
                # Get the row we just added so we can use it as a parent for the dictionary
                self._add_dict_to_tree(value, new_row)
            else:
                # Can't handle lists so convert to string
                data_type = type(value)
                value = str(value)
                parent_item.appendRow([QStandardItem(key), QStandardItem(value)])
                # Try to indicate that the actual content of this object is a list, so we can convert it back later
                row = parent_item.child(parent_item.rowCount() - 1)
                row.data_type = data_type

    def _reconstruct_param_name(self, model_index, param_name=""):
        """
        Reconstruct the name of a parameter that is in a subtree

        This is necessary because the tree is constructed such that each namespace is on a new row
        Uses recursion to go up the model tree and add the namespace names of each row

        :param model_index: The model index of the parameter name to reconstruct
        :param param_name: The currently built name of the parameter
        :return: The full name of the parameter
        """
        # The current model index is the name of the namespace which contains the parameter name or sub-namespace
        param_name = self._model.data(model_index, QtCore.Qt.DisplayRole) + (
            "/" + param_name if param_name else ""
        )

        # If the item has no parent, it is a top level namespace, so the param name construction is complete
        parent = self._model.itemFromIndex(model_index).parent()
        if not parent:
            return param_name
        else:
            # Otherwise, we continue with the reconstruction by passing the parent index
            return self._reconstruct_param_name(parent.index(), param_name)

    def _model_item_changed(self, item):
        """Called when the user modifies a parameter value"""
        # Must use the DisplayRole otherwise just returns none
        new_value = item.data(QtCore.Qt.DisplayRole)
        # Get the parameter name by getting the modelIndex of the sibling on this row at column 0, which is the
        # parameter column
        parameter = self._reconstruct_param_name(item.index().siblingAtColumn(0))
        self._set_param(parameter, new_value)

    def _set_feedback(self, message):
        """
        Set the message to display in the feedback label
        :param message: The message to display
        :return:
        """
        self._feedback_label.setText(message)

    def _set_param(self, parameter, value):
        try:
            rospy.set_param(parameter, value)
        except TypeError as e:
            message = "Failed to set param {}: {}".format(parameter, e)
            rospy.logerr(message)
            self._set_feedback(message)

    def _set_param_from_button(self):
        # If there is no param name set we can't do anything
        parameter = self._param_name_edit.text()
        if parameter:
            self._set_param(parameter, self._param_value_edit.text())
            self._add_dict_to_tree(
                {parameter: self._param_value_edit.text()},
                self._model.invisibleRootItem(),
            )
