#!/usr/bin/env python
from copy import deepcopy
import sys
import signal
import queue

from PyQt5.QtWidgets import QWidget, QTreeView, QApplication, QCheckBox, QPushButton
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtCore import pyqtSignal
from threading import Thread, Lock

import rospy
from visualization_msgs.msg import Marker, MarkerArray


class RVizRepeater(QWidget):
    marker_arrived_signal = pyqtSignal()

    def __init__(self):
        super(RVizRepeater, self).__init__()
        self.title = 'RViz Marker Repeater and Muter'
        self.NAMESPACE_IDX = 0
        self.ID_IDX = 0
        self.ENABLE_IDX = 1
        self.DELETE_IDX = 2

        self.left = 10
        self.top = 10
        self.width = 450
        self.height = 600

        self.data_mtx = Lock()
        # Used to keep pointers into various parts of the data structure
        # Format is: {namespace: [ns_item, ns_checkbox, {id: id_item, id_checkbox, marker}]}
        self.namespaces_map = {}

        self.initUI()

        self.initPubSub()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        rows = 0
        cols = max(self.NAMESPACE_IDX, self.ID_IDX, self.ENABLE_IDX, self.DELETE_IDX)
        self.model = QStandardItemModel(rows, cols, self)
        self.model.setHorizontalHeaderLabels(["Namespace/ID", "Enabled", "Delete"])

        self.view = QTreeView(self)
        self.view.setModel(self.model)
        self.view.setAlternatingRowColors(True)
        self.view.setSortingEnabled(True)
        self.view.setColumnWidth(self.NAMESPACE_IDX, 348)
        self.view.setColumnWidth(self.ENABLE_IDX, 25)
        self.view.setColumnWidth(self.DELETE_IDX, 75)
        self.view.resize(self.size())

        self.show()

    def resizeEvent(self, event):
        """ Resize all sections to content and user interactive """
        super(RVizRepeater, self).resizeEvent(event)
        self.view.resize(self.size())

    def _add_namespace(self, namespace):
        """
        :type namespace str:
        """
        if namespace not in self.namespaces_map.keys():
            # Create an item with a caption
            ns_item = QStandardItem(namespace)
            ns_item.setEditable(False)

            row = self.model.rowCount()
            self.model.setItem(row, self.NAMESPACE_IDX, ns_item)

            enabled_box = QCheckBox()
            enabled_box.setChecked(True)
            enabled_box.setTristate(True)
            enabled_box.clicked.connect(lambda: self._click_enable_ns(enabled_box, namespace))
            # Get the index into the location in the model that we want to put the CheckBox into
            enabled_box_index = self.model.index(row, self.ENABLE_IDX)
            # Update the view to include this CheckBox in the correct index of the model
            self.view.setIndexWidget(enabled_box_index, enabled_box)

            delete_button = QPushButton()
            delete_button.setText("Delete")
            delete_button.clicked.connect(lambda: self._delete_namespace(namespace))
            # Get the index into the location in the model that we want to put the CheckBox into
            delete_button_index = self.model.index(row, self.DELETE_IDX)
            # Update the view to include this PushButton in the correct index of the model
            self.view.setIndexWidget(delete_button_index, delete_button)

            # Record pointers to data items so that we can lookup by namespace
            self.namespaces_map[namespace] = [ns_item, enabled_box, {}]

    def _add_id_in_namespace(self, namespace, id):
        """
        :type namespace str:
        :type id int:
        """
        # Create the namespace if needed
        self._add_namespace(namespace)
        ns_item, ns_enabled_box, ids_map = self.namespaces_map[namespace]

        if id not in ids_map.keys():
            id_item = QStandardItem(str(id))
            id_item.setEditable(False)

            # Append 3 items to allow for indexing into the extra locations
            ns_item.appendRow([id_item, QStandardItem(), QStandardItem()])

            enabled_box = QCheckBox()
            # Set the state of the checkbox based on the state of the corresponding namespace checkbox
            if ns_enabled_box.checkState() == 0:
                enabled_box.setChecked(False)
            else:
                enabled_box.setChecked(True)
            enabled_box.clicked.connect(lambda: self._click_enable_ns_id(enabled_box, namespace, id))
            # Get the index into the location in the model that we want to put the CheckBox into
            enable_checkbox_index = ns_item.child(id_item.row(), self.ENABLE_IDX).index()
            # Update the view to include this CheckBox in the correct index of the model
            self.view.setIndexWidget(enable_checkbox_index, enabled_box)

            delete_button = QPushButton()
            delete_button.setText("Delete")
            delete_button.clicked.connect(lambda: self._delete_marker(namespace, id))
            # Get the index into the location in the model that we want to put the PushButton into
            delete_button_index = ns_item.child(id_item.row(), self.DELETE_IDX).index()
            # Update the view to include this PushButton in the correct index of the model
            self.view.setIndexWidget(delete_button_index, delete_button)

            # Record pointers to the data items so that we can lookup by id
            ids_map[id] = [id_item, enabled_box, Marker()]

    def _add_marker(self, marker):
        """
        :type marker Marker
        """

        if marker.action == Marker.DELETE:
            self._delete_marker_internal(marker.ns, marker.id)
        elif marker.action == Marker.DELETEALL:
            self._delete_namespace_internal(marker.ns)
        else:
            # Add the namespace to the model list if needed
            self._add_namespace(marker.ns)

            # Add the marker to the sublists if needed
            self._add_id_in_namespace(marker.ns, marker.id)

            # Copy the marker into the correct location in the data structure
            # Format is: {namespace: [ns_item, ns_checkbox, {id: id_item, id_checkbox, marker}]}
            _, _, ids_map = self.namespaces_map[marker.ns]
            data = ids_map[marker.id]
            data[2] = deepcopy(marker)

    def _delete_namespace_internal(self, namespace):
        try:
            ns_item, _, _ = self.namespaces_map[namespace]
            self.model.removeRow(ns_item.row())
            del self.namespaces_map[namespace]
        except:
            pass

    def _delete_marker_internal(self, namespace, id):
        try:
            ns_item, _, ids_map = self.namespaces_map[namespace]
            id_item, _, _ = ids_map[id]

            ns_item.removeRow(id_item.row())
            del ids_map[id]
        except:
            pass

    def _delete_namespace(self, namespace):
        with self.data_mtx:
            self._delete_namespace_internal(namespace)

    def _delete_marker(self, namespace, id):
        with self.data_mtx:
            self._delete_marker_internal(namespace, id)

    def _click_enable_ns(self, checkbox, namespace):
        """
        :type checkbox QCheckBox
        :type namespace str
        """
        with self.data_mtx:
            ns_item, ns_checkbox, ids_map = self.namespaces_map[namespace]

            # Don't allow the checkbox to be clicked to the partial state, instead bypass directly to fully checked
            if ns_checkbox.checkState() == 1:
                ns_checkbox.setCheckState(2)

            if ns_checkbox.isChecked():
                for id, value in ids_map.items():
                    value[1].setChecked(True)
            else:
                for id, value in ids_map.items():
                    value[1].setChecked(False)

    def _click_enable_ns_id(self, checkbox, namespace, id):
        """
        :type checkbox QCheckBox
        :type namespace str
        :type id int
        """
        with self.data_mtx:
            ns_item, ns_checkbox, ids_map = self.namespaces_map[namespace]
            id_item, id_checkbox, marker = ids_map[id]

            assert (id_checkbox is checkbox)
            assert (marker.ns == namespace)
            assert (marker.id == id)

            if id_checkbox.isChecked():
                # If the namespace's box is not checked, then partially check it
                if ns_checkbox.checkState() == 0:
                    ns_checkbox.setCheckState(1)
            else:
                # If the namespace's box is checked, then partially check it
                if ns_checkbox.checkState() == 2:
                    ns_checkbox.setCheckState(1)

    def initPubSub(self):
        self.marker_queue = queue.Queue()
        self.marker_arrived_signal.connect(self.marker_slot)

        self.marker_sub = rospy.Subscriber("visualization_marker", Marker, self.marker_callback)
        self.marker_array_sub = rospy.Subscriber("visualization_marker_vector", MarkerArray, self.marker_array_callback,
                                                 queue_size=10, buff_size=2 ** 24)

        self.publish_thread = Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def marker_slot(self):
        """
        :type marker Marker
        """
        with self.data_mtx:
            while not self.marker_queue.empty():
                try:
                    marker = self.marker_queue.get_nowait()
                    self._add_marker(marker)
                except queue.Empty:
                    pass

    def marker_callback(self, marker):
        """
        :type marker Marker
        """
        self.marker_queue.put(marker)
        self.marker_arrived_signal.emit(marker)

    def marker_array_callback(self, marker_array):
        """
        :type marker_array MarkerArray
        """
        for marker in marker_array.markers:
            self.marker_queue.put(marker)
        self.marker_arrived_signal.emit()

    def publish_loop(self):
        marker_array_pub = rospy.Publisher("visualization_marker_vector", MarkerArray, queue_size=1)
        rate = rospy.Rate(hz=2.0)

        while not rospy.is_shutdown():
            with self.data_mtx:
                m = MarkerArray()
                for _, ns_value in self.namespaces_map.items():
                    _, _, ids_map = ns_value
                    for _, id_value in ids_map.items():
                        _, id_checkbox, marker = id_value
                        if id_checkbox.isChecked():
                            m.markers.append(marker)
            marker_array_pub.publish(m)
            rate.sleep()
