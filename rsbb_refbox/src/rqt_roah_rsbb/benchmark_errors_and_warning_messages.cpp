/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB.
 *
 * RoAH RSBB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

#include "benchmark_errors_and_warning_messages.h"

#include <QStringList>
#include <QMessageBox>
#include <QScrollBar>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>

#include <ros_roah_rsbb.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
BenchmarkErrorsAndWarningMessages::BenchmarkErrorsAndWarningMessages() :
		rqt_gui_cpp::Plugin(), widget_(0), nh_(ros::NodeHandle()) {
	setObjectName("BenchmarkErrorsAndWarningMessages");
}

void BenchmarkErrorsAndWarningMessages::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	connect(ui_.clear_button, SIGNAL(released()), this, SLOT(clear()));

	// add widget to the user interface
	context.addWidget(widget_);

	ui_.table->setRowCount(1);
	ui_.table->setItem(0, 0, new QTableWidgetItem(""));
	ui_.table->setItem(0, 1, new QTableWidgetItem(""));
	ui_.table->setItem(0, 2, new QTableWidgetItem(QString::fromStdString("No messages to display")));

	rosout_subscriber_ = nh_.subscribe("/rosout", 1000, &BenchmarkErrorsAndWarningMessages::rosout_callback, this);

}

void BenchmarkErrorsAndWarningMessages::shutdownPlugin() {
	rosout_subscriber_.shutdown();
}

void BenchmarkErrorsAndWarningMessages::rosout_callback(const rosgraph_msgs::Log::ConstPtr& m) {
	Time now = Time::now();

	auto level = m->level;
	string name = m->name;
	string msg = m->msg;

	if (level & (rosgraph_msgs::Log::ERROR | rosgraph_msgs::Log::WARN | rosgraph_msgs::Log::FATAL)) {

		QColor level_color;
		string level_string;
		switch (level) {
		case rosgraph_msgs::Log::WARN:
			level_string = "WARN ";
			level_color = Qt::yellow;
			break;
		case rosgraph_msgs::Log::ERROR:
			level_string = "ERROR";
			level_color = Qt::red;
			break;
		case rosgraph_msgs::Log::FATAL:
			level_string = "FATAL";
			level_color = Qt::red;
			break;
		}

		ui_.table->setRowCount(next_row_index_ + 1);

		QTableWidgetItem* level_item = new QTableWidgetItem(QString::fromStdString(level_string));
		level_item->setBackgroundColor(level_color);

		ui_.table->setItem(next_row_index_, 0, level_item);
		ui_.table->setItem(next_row_index_, 1, new QTableWidgetItem(QString::fromStdString(name)));
		ui_.table->setItem(next_row_index_, 2, new QTableWidgetItem(QString::fromStdString(msg)));

		ui_.table->scrollToBottom();

		next_row_index_++; // = ui_.table->rowCount();

	}
}

void BenchmarkErrorsAndWarningMessages::clear() {

	ui_.table->setRowCount(1);
	ui_.table->setItem(0, 0, new QTableWidgetItem(""));
	ui_.table->setItem(0, 1, new QTableWidgetItem(""));
	ui_.table->setItem(0, 2, new QTableWidgetItem(QString::fromStdString("No messages to display (cleared)")));

	next_row_index_ = 0;
}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, BenchmarkErrorsAndWarningMessages, rqt_roah_rsbb::BenchmarkErrorsAndWarningMessages, rqt_gui_cpp::Plugin)
