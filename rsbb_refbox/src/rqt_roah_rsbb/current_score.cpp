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

#include "current_score.h"

#include <QStringList>
#include <QMessageBox>
#include <QScrollBar>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>

#include <ros_roah_rsbb.h>

#include <roah_rsbb/Zone.h>

#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
CurrentScore::CurrentScore() :
		rqt_gui_cpp::Plugin(), widget_(0) {
	setObjectName("CurrentScore");
}

void CurrentScore::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	// add widget to the user interface
	context.addWidget(widget_);

	result_rcv_.start("/rsbb/current_benchmark_result", getNodeHandle());

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(200);
}

void CurrentScore::shutdownPlugin() {
	update_timer_.stop();
	result_rcv_.stop();
}

void CurrentScore::update() {
//    auto core = result_rcv_.last ();
//    string current_zone = param_direct ("current_zone", string());
//
//    if (core) {
//      for (roah_rsbb::ZoneState const& zone : core->zones) {
//        if (zone.zone == current_zone) {
//          QString new_text = QString::fromStdString (zone.log);
//          if (ui_.display->toPlainText() != new_text) {
//            ui_.display->setPlainText (new_text);
//            QScrollBar* sb = ui_.display->verticalScrollBar();
//            sb->setValue (sb->maximum());
//          }
//          return;
//        }
//      }
//    }

	std_msgs::String::ConstPtr current_result_msg = result_rcv_.last();
	if(current_result_msg) {
		ui_.display->clear();
		ui_.display->setPlainText(QString::fromStdString(current_result_msg->data));
	}

}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, CurrentScore, rqt_roah_rsbb::CurrentScore, rqt_gui_cpp::Plugin)